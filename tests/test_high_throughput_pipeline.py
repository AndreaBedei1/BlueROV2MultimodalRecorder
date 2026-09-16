"""Hardware-independent bounded pipeline and numerical regression tests."""

from __future__ import annotations

import json
import queue
import struct
import tempfile
import time
from pathlib import Path

import numpy as np

from bluerov_recorder.processing import (
    ATOF_HEADER,
    CHANNEL_DATA_HEADER,
    END_PING,
    PacketFramingDiagnostics,
    PingProtocolFramer,
    beamform_surveyor_channels,
    beamform_surveyor_channels_reference,
    iter_svlog_packets,
    make_packet,
)
from bluerov_recorder.runtime import BufferedBinaryWriter, LatestValueMailbox, MetricsRegistry
from bluerov_recorder.surveyor_pipeline import SurveyorDecoderWorker


def channel_payload(ping_number: int, ch1: int, ch2: int, bins: int = 32) -> bytes:
    values = []
    for channel in (ch1, ch2):
        for index in range(bins):
            values.extend((float(channel + 1) * (index + 1), float(index) * 0.25))
    return CHANNEL_DATA_HEADER.pack(
        ping_number, 1.0, 0, 0, 100, ch1, ch2, bins,
    ) + struct.pack("<%df" % len(values), *values)


def end_payload(ping_number: int, bins: int = 32) -> bytes:
    values = list(END_PING.unpack(bytes(END_PING.size)))
    values[1] = 0.0
    values[2] = 20.0
    values[6] = ping_number
    values[13] = 240000.0
    values[16] = bins
    values[17] = 1
    values[-1] = 1_700_000_000_000
    return END_PING.pack(*values)


def atof_payload(ping_number: int) -> bytes:
    return ATOF_HEADER.pack(
        0, 1_700_000_000_000, 0.1, 1500.0, ping_number,
        240000, 0.001, 0, 0, 0,
    )


def test_latest_mailbox_never_exceeds_one_and_counts_preview_replacement():
    metrics = MetricsRegistry()
    mailbox = LatestValueMailbox(metrics, "preview_dropped", "preview_published")
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    for index in range(60):
        mailbox.publish((index, frame))
        assert len(mailbox) == 1
    sequence, (index, returned) = mailbox.take()
    assert sequence == 60
    assert index == 59
    assert returned is frame
    assert len(mailbox) == 0
    snapshot = metrics.snapshot()
    assert snapshot["preview_published"] == 60
    assert snapshot["preview_dropped"] == 59


def test_buffered_raw_writer_preserves_every_accepted_byte_in_order():
    metrics = MetricsRegistry()
    packets = [make_packet(3010, end_payload(index)) for index in range(2000)]
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "raw.svlog"
        writer = BufferedBinaryWriter(
            path, metrics, "surveyor_raw", max_queue_bytes=2 * 1024 * 1024,
            batch_bytes=64 * 1024, flush_interval_s=0.02,
        )
        writer.start()
        for packet in packets:
            writer.submit(packet)
        writer.close()
        assert path.read_bytes() == b"".join(packets)
    snapshot = metrics.snapshot()
    assert snapshot["surveyor_packets_written"] == len(packets)
    assert snapshot.get("app_raw_drop_count", 0) == 0
    assert snapshot["surveyor_raw_queue_bytes_high_watermark"] <= 2 * 1024 * 1024


def test_checksum_parser_recovers_valid_packet_after_corruption():
    first = make_packet(3010, end_payload(1))
    corrupt = bytearray(make_packet(3010, end_payload(2)))
    corrupt[-1] ^= 0xFF
    final = make_packet(3010, end_payload(3))
    framer = PingProtocolFramer(validate_checksum=True)
    decoded = []
    stream = first + bytes(corrupt) + b"garbage" + final
    for offset in range(0, len(stream), 17):
        decoded.extend(framer.feed(stream[offset : offset + 17]))
    assert [END_PING.unpack(item[1])[6] for item in decoded] == [1, 3]
    assert framer.diagnostics.checksum_invalid >= 1
    assert framer.diagnostics.framing_resync_count >= 1


def test_vector_beamformer_matches_scalar_reference():
    channels = {}
    bins = 64
    for channel in range(16):
        values = []
        for index in range(bins):
            values.extend((np.sin(index * 0.1 + channel), np.cos(index * 0.07 - channel)))
        channels[channel] = values
    reference = np.asarray(beamform_surveyor_channels_reference(channels, 0.0, 20.0))
    vectorized = np.asarray(beamform_surveyor_channels(channels, 0.0, 20.0))
    np.testing.assert_allclose(vectorized, reference, rtol=1e-5, atol=1e-4)


def test_decoder_accounts_complete_incomplete_and_reordered_terminal_messages():
    metrics = MetricsRegistry()
    records = []
    worker = SurveyorDecoderWorker(
        LatestValueMailbox(metrics), LatestValueMailbox(metrics),
        queue.Queue(maxsize=128), metrics, records.append,
        configured_ping_rate_hz=5.0,
    )
    worker.start()
    sequence = []
    for ch1 in range(0, 16, 2):
        sequence.append((3009, channel_payload(10, ch1, ch1 + 1)))
    # The next ping starts before END/ATOF for ping 10, matching the real log.
    sequence.append((3009, channel_payload(11, 0, 1)))
    sequence.append((3010, end_payload(10)))
    sequence.append((3012, atof_payload(10)))
    sequence.append((3010, end_payload(11)))
    sequence.append((3012, atof_payload(11)))
    for message_id, payload in sequence:
        packet = make_packet(message_id, payload)
        worker.submit((message_id, payload, packet, time.monotonic_ns(), time.time_ns()))
    worker.close()
    by_ping = {item["ping_number"]: item for item in records}
    assert set(by_ping) == {10, 11}
    assert by_ping[10]["complete_16_channels"] is True
    assert by_ping[10]["has_end_ping"] is True
    assert by_ping[10]["has_atof"] is True
    assert by_ping[11]["complete_16_channels"] is False
    assert by_ping[11]["channel_packets_received"] == 1
    snapshot = metrics.snapshot()
    assert snapshot["surveyor_complete_channel_pings"] == 1
    assert snapshot["surveyor_incomplete_channel_pings"] == 1


def test_validating_svlog_exposes_diagnostics_without_changing_raw_file():
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "test.svlog"
        raw = make_packet(3010, end_payload(77))
        path.write_bytes(raw)
        diagnostics = PacketFramingDiagnostics()
        packets = list(iter_svlog_packets(path, diagnostics, validate_checksum=True))
        assert packets[0][2] == raw
        assert diagnostics.checksum_valid == 1
        assert diagnostics.checksum_invalid == 0
        assert path.read_bytes() == raw
