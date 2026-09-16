"""Shared, protocol-confirmed Surveyor 240-16 processing helpers.

This module deliberately contains no device-control code.  It is shared by
the offline log explorer and the live/replay recorder so that there is one
implementation of the confirmed 3009 channel-data decoder and Surveyor fan
beamformer.
"""

from __future__ import annotations

import math
import mmap
import struct
from functools import lru_cache
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple, Union

try:
    import numpy as np
except ImportError:  # The reference path keeps log inspection usable.
    np = None


PACKET_HEADER = struct.Struct("<BBHHBB")
PACKET_CHECKSUM = struct.Struct("<H")
CHANNEL_DATA_HEADER = struct.Struct("<IfBxHiBBH")

MSG_JSON = 10
MSG_ATTITUDE = 504
MSG_RAW_PROFILE = 3009
MSG_END_PING = 3010
MSG_ATOF = 3012

SURVEYOR_CHANNEL_COUNT = 16
SURVEYOR_PACKET_COUNT = 8
SURVEYOR_ACOUSTIC_HZ = 240000.0
SURVEYOR_FOV_DEG = (-40.0, 40.0)
CHANNEL_SPACING_M = 0.136 * 0.0254

END_PING = struct.Struct("<IfffffIfffffffiHHHBBIQ")
ATOF_HEADER = struct.Struct("<IQffIIfIHH")
ATTITUDE = struct.Struct("<ffffffQI")


@dataclass
class PacketFramingDiagnostics:
    bytes_received: int = 0
    packets_framed: int = 0
    checksum_valid: int = 0
    checksum_invalid: int = 0
    framing_resync_count: int = 0
    framing_discarded_bytes: int = 0
    unknown_message_count: int = 0

    def as_dict(self) -> Dict[str, int]:
        return {
            "bytes_received": self.bytes_received,
            "packets_framed": self.packets_framed,
            "checksum_valid": self.checksum_valid,
            "checksum_invalid": self.checksum_invalid,
            "framing_resync_count": self.framing_resync_count,
            "framing_discarded_bytes": self.framing_discarded_bytes,
            "unknown_message_count": self.unknown_message_count,
        }


class PingProtocolFramer:
    """Incremental Ping Protocol framer with explicit corruption accounting.

    Raw socket chunks must be persisted before calling :meth:`feed`; this
    parser may discard malformed bytes only from its private decode buffer.
    """

    KNOWN_MESSAGES = {MSG_JSON, MSG_ATTITUDE, MSG_RAW_PROFILE, MSG_END_PING, MSG_ATOF, 14}

    def __init__(self, validate_checksum: bool = True, accept_invalid: bool = False) -> None:
        self.buffer = bytearray()
        self.validate_checksum = bool(validate_checksum)
        self.accept_invalid = bool(accept_invalid)
        self.diagnostics = PacketFramingDiagnostics()

    @staticmethod
    def checksum_ok(packet: bytes) -> bool:
        if len(packet) < PACKET_HEADER.size + PACKET_CHECKSUM.size:
            return False
        expected = PACKET_CHECKSUM.unpack_from(packet, len(packet) - PACKET_CHECKSUM.size)[0]
        return (sum(packet[:-PACKET_CHECKSUM.size]) & 0xFFFF) == expected

    def feed(self, chunk: bytes) -> List[Tuple[int, bytes, bytes]]:
        payload_chunk = bytes(chunk)
        self.diagnostics.bytes_received += len(payload_chunk)
        self.buffer.extend(payload_chunk)
        packets: List[Tuple[int, bytes, bytes]] = []
        while True:
            start = self.buffer.find(b"BR")
            if start < 0:
                if len(self.buffer) > 1:
                    discarded = len(self.buffer) - 1
                    del self.buffer[:-1]
                    self.diagnostics.framing_resync_count += 1
                    self.diagnostics.framing_discarded_bytes += discarded
                break
            if start:
                del self.buffer[:start]
                self.diagnostics.framing_resync_count += 1
                self.diagnostics.framing_discarded_bytes += start
            if len(self.buffer) < PACKET_HEADER.size:
                break
            try:
                sync_a, sync_b, payload_len, message_id, _src, _dst = PACKET_HEADER.unpack_from(self.buffer)
            except struct.error:
                break
            if sync_a != 66 or sync_b != 82:
                del self.buffer[0]
                self.diagnostics.framing_resync_count += 1
                self.diagnostics.framing_discarded_bytes += 1
                continue
            total = PACKET_HEADER.size + int(payload_len) + PACKET_CHECKSUM.size
            if len(self.buffer) < total:
                break
            packet = bytes(self.buffer[:total])
            valid = self.checksum_ok(packet)
            if valid:
                self.diagnostics.checksum_valid += 1
            else:
                self.diagnostics.checksum_invalid += 1
            if self.validate_checksum and not valid and not self.accept_invalid:
                # Shift one byte rather than trusting a corrupt length.  The
                # next iteration safely searches for the following sync word.
                del self.buffer[0]
                self.diagnostics.framing_resync_count += 1
                self.diagnostics.framing_discarded_bytes += 1
                continue
            del self.buffer[:total]
            payload_start = PACKET_HEADER.size
            payload = packet[payload_start : payload_start + int(payload_len)]
            message_id = int(message_id)
            self.diagnostics.packets_framed += 1
            if message_id not in self.KNOWN_MESSAGES:
                self.diagnostics.unknown_message_count += 1
            packets.append((message_id, payload, packet))
        return packets


def make_packet(packet_id: int, payload: bytes) -> bytes:
    """Build one checksum-bearing Ping Protocol packet."""
    header = PACKET_HEADER.pack(66, 82, len(payload), int(packet_id), 0, 0)
    body = header + bytes(payload)
    return body + PACKET_CHECKSUM.pack(sum(body) & 0xFFFF)


def iter_svlog_packets(
    path: Path,
    diagnostics: Optional[PacketFramingDiagnostics] = None,
    validate_checksum: bool = False,
) -> Iterator[Tuple[int, bytes, bytes]]:
    """Yield ``(message_id, payload, complete_packet)`` from a ``.svlog``.

    The parser is intentionally conservative: it follows the Ping Protocol
    framing and checksum position, but does not decode unknown message bodies.
    This lets recordings retain future/unknown packets without inventing a
    schema for them.
    """
    path = Path(path)
    framer = PingProtocolFramer(validate_checksum=validate_checksum, accept_invalid=not validate_checksum)
    with Path(path).open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            for packet in framer.feed(chunk):
                yield packet
    if diagnostics is not None:
        state = framer.diagnostics
        diagnostics.bytes_received = state.bytes_received
        diagnostics.packets_framed = state.packets_framed
        diagnostics.checksum_valid = state.checksum_valid
        diagnostics.checksum_invalid = state.checksum_invalid
        diagnostics.framing_resync_count = state.framing_resync_count
        diagnostics.framing_discarded_bytes = state.framing_discarded_bytes
        diagnostics.unknown_message_count = state.unknown_message_count


def parse_channel_data_header(payload: bytes) -> Optional[Dict[str, Union[int, float]]]:
    """Decode the confirmed 3009 ``ChPairGoertzelData`` header."""
    if len(payload) < CHANNEL_DATA_HEADER.size:
        return None
    try:
        ping_number, analog_gain, device_number, device_index, adc_pp_signal, ch1, ch2, results = CHANNEL_DATA_HEADER.unpack_from(payload)
    except struct.error:
        return None
    return {
        "ping_number": int(ping_number),
        "analog_gain": float(analog_gain),
        "device_number": int(device_number),
        "device_index_unused": int(device_index),
        "adc_pp_signal": int(adc_pp_signal),
        "ch1": int(ch1),
        "ch2": int(ch2),
        "results_per_channel": int(results),
    }


def channel_sample_bytes(results_per_channel: int) -> int:
    """Return bytes used by the protocol-defined 3009 IQ sample area."""
    return 4 * int(results_per_channel) * 4


def validate_channel_payloads(
    payloads: Sequence[bytes],
    expected_ping: Optional[int] = None,
    expected_packet_count: int = SURVEYOR_PACKET_COUNT,
) -> Tuple[bool, str, Dict[int, List[float]], int]:
    """Validate and decode one complete 16-channel 3009 ping.

    The returned channel data contains exactly ``2 * n`` floats per channel,
    ordered as ``[I0, Q0, I1, Q1, ...]``.  Bytes after the first ``4*n``
    Float32 values are ignored because the official SonarView parser ignores
    them too.
    """
    headers = []
    decoded: Dict[int, List[float]] = {}
    for payload in payloads:
        header = parse_channel_data_header(payload)
        if header is None:
            return False, "invalid 3009 header", {}, 0
        ping_number = int(header["ping_number"])
        if expected_ping is not None and ping_number != int(expected_ping):
            return False, "ping mismatch in 3009", {}, 0
        ch1 = int(header["ch1"])
        ch2 = int(header["ch2"])
        results = int(header["results_per_channel"])
        if not (0 <= ch1 < SURVEYOR_CHANNEL_COUNT and 0 <= ch2 < SURVEYOR_CHANNEL_COUNT and ch1 != ch2):
            return False, "invalid channel indices", {}, 0
        if results <= 0:
            return False, "invalid results_per_channel", {}, 0
        expected = CHANNEL_DATA_HEADER.size + channel_sample_bytes(results)
        if len(payload) < expected:
            return False, "short 3009 sample area", {}, 0
        try:
            values = struct.unpack_from("<%df" % (4 * results), payload, CHANNEL_DATA_HEADER.size)
        except struct.error:
            return False, "invalid Float32 IQ sample area", {}, 0
        decoded[ch1] = list(values[: 2 * results])
        decoded[ch2] = list(values[2 * results : 4 * results])
        headers.append(header)

    if len(headers) != int(expected_packet_count):
        return False, "incomplete channel packet set", {}, 0
    pairs = {(int(item["ch1"]), int(item["ch2"])) for item in headers}
    channels = set(decoded)
    results_set = {int(item["results_per_channel"]) for item in headers}
    ping_numbers = {int(item["ping_number"]) for item in headers}
    if ping_numbers != ({int(expected_ping)} if expected_ping is not None else ping_numbers):
        return False, "inconsistent ping numbers", {}, 0
    if channels != set(range(SURVEYOR_CHANNEL_COUNT)) or len(pairs) != expected_packet_count:
        return False, "incomplete 16-channel set", {}, 0
    if len(results_set) != 1:
        return False, "inconsistent results_per_channel", {}, 0
    return True, "16 channels · Float32 IQ", decoded, next(iter(results_set))


class SurveyorChannelAccumulator:
    """Collect 3009 channel pairs until the matching 3010 message arrives."""

    def __init__(self) -> None:
        self.reset()

    def reset(self, ping_number: Optional[int] = None) -> None:
        self.ping_number = ping_number
        self.payloads: List[bytes] = []

    def add(self, payload: bytes) -> bool:
        header = parse_channel_data_header(payload)
        if header is None:
            return False
        ping_number = int(header["ping_number"])
        if self.ping_number is None:
            self.ping_number = ping_number
        if ping_number != self.ping_number:
            return False
        self.payloads.append(bytes(payload))
        return True

    def complete(self) -> bool:
        if self.ping_number is None:
            return False
        valid, _note, _channels, _bins = validate_channel_payloads(self.payloads, self.ping_number)
        return valid

    def decode(self) -> Optional[Tuple[Dict[int, List[float]], int, str]]:
        if self.ping_number is None:
            return None
        valid, note, channels, bins = validate_channel_payloads(self.payloads, self.ping_number)
        if not valid:
            return None
        return channels, bins, note


def _range_compensation(start_m: float, end_m: float, bins: int) -> List[float]:
    end_m = max(float(end_m), float(start_m) + 1e-6)
    if end_m <= 8.0:
        exponent = 0.0
    elif end_m >= 20.0:
        exponent = 2.0
    else:
        exponent = (end_m - 8.0) / 12.0 * 2.0
    ratio = max(0.0, min(1.0, float(start_m) / end_m))
    return [
        (ratio + (1.0 - ratio) * index / max(1, bins - 1)) ** exponent
        for index in range(bins)
    ]


def beamform_surveyor_channels_reference(
    channel_signals: Dict[int, Sequence[float]],
    start_m: float,
    end_m: float,
    sos_mps: float = 1500.0,
    threshold_percent: float = 0.0,
    angle_min_deg: float = SURVEYOR_FOV_DEG[0],
    angle_max_deg: float = SURVEYOR_FOV_DEG[1],
) -> List[List[float]]:
    """Original scalar implementation retained as a regression oracle."""
    if set(channel_signals) != set(range(SURVEYOR_CHANNEL_COUNT)):
        raise ValueError("16 Surveyor channels required")
    lengths = {len(channel_signals[index]) for index in range(SURVEYOR_CHANNEL_COUNT)}
    if len(lengths) != 1 or not lengths or next(iter(lengths)) < 2 or next(iter(lengths)) % 2:
        raise ValueError("channel IQ arrays must have an equal even length")
    bins = next(iter(lengths)) // 2
    compensation = _range_compensation(start_m, end_m, bins)
    compensated: Dict[int, List[float]] = {}
    for channel in range(SURVEYOR_CHANNEL_COUNT):
        source = channel_signals[channel]
        values: List[float] = []
        for index, value in enumerate(source):
            values.append(float(value) * compensation[index // 2])
        compensated[channel] = values

    range_power = [
        sum(
            compensated[channel][2 * index] ** 2
            + compensated[channel][2 * index + 1] ** 2
            for channel in range(SURVEYOR_CHANNEL_COUNT)
        )
        for index in range(bins)
    ]
    threshold_percent = max(0.0, min(100.0, float(threshold_percent)))
    threshold = None
    if threshold_percent > 0.0:
        ordered = sorted(range_power)
        threshold = ordered[int(round((len(ordered) - 1) * threshold_percent / 100.0))]

    channel_positions = [
        index * CHANNEL_SPACING_M - 0.5 * (SURVEYOR_CHANNEL_COUNT - 1) * CHANNEL_SPACING_M
        for index in range(SURVEYOR_CHANNEL_COUNT)
    ]
    sos_mps = float(sos_mps or 1500.0)
    wavelength_m = sos_mps / SURVEYOR_ACOUSTIC_HZ
    beam_angles = [math.radians(angle) for angle in range(int(angle_min_deg), int(angle_max_deg) + 1)]
    delays = []
    for angle in beam_angles:
        row = []
        for position in channel_positions:
            phase = math.sin(angle) / wavelength_m * 2.0 * math.pi * position
            row.append((math.cos(phase), math.sin(phase)))
        delays.append(row)

    matrix = [[0.0] * bins for _ in beam_angles]
    for range_index, power in enumerate(range_power):
        if threshold is not None and power < threshold:
            continue
        for beam_index, beam_delays in enumerate(delays):
            real_sum = 0.0
            imag_sum = 0.0
            for channel, (cos_phase, sin_phase) in enumerate(beam_delays):
                real = compensated[channel][2 * range_index]
                imag = compensated[channel][2 * range_index + 1]
                real_sum += real * cos_phase - imag * sin_phase
                imag_sum += imag * cos_phase + real * sin_phase
            matrix[beam_index][range_index] = real_sum * real_sum + imag_sum * imag_sum
    return matrix


@lru_cache(maxsize=32)
def _steering_matrix(sos_mps: float, angle_min_deg: int, angle_max_deg: int):
    if np is None:
        return None
    positions = (
        np.arange(SURVEYOR_CHANNEL_COUNT, dtype=np.float32)
        - np.float32(0.5 * (SURVEYOR_CHANNEL_COUNT - 1))
    ) * np.float32(CHANNEL_SPACING_M)
    angles = np.deg2rad(
        np.arange(int(angle_min_deg), int(angle_max_deg) + 1, dtype=np.float32)
    )
    wavelength = np.float32(float(sos_mps) / SURVEYOR_ACOUSTIC_HZ)
    phases = np.sin(angles)[:, None] / wavelength * np.float32(2.0 * math.pi) * positions[None, :]
    return np.exp(1j * phases).astype(np.complex64)


def beamform_surveyor_channels(
    channel_signals: Dict[int, Sequence[float]],
    start_m: float,
    end_m: float,
    sos_mps: float = 1500.0,
    threshold_percent: float = 0.0,
    angle_min_deg: float = SURVEYOR_FOV_DEG[0],
    angle_max_deg: float = SURVEYOR_FOV_DEG[1],
) -> List[List[float]]:
    """Vectorized 16-channel IQ beamformer preserving the scalar convention.

    ``threshold_percent=0`` is intentionally the default: it preserves the
    background.  A positive value removes only range columns whose mean
    channel power is below that percentile, as a display option.
    """
    if np is None:
        return beamform_surveyor_channels_reference(
            channel_signals, start_m, end_m, sos_mps, threshold_percent,
            angle_min_deg, angle_max_deg,
        )
    if set(channel_signals) != set(range(SURVEYOR_CHANNEL_COUNT)):
        raise ValueError("16 Surveyor channels required")
    lengths = {len(channel_signals[index]) for index in range(SURVEYOR_CHANNEL_COUNT)}
    if len(lengths) != 1 or not lengths or next(iter(lengths)) < 2 or next(iter(lengths)) % 2:
        raise ValueError("channel IQ arrays must have an equal even length")
    bins = next(iter(lengths)) // 2
    packed = np.asarray(
        [channel_signals[index] for index in range(SURVEYOR_CHANNEL_COUNT)],
        dtype=np.float32,
    ).reshape(SURVEYOR_CHANNEL_COUNT, bins, 2)
    iq = packed[:, :, 0].astype(np.complex64)
    iq += np.complex64(1j) * packed[:, :, 1]
    compensation = np.asarray(_range_compensation(start_m, end_m, bins), dtype=np.float32)
    iq *= compensation[None, :]
    range_power = np.sum(np.abs(iq) ** 2, axis=0)
    threshold_percent = max(0.0, min(100.0, float(threshold_percent)))
    keep = None
    if threshold_percent > 0.0:
        ordered = np.sort(range_power)
        index = int(round((len(ordered) - 1) * threshold_percent / 100.0))
        keep = range_power >= ordered[index]

    steering = _steering_matrix(
        round(float(sos_mps or 1500.0), 6), int(angle_min_deg), int(angle_max_deg)
    )
    beam_complex = steering @ iq
    power = (beam_complex.real * beam_complex.real + beam_complex.imag * beam_complex.imag).astype(np.float32)
    if keep is not None:
        power[:, ~keep] = 0.0
    return power.tolist()


def decode_atof_payload(payload: bytes) -> Optional[Dict[str, object]]:
    """Decode the confirmed fixed ATOF header and its angle/TOF points."""
    if len(payload) < ATOF_HEADER.size:
        return None
    try:
        pwr_up_msec, utc_msec, listening_sec, sos, ping_number, ping_hz, pulse_sec, flags, count, reserved = ATOF_HEADER.unpack_from(payload)
    except struct.error:
        return None
    available = min(int(count), (len(payload) - ATOF_HEADER.size) // 16)
    points = []
    for index in range(available):
        angle, tof, reserved_a, reserved_b = struct.unpack_from("<ffII", payload, ATOF_HEADER.size + index * 16)
        if -math.pi / 2.0 <= angle <= math.pi / 2.0 and 0.0 <= tof <= 10.0:
            points.append({
                "angle_rad": float(angle),
                "tof_s": float(tof),
                "distance_m": float(tof) * float(sos or 1500.0) * 0.5,
                "reserved_a": int(reserved_a),
                "reserved_b": int(reserved_b),
            })
    return {
        "pwr_up_msec": int(pwr_up_msec),
        "utc_msec": int(utc_msec),
        "listening_sec": float(listening_sec),
        "sos_mps": float(sos or 1500.0),
        "ping_number": int(ping_number),
        "ping_hz": float(ping_hz or 0.0),
        "pulse_sec": float(pulse_sec),
        "flags": int(flags),
        "num_points": len(points),
        "reserved": int(reserved),
        "points": points,
    }


def decode_end_ping_payload(payload: bytes) -> Optional[Dict[str, object]]:
    if len(payload) != END_PING.size:
        return None
    try:
        values = END_PING.unpack(payload)
    except struct.error:
        return None
    return {
        "start_m": float(values[1]),
        "end_m": max(float(values[2]), float(values[1]) + 0.1),
        "ping_number": int(values[6]),
        "ping_hz": float(values[13] or 0.0),
        "n_range_steps": int(values[16]),
        "samples_per_range_bin": int(values[17]),
        "timestamp": int(values[-1]),
    }


def decode_attitude_payload(payload: bytes) -> Optional[Dict[str, Union[float, int]]]:
    if len(payload) < ATTITUDE.size:
        return None
    try:
        up_x, up_y, up_z, reserved_1, reserved_2, reserved_3, utc_msec, pwr_up_msec = ATTITUDE.unpack_from(payload)
    except struct.error:
        return None
    return {
        "up_vec_x": float(up_x), "up_vec_y": float(up_y), "up_vec_z": float(up_z),
        "reserved_1": float(reserved_1), "reserved_2": float(reserved_2), "reserved_3": float(reserved_3),
        "utc_msec": int(utc_msec), "pwr_up_msec": int(pwr_up_msec),
    }
