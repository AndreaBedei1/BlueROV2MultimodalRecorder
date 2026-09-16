"""Persistent process, thread, and Tk exception diagnostics."""

from __future__ import annotations

import faulthandler
import logging
import logging.handlers
import sys
import threading
from pathlib import Path
from typing import Optional, TextIO


_fault_stream: Optional[TextIO] = None


def configure_diagnostics(root: Path) -> logging.Logger:
    """Install rotating logs and thread/fatal exception hooks once."""
    global _fault_stream
    log_dir = Path(root) / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("bluerov_recorder")
    logger.setLevel(logging.INFO)
    if not logger.handlers:
        handler = logging.handlers.RotatingFileHandler(
            str(log_dir / "recorder.log"), maxBytes=5 * 1024 * 1024,
            backupCount=3, encoding="utf-8",
        )
        handler.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)s %(threadName)s %(name)s: %(message)s"
        ))
        logger.addHandler(handler)
    if _fault_stream is None:
        _fault_stream = (log_dir / "faulthandler.log").open("a", encoding="utf-8")
        faulthandler.enable(file=_fault_stream, all_threads=True)

    def thread_exception(args):
        logger.critical(
            "Unhandled worker exception in %s", args.thread.name,
            exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
        )

    threading.excepthook = thread_exception

    def process_exception(exc_type, exc_value, exc_traceback):
        logger.critical(
            "Unhandled process exception",
            exc_info=(exc_type, exc_value, exc_traceback),
        )
        sys.__excepthook__(exc_type, exc_value, exc_traceback)

    sys.excepthook = process_exception
    logger.info("Persistent diagnostics configured")
    return logger
