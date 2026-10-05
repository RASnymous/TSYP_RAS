"""Minimal rclpy stand-in (tests only). The test harness drives time and timers."""
from . import time, duration, qos, node  # noqa: F401

_ok = False


def init(args=None, signal_handler_options=None):
    global _ok
    _ok = True


def ok():
    return _ok


def shutdown():
    global _ok
    _ok = False


def spin(node):  # the harness calls the timers itself
    raise KeyboardInterrupt
