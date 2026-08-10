import threading
import time

from linkerbot_hand_ros.node import _PriorityIoLock


def test_poll_uses_lock_when_no_priority_operation_is_pending():
    lock = _PriorityIoLock()

    with lock.poll() as acquired:
        assert acquired

    with lock.poll() as acquired:
        assert acquired


def test_poll_yields_while_priority_operation_owns_lock():
    lock = _PriorityIoLock()

    with lock.priority():
        with lock.poll() as acquired:
            assert not acquired


def test_waiting_priority_operation_prevents_poll_from_reacquiring_lock():
    lock = _PriorityIoLock()
    priority_entered = threading.Event()
    allow_priority_exit = threading.Event()

    def run_priority_operation():
        with lock.priority():
            priority_entered.set()
            assert allow_priority_exit.wait(timeout=1.0)

    with lock.poll() as acquired:
        assert acquired
        worker = threading.Thread(target=run_priority_operation)
        worker.start()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with lock._state_lock:
                if lock._priority_waiters == 1:
                    break
            time.sleep(0.001)
        else:
            raise AssertionError("priority operation did not start waiting")

    assert priority_entered.wait(timeout=1.0)
    with lock.poll() as acquired:
        assert not acquired

    allow_priority_exit.set()
    worker.join(timeout=1.0)
    assert not worker.is_alive()

    with lock.poll() as acquired:
        assert acquired
