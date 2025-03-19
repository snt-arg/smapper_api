import unittest
from app.core.processes import ProcessHandler, ProcessState
import time


class TestProcess(unittest.TestCase):
    def test_start(self):
        process = ProcessHandler("Sleep Service", "sleep 0.1")
        self.assertTrue(process.start())
        self.assertEqual(process.get_state(), ProcessState.ACTIVE)
        time.sleep(0.15)
        self.assertEqual(process.get_state(), ProcessState.INACTIVE)
        process.stop()

    def test_stop(self):
        process = ProcessHandler("Sleep Service", "sleep 0.2")
        self.assertTrue(process.start())
        self.assertTrue(process.stop())
        time.sleep(0.1)
        self.assertEqual(process.get_state(), ProcessState.INACTIVE)

    def test_bad_command(self):
        process = ProcessHandler("Sleep Service", "this command does not exist")
        self.assertFalse(process.start())
        self.assertEqual(process.get_state(), ProcessState.ERROR)
        process.stop()


if __name__ == "__main__":
    unittest.main()
