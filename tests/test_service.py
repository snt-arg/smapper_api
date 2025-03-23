import unittest

from app.core.services import Service, ServiceState
from app.exceptions import ServiceException


class TestService(unittest.TestCase):
    def test_start(self):
        process = Service("Start Service Test", "test_service", "sleep 0.1")
        process.start()
        self.assertEqual(process.get_state(), ServiceState.ACTIVE)
        process.stop()

    def test_stop(self):
        process = Service("Stop Service Test", "test_service", "sleep 0.1")
        process.start()
        process.stop()
        self.assertEqual(process.get_state(), ServiceState.INACTIVE)

    def test_bad_command(self):
        process = Service(
            "Bad Service Test", "test_service", "this command does not exist"
        )
        self.assertRaises(ServiceException, process.start)
        self.assertEqual(process.get_state(), ServiceState.ERROR)


if __name__ == "__main__":
    unittest.main()
