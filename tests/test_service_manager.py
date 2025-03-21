import time
import unittest
from unittest.mock import patch
from app.core.services import Service, ServiceState, ServiceException
from app.core.service_manager import ServiceManager, ServiceManagerException


class TestServiceManager(unittest.TestCase):

    def setUp(self):
        """Setup a fresh instance of ServiceManager for each test"""
        self.manager = ServiceManager()

    def test_add_service(self):
        """Test adding a service successfully"""
        result = self.manager.add_service("1", "test_service_1", "sleep 0.1")
        self.assertTrue(result)
        self.assertEqual(len(self.manager.services), 1)
        self.manager.remove_all_services()

    def test_add_service_duplicate_id(self):
        """Test adding a service with a duplicate ID"""
        self.manager.add_service("1", "test_service_1", "sleep 0.1")
        result = self.manager.add_service("1", "test_service_2", "sleep 0.2")
        self.assertFalse(result)
        self.assertEqual(len(self.manager.services), 1)
        self.manager.remove_all_services()

    def test_start_service(self):
        """Test starting a single service"""
        self.manager.add_service("1", "test_service_1", "sleep 0.1")
        self.manager.start_service("1")
        service = self.manager.services["1"]
        self.assertEqual(service.get_state(), ServiceState.ACTIVE)
        self.manager.remove_all_services()

    def test_stop_service(self):
        """Test stopping a service"""
        self.manager.add_service("1", "test_service_1", "sleep 0.1")
        self.manager.start_service("1")
        self.manager.stop_service("1")
        service = self.manager.services["1"]
        self.assertEqual(service.get_state(), ServiceState.INACTIVE)
        self.manager.remove_all_services()

    def test_restart_service(self):
        """Test restarting a service"""
        self.manager.add_service("1", "test_service_1", "sleep 0.1")
        self.manager.start_service("1")
        self.manager.restart_service("1")
        service = self.manager.services["1"]
        self.assertEqual(service.get_state(), ServiceState.ACTIVE)
        self.manager.remove_all_services()

    def test_get_service_state(self):
        """Test getting the state of a specific service"""
        self.manager.add_service("1", "test_service_1", "sleep 0.1")
        self.manager.start_service("1")
        self.assertEqual(self.manager.get_service_state("1"), ServiceState.ACTIVE)
        self.manager.stop_service("1")
        self.assertEqual(self.manager.get_service_state("1"), ServiceState.INACTIVE)
        self.manager.remove_all_services()

    def test_get_service_states(self):
        """Test getting the states of all services"""
        self.manager.add_service("1", "test_service_1", "sleep 0.2")
        self.manager.add_service("2", "test_service_2", "sleep 0.3")
        self.manager.start_service("1")
        self.manager.start_service("2")
        states = self.manager.get_service_states()
        self.assertEqual(states["1"], ServiceState.ACTIVE)
        self.assertEqual(states["2"], ServiceState.ACTIVE)
        self.manager.remove_all_services()

    def test_service_not_found(self):
        """Test handling of a non-existent service"""
        with self.assertRaises(ServiceManagerException):
            self.manager.get_service_state("non_existent_service")

    def test_start_all_services(self):
        """Test starting all services"""
        self.manager.add_service("1", "test_service_1", "sleep 1")
        self.manager.add_service("2", "test_service_2", "sleep 2")
        self.manager.start_all()
        for service in self.manager.services.values():
            self.assertEqual(service.get_state(), ServiceState.ACTIVE)
        self.manager.remove_all_services()

    def test_stop_all_services(self):
        """Test stopping all services"""
        self.manager.add_service("1", "test_service_1", "sleep 0.1")
        self.manager.add_service("2", "test_service_2", "sleep 0.2")
        self.manager.start_all()
        self.manager.stop_all()
        for service in self.manager.services.values():
            self.assertEqual(service.get_state(), ServiceState.INACTIVE)
        self.manager.remove_all_services()

    def test_restart_all_services(self):
        """Test restarting all services"""
        self.manager.add_service("1", "test_service_1", "sleep 1")
        self.manager.add_service("2", "test_service_2", "sleep 2")
        self.manager.start_all()
        self.manager.restart_all()
        for service in self.manager.services.values():
            self.assertEqual(service.get_state(), ServiceState.ACTIVE)
        self.manager.remove_all_services()

    @patch(
        "app.core.services.Service.start",
        side_effect=ServiceException("Failed to start"),
    )
    def test_start_service_failure(self, mock_start):
        """Test handling failure when starting a service"""
        self.manager.add_service("1", "test_service_1", "sleep 0.1")
        with self.assertRaises(ServiceException):
            self.manager.start_service("1")
        self.manager.remove_all_services()

    @patch(
        "app.core.services.Service.stop", side_effect=ServiceException("Failed to stop")
    )
    def test_stop_service_failure(self, mock_stop):
        """Test handling failure when stopping a service"""
        self.manager.add_service("1", "test_service_1", "sleep 0.1")
        self.manager.start_service("1")
        with self.assertRaises(ServiceException):
            self.manager.stop_service("1")
        self.manager.remove_all_services()


if __name__ == "__main__":
    unittest.main()
