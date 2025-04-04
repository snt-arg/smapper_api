import time
import unittest
from unittest.mock import patch
from app.core.services import ServiceState
from app.core.service_manager import ServiceManager
from app.core.services.service import Service
from app.exceptions import ServiceException, ServiceManagerException
from app.schemas.services import ServiceConfigSchema


class TestServiceManager(unittest.TestCase):

    def setUp(self):
        """Setup a fresh instance of ServiceManager for each test"""
        self.manager = ServiceManager()
        self.service_schema_1 = ServiceConfigSchema(
            name="Test 1", id="test_service_1", srv_type="SERVICE", cmd="sleep 0.1"
        )

        self.service_schema_2 = ServiceConfigSchema(
            name="Test 2", id="test_service_2", srv_type="SERVICE", cmd="sleep 0.3"
        )

    def test_add_service(self):
        """Test adding a service successfully"""
        result = self.manager.add_service(self.service_schema_1)
        self.assertTrue(result)
        self.assertEqual(len(self.manager.services), 1)
        self.manager.remove_all_services()

    def test_add_service_duplicate_id(self):
        """Test adding a service with a duplicate ID"""
        self.manager.add_service(self.service_schema_1)
        result = self.manager.add_service(self.service_schema_1)
        self.assertFalse(result)
        self.assertEqual(len(self.manager.services), 1)
        self.manager.remove_all_services()

    def test_start_service(self):
        """Test starting a single service"""
        self.manager.add_service(self.service_schema_1)
        self.manager.start_service(self.service_schema_1.id)
        service = self.manager.services[self.service_schema_1.id]
        self.assertEqual(service.get_state(), ServiceState.ACTIVE)
        self.manager.remove_all_services()

    def test_stop_service(self):
        """Test stopping a service"""
        self.manager.add_service(self.service_schema_1)
        self.manager.start_service(self.service_schema_1.id)
        self.manager.stop_service(self.service_schema_1.id)
        service = self.manager.services[self.service_schema_1.id]
        self.assertEqual(service.get_state(), ServiceState.INACTIVE)
        self.manager.remove_all_services()

    def test_restart_service(self):
        """Test restarting a service"""
        self.manager.add_service(self.service_schema_1)
        self.manager.start_service(self.service_schema_1.id)
        self.manager.restart_service(self.service_schema_1.id)
        service = self.manager.services[self.service_schema_1.id]
        self.assertEqual(service.get_state(), ServiceState.ACTIVE)
        self.manager.remove_all_services()

    def test_get_service_state(self):
        """Test getting the state of a specific service"""
        self.manager.add_service(self.service_schema_1)
        self.manager.start_all()
        self.assertEqual(
            self.manager.get_service_state(self.service_schema_1.id),
            ServiceState.ACTIVE,
        )
        self.manager.stop_all()
        self.assertEqual(
            self.manager.get_service_state(self.service_schema_1.id),
            ServiceState.INACTIVE,
        )
        self.manager.remove_all_services()

    def test_get_service_states(self):
        """Test getting the states of all services"""
        self.manager.add_service(self.service_schema_1)
        self.manager.add_service(self.service_schema_2)
        self.manager.start_all()
        states = self.manager.get_service_states()
        self.assertEqual(states[self.service_schema_1.id], ServiceState.ACTIVE)
        self.assertEqual(states[self.service_schema_2.id], ServiceState.ACTIVE)
        self.manager.remove_all_services()

    def test_service_not_found(self):
        """Test handling of a non-existent service"""
        with self.assertRaises(ServiceManagerException):
            self.manager.get_service_state("non_existent_service")

    def test_start_all_services(self):
        """Test starting all services"""
        self.manager.add_service(self.service_schema_1)
        self.manager.add_service(self.service_schema_2)
        self.manager.start_all()
        for service in self.manager.services.values():
            self.assertEqual(service.get_state(), ServiceState.ACTIVE)
        self.manager.remove_all_services()

    def test_stop_all_services(self):
        """Test stopping all services"""
        self.manager.add_service(self.service_schema_1)
        self.manager.add_service(self.service_schema_2)
        self.manager.start_all()
        self.manager.stop_all()
        for service in self.manager.services.values():
            self.assertEqual(service.get_state(), ServiceState.INACTIVE)
        self.manager.remove_all_services()

    def test_restart_all_services(self):
        """Test restarting all services"""
        self.manager.add_service(self.service_schema_1)
        self.manager.add_service(self.service_schema_2)
        self.manager.start_all()
        self.manager.restart_all()
        for service in self.manager.services.values():
            self.assertEqual(service.get_state(), ServiceState.ACTIVE)
        self.manager.remove_all_services()

    @patch(
        "app.core.services.Service.start",
        side_effect=ServiceException("dummy_id", "Failed to start", "no cmd", ""),
    )
    def test_start_service_failure(self, mock_start):
        """Test handling failure when starting a service"""
        self.manager.add_service(self.service_schema_1)
        with self.assertRaises(ServiceException):
            self.manager.start_service(self.service_schema_1.id)
        self.manager.remove_all_services()

    @patch(
        "app.core.services.Service.stop",
        side_effect=ServiceException("dummy_id", "Failed to stop", "no cmd", ""),
    )
    def test_stop_service_failure(self, mock_stop):
        """Test handling failure when stopping a service"""
        self.manager.add_service(self.service_schema_1)
        self.manager.start_service(self.service_schema_1.id)
        with self.assertRaises(ServiceException):
            self.manager.stop_service(self.service_schema_1.id)
        self.manager.remove_all_services()


if __name__ == "__main__":
    unittest.main()
