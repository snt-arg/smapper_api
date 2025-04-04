import threading
import time
from typing import List

from app.core.services import Service, RosService
from app.core.services.service import ServiceException, ServiceState
from app.exceptions import ServiceManagerException
from app.logger import logger
from app.schemas.services import (
    RosServiceConfigSchema,
    ServiceConfigSchema,
    ServiceSchema,
)

# TODO: Improve errors/exceptions


class ServiceManager:
    """Manages a collection of services, handling lifecycle operations and polling in a background thread."""

    def __init__(self):
        """Initialize the service manager and start a background polling thread."""

        self.services: dict[str, Service] = {}
        self._poll_thread = threading.Thread(
            target=self._poll_services_cb,
            name="service_manager_polling_thread",
            daemon=True,
        )
        self.lock = threading.Lock()
        self._poll_thread.start()

    def _poll_services_cb(self):
        """Background callback that continuously polls all services to update their states."""

        while True:
            logger.debug("Polling services...")
            for _, service in self.services.items():
                self.lock.acquire()
                service.poll()
                self.lock.release()
            time.sleep(1)

    def add_service(
        self, service: ServiceConfigSchema | RosServiceConfigSchema
    ) -> bool:
        """Add a new service to the manager.

        Args:
            service: The service schema to add.

        Returns:
            True if added successfully, False if a service with the same ID already exists.
        """
        if self.services.get(service.id):
            logger.error(f"A Service with id: {id} already exists")
            return False
        if isinstance(service, ServiceConfigSchema):
            self.services[service.id] = Service(**service.model_dump())
        else:
            self.services[service.id] = RosService(**service.model_dump())

        return True

    def remove_service(self, id) -> bool:
        """Remove a service from the manager.

        Args:
            id: ID of the service to remove.

        Returns:
            True if the service was removed, False if it did not exist.
        """
        if self.services.get(id):
            logger.error(f"A Service with id: {id} does not exist")
            return False
        self.stop_service(id)
        self.services.pop(id)

        return True

    def remove_all_services(self) -> None:
        """Stop and remove all services from the manager."""

        self.stop_all()
        self.services.clear()

    def start_all(self) -> None:
        """Start all registered services.

        Ignores and continues on failure.
        """
        for _, service in self.services.items():
            try:
                service.start()
            except ServiceException as e:
                continue

    def stop_all(self) -> None:
        """Stop all registered services.

        Logs errors but continues stopping others.
        """
        for id, service in self.services.items():
            try:
                service.stop()
            except ServiceException as e:
                logger.error(f"Failed to start service {id} with error: {e}")

    def restart_all(self) -> None:
        """Restart all registered services.

        Logs errors but continues restarting others.
        """
        for id, service in self.services.items():
            try:
                service.restart()
            except ServiceException as e:
                logger.error(f"Failed to start service {id} with error: {e}")

    def start_service(self, id: str) -> None:
        """Start a specific service by its ID.

        Args:
            id: The ID of the service to start.

        Raises:
            ServiceManagerException: If the service does not exist.
        """
        service = self.__get_service_by_id(id)
        service.start()

    def stop_service(self, id: str) -> None:
        """Stop a specific service by its ID.

        Args:
            id: The ID of the service to stop.

        Raises:
            ServiceManagerException: If the service does not exist.
        """
        service = self.__get_service_by_id(id)
        service.stop()

    def restart_service(self, id: str) -> None:
        """Restart a specific service by its ID.

        Args:
            id: The ID of the service to restart.

        Raises:
            ServiceManagerException: If the service does not exist.
        """
        service = self.__get_service_by_id(id)
        service.restart()

    def get_services(self) -> List[ServiceSchema]:
        return [service.get_schema() for _, service in self.services.items()]

    def get_service_by_id(self, id: str) -> ServiceSchema:
        """Retrieve the service schema for a specific service.

        Args:
            id: The ID of the service to fetch.

        Returns:
            The ServiceSchema for the requested service.

        Raises:
            ServiceManagerException: If the service does not exist.
        """
        return self.__get_service_by_id(id).get_schema()

    def __get_service_by_id(self, id) -> Service:
        """Internal helper to retrieve a service object by ID.

        Args:
            id: The ID of the service.

        Returns:
            The corresponding Service instance.

        Raises:
            ServiceManagerException: If the service does not exist.
        """
        service = self.services.get(id)
        if service is None:
            logger.error(
                f"Service with id {id} does not yet exist. It must be created first."
            )
            raise ServiceManagerException(
                f"Service with id {id} does not yet exist. It must be created first."
            )
        return service
