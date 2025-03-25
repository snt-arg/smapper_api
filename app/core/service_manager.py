import threading
import time

from app.core.services import Service, RosService
from app.core.services.service import ServiceException, ServiceState
from app.exceptions import ServiceManagerException
from app.logger import logger
from app.schemas.services import RosServiceSchema, ServiceSchema

# TODO: Improve errors/exceptions


class ServiceManager:
    def __init__(self):
        self.services: dict[str, Service] = {}
        self._poll_thread = threading.Thread(
            target=self._poll_services_cb,
            name="service_manager_polling_thread",
            daemon=True,
        )

        self._poll_thread.start()

    def _poll_services_cb(self):
        while True:
            logger.debug("Polling services...")
            for _, service in self.services.items():
                service.poll()
            time.sleep(1)

    def add_service(self, service: ServiceSchema | RosServiceSchema) -> bool:
        if self.services.get(service.id):
            logger.error(f"A Service with id: {id} already exists")
            return False
        if isinstance(service, ServiceSchema):
            self.services[service.id] = Service(**service.model_dump())
        else:
            self.services[service.id] = RosService(**service.model_dump())

        return True

    def remove_service(self, id) -> bool:
        if self.services.get(id):
            logger.error(f"A Service with id: {id} does not exist")
            return False
        self.stop_service(id)
        self.services.pop(id)

        return True

    def remove_all_services(self) -> None:
        self.stop_all()
        self.services.clear()

    def start_all(self) -> None:
        for id, service in self.services.items():
            try:
                service.start()
            except ServiceException as e:
                continue

    def stop_all(self) -> None:
        for id, service in self.services.items():
            try:
                service.stop()
            except ServiceException as e:
                logger.error(f"Failed to start service {id} with error: {e}")

    def restart_all(self) -> None:
        for id, service in self.services.items():
            try:
                service.restart()
            except ServiceException as e:
                logger.error(f"Failed to start service {id} with error: {e}")

    def start_service(self, id: str) -> None:
        service = self.__get_service_by_id(id)
        service.start()

    def stop_service(self, id: str) -> None:
        service = self.__get_service_by_id(id)
        service.stop()

    def restart_service(self, id: str) -> None:
        service = self.__get_service_by_id(id)
        service.restart()

    def get_service_by_id(self, id: str) -> ServiceSchema:
        return self.__get_service_by_id(id).get_schema()

    def get_service_state(self, id: str) -> ServiceState:
        service = self.__get_service_by_id(id)
        return service.get_state()

    def get_service_states(self) -> dict[str, ServiceState]:
        states = {}
        for id in self.services:
            states[id] = self.get_service_state(id)
        return states

    def __get_service_by_id(self, id) -> Service:
        service = self.services.get(id)
        if service is None:
            logger.error(
                f"Service with id {id} does not yet exist. It must be created first."
            )
            raise ServiceManagerException(
                f"Service with id {id} does not yet exist. It must be created first."
            )
        return service
