from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse


class NotYetImplementedException(Exception):
    def __init__(self, detail: str = "This functionality is not yet implemented."):
        self.detail = detail
        super().__init__(detail)


class ServiceException(Exception):
    """Exception raised when a service fails."""

    def __init__(
        self,
        service_id: str,
        reason: str,
        cmd: str,
        stderr: str,
    ):
        self.reason = reason
        self.service_id = service_id
        self.cmd = cmd
        self.stderr = stderr
        super().__init__(f"[service:{self.service_id}] - {self.reason}")


class ServiceManagerException(Exception):
    def __init__(self, detail: str = "This functionality is not yet implemented."):
        self.detail = detail
        super().__init__(detail)


class BagNotFoundException(Exception):
    def __init__(self, bag_id):
        self.bag_id = bag_id
        self.detail = f"Bag with id {bag_id} not found."
        super().__init__(self.detail)


def init_exception_handlers(app: FastAPI):
    @app.exception_handler(NotYetImplementedException)
    async def not_yet_implemented_exception_handler(
        request: Request, exc: NotYetImplementedException
    ):
        return JSONResponse(
            status_code=500,
            content={"message": f"{exc}"},
        )

    @app.exception_handler(ServiceException)
    async def service_exception_handler(request: Request, exc: ServiceException):
        return JSONResponse(
            status_code=500,
            content={
                "service_id": exc.service_id,
                "reason": exc.reason,
                "cmd": exc.cmd,
                "stderr": exc.stderr,
            },
        )

    @app.exception_handler(ServiceManagerException)
    async def service_manager_exception_handler(
        request: Request, exc: ServiceManagerException
    ):
        return JSONResponse(
            status_code=500,
            content={"message": exc.detail},
        )
