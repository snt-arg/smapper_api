from typing import Optional
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse


class NotYetImplementedException(Exception):
    """Raised when a feature is not yet implemented."""

    def __init__(self, detail: str = "This functionality is not yet implemented."):
        self.detail = detail
        super().__init__(detail)


class ServiceException(Exception):
    """Exception raised when a service fails to start, run, or terminate correctly.

    Attributes:
        service_id: Identifier of the service that failed.
        reason: Description of the failure reason.
        cmd: The command that was attempted.
        stderr: Standard error output from the failed process.
    """

    def __init__(
        self,
        service_id: str,
        reason: str,
        stderr: Optional[str] = None,
    ):
        self.reason = reason
        self.service_id = service_id
        self.stderr = stderr
        super().__init__(f"[service:{self.service_id}] - {self.reason}")


class ServiceManagerException(Exception):
    """Raised for service manager-level errors such as missing service instances."""

    def __init__(self, detail: str):
        self.detail = detail
        super().__init__(detail)


class BagNotFoundException(Exception):
    """Raised when a bag file or recording with the given ID is not found."""

    def __init__(self, bag_id):
        self.bag_id = bag_id
        self.detail = f"Bag with id {bag_id} not found."
        super().__init__(self.detail)


class BagRecordingOnGoingException(Exception):
    """Raised when a new recording is requested while one is already in progress."""

    def __init__(
        self,
        detail="There exists already an ongoing recording session. You must stop it first.",
    ):
        self.detail = detail
        super().__init__(self.detail)


def init_exception_handlers(app: FastAPI):
    """Register exception handlers for custom exceptions in a FastAPI app.

    Args:
        app: The FastAPI application instance where handlers will be registered.
    """

    @app.exception_handler(NotYetImplementedException)
    async def not_yet_implemented_exception_handler(
        request: Request, exc: NotYetImplementedException
    ):
        """Handle NotYetImplementedException and return a 500 response."""
        return JSONResponse(
            status_code=500,
            content={"message": f"{exc}"},
        )

    @app.exception_handler(ServiceException)
    async def service_exception_handler(request: Request, exc: ServiceException):
        """Handle ServiceException and return structured error details."""
        return JSONResponse(
            status_code=500,
            content={
                "service_id": exc.service_id,
                "reason": exc.reason,
                "stderr": exc.stderr,
            },
        )

    @app.exception_handler(ServiceManagerException)
    async def service_manager_exception_handler(
        request: Request, exc: ServiceManagerException
    ):
        """Handle ServiceManagerException and return a 500 response with the error detail."""
        return JSONResponse(
            status_code=500,
            content={"message": exc.detail},
        )
