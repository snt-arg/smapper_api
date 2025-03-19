from app.core.processes import ProcessHandler


class ProcessPool:
    def __init__(self):
        self.processes = []

    def add_process(self, name: str, cmd: str | list[str]) -> None:
        process = ProcessHandler(name, cmd)
        self.processes.append(process)

    def start_all(self) -> None:
        pass

    def stop_all(self) -> None:
        pass

    def start_process(self, name: str) -> None:
        pass

    def stop_process(self, name: str) -> None:
        pass
