import time

class Timeout:
    def __init__(self, seconds, operation):
        self.seconds = seconds
        self.start_time = time.monotonic()
        self.operation = operation

    def check(self):
        if time.monotonic() - self.start_time >= self.seconds:
            raise TimeoutError(f"Timed out after {self.seconds} seconds whilst doing {self.operation}")

    def reset(self):
        self.start_time = time.monotonic()