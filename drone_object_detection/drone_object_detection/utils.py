from time import monotonic
import logging

logging.basicConfig(
    format='%(asctime)s %(levelname)s:%(message)s',
    level=logging.INFO,
    datefmt='%m/%d/%Y %I:%M:%S %p',
)

class Timer():
    def __init__(self):
        self.tick = None

    def start(self):
        self.tick = monotonic()

    def end(self):
        if self.tick is None:
            raise Exception(f"clock is not started.")

        return monotonic() - self.tick