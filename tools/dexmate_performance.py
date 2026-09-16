"""Wall-clock timings; nested sections are inclusive and must not be added together."""

from collections import defaultdict
from contextlib import contextmanager
from time import perf_counter


class Timings:
    def __init__(self):
        self.seconds = defaultdict(float)
        self.counts = defaultdict(int)

    @contextmanager
    def measure(self, name):
        start = perf_counter()
        try:
            yield
        finally:
            self.seconds[name] += perf_counter() - start
            self.counts[name] += 1

    def snapshot(self):
        return {name: {"seconds": seconds, "calls": self.counts[name],
                       "mean_ms": 1000 * seconds / self.counts[name]}
                for name, seconds in self.seconds.items()}
