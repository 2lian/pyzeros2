import time
from abc import ABC, abstractmethod

import numpy as np

class MyNode(ABC):
    def __init__(self):
        # super().__init__("bench")
        # self.hello_pub = self.create_publisher(**TOPIC_HELLO.as_kwarg())
        # self.world_pub = self.create_publisher(**TOPIC_WORLD.as_kwarg())
        # self.hello_sub = self.create_subscription(**TOPIC_HELLO.as_kwarg())
        # self.world_pub = self.create_subscription(**TOPIC_WORLD.as_kwarg())
        self.stamps = []
        self.count = 0

    @abstractmethod
    def world_send(self): ...

    @abstractmethod
    def hello_send(self): ...

    @abstractmethod
    def finish(self): ...

    @abstractmethod
    def run(self, *args, **kwargs): ...

    def results(self, warmup: int = 1000, nominal_hz: float | None = None) -> None:

        stamps = np.asarray(self.stamps[warmup:-warmup], dtype=np.int64)
        if stamps.size < warmup*2:
            print(f"Not enough stamps after warmup={warmup}: n={stamps.size}")
            return

        # Total duration (s)
        total_s = (stamps[-1] - stamps[0]) / 1e9

        # Inter-arrival / period sequence (s)
        periods_s = np.diff(stamps) / 1e9  # length = n-1

        mean_T = periods_s.mean()
        std_T = periods_s.std(ddof=1) if periods_s.size > 1 else 0.0
        var_T = periods_s.var(ddof=1) if periods_s.size > 1 else 0.0

        # Choose reference period for jitter
        ref_T = (1.0 / nominal_hz) if (nominal_hz is not None) else mean_T
        jitter_s = periods_s - ref_T
        jitter_abs_s = np.abs(jitter_s)

        # Frequency estimates
        hz_from_total_events = stamps.size / total_s  # matches your original intent
        hz_from_intervals = (stamps.size - 1) / total_s  # interval-consistent

        # Percentiles for periods and absolute jitter
        p = np.percentile(periods_s, [0, 1, 5, 50, 95, 99, 100])
        ja = np.percentile(jitter_abs_s, [50, 95, 99, 100])

        print(
            f"n_stamps: {stamps.size}  n_intervals: {periods_s.size}  warmup: {warmup}"
        )
        print(f"Time (s): {total_s*1e9:_}")
        print(
            f"Hz (events/time): {hz_from_total_events:_.2f}   Hz (intervals/time): {hz_from_intervals:_.2f}"
        )
        print(f"Period mean (s): {mean_T*1e9:_.1f}")
        print(f"Period std  (s): {std_T*1e9:_.1f}")
        print(f"Period var  (s^2): {var_T:.9e}")
        print(f"Period min/med/max (s): {p[0]*1e9:_.1f} / {p[3]*1e9:_.1f} / {p[6]*1e9:_.1f}")
        print(f"Period p95/p99 (s): {p[4]*1e9:_.1f} / {p[5]*1e9:_.1f}")

        print(f"Jitter ref period (s): {ref_T*1e9:_.1f}  (nominal_hz={nominal_hz})")
        print(
                f"Jitter std (s): {jitter_s.std(ddof=1) if jitter_s.size > 1 else 0.0*1e9:_.1f}"
        )
        print(
                f"Jitter p50/p95/p99/max |e| (s): {ja[0]*1e9:_} / {ja[1]*1e9:_} / {ja[2]*1e9:_} / {ja[3]*1e9:_}"
        )
        print(f"Jitter peak-to-peak (s): {(periods_s.max() - periods_s.min())*1e9:_}")
        print(
            f"Coeff. of variation (std/mean): {(std_T / mean_T) if mean_T > 0 else np.nan:.6e}"
        )

    def world_cbk(self, msg):
        # print("w")
        the_end = self.cbk()
        if not the_end:
            self.hello_send()

    def hello_cbk(self, msg):
        # print("h")
        the_end = self.cbk()
        if not the_end:
            self.world_send()

    def cbk(self):
        # print(self.count)
        self.count += 1
        self.stamps.append(time.perf_counter_ns())
        # if self.count > 100_000:
        #     print("\nDONE\n")
        #     self.results()
        #     self.finish()
        #     return True
        # return False

