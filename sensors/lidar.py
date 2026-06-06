import asyncio
import threading
import atexit
import signal
import statistics
import logging
from threading import Lock
from sensors.rplidarc1 import RPLidar

class LidarManager:
    def __init__(self, port="/dev/ttyUSB0", baud=460800):
        self.lidar = RPLidar(port, baud)

        self.latest_scan = {}
        self.scan_lock = Lock()
        self.data_ready = threading.Event()

        self.thread = None
        self.loop = None
        self.logger = logging.getLogger(f"{__name__}")
        
        # ✅ auto cleanup on normal exit
        atexit.register(self._safe_stop)

        # ✅ cleanup on Ctrl+C / kill
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self._stopped = False
        
    def _safe_stop(self):
        try:
            self.stop()
        except Exception:
            pass

    def _signal_handler(self, sig, frame):
        self._safe_stop()
        raise SystemExit

    def start(self, timeout = 10):
        self.thread = threading.Thread(target=self._thread_fn, daemon=True)
        self.thread.start()
        
        # ✅ wait until all 360 data has come in
        if not self.data_ready.wait(timeout):
            raise RuntimeError("Lidar failed to produce data in time")

    def stop(self):
        if (not self._stopped):
            self._stopped = True
            self.lidar.stop_event.set()
            if self.thread:
                self.thread.join(timeout=2)
            self.lidar.reset()


    def _get_complete_scan(self):
        with self.scan_lock:
            return self.latest_scan.copy()
        
    def _get_subset(self, subset_angle):
        """
        start_angle = counter_clockwise most angle
        end_angle = clockwise most angle
        """
        start_angle = subset_angle[0]
        end_angle = subset_angle[1]
        scan_360 = self._get_complete_scan()
        returned_scan = {}
        if (start_angle > end_angle):
            end_angle = end_angle + 360
        for angle in range(start_angle, end_angle+1, 1):
            adjusted_angle = angle % 360 # Handle the wrap around case
            distance = scan_360.get(adjusted_angle)
            if (distance is not None):
                returned_scan[angle] = (distance)
        return returned_scan
    
    def get_median_from_subset(self, subset_angle):
        return statistics.median(self._get_subset(subset_angle).values())
    
    def get_angle(self, angle):
        the_dist = -1 # This is terrible please fix
        with self.scan_lock:
            if angle in self.latest_scan:
                the_dist = self.latest_scan[angle]
        return the_dist

    def _thread_fn(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._async_runner())
        finally:
            self.loop.close()

    async def _async_runner(self):
        async def forward_data():
            while not self.lidar.stop_event.is_set():
                try:
                    data = await asyncio.wait_for(
                        self.lidar.output_queue.get(),
                        timeout=0.1
                    )

                    angle = int(data['a_deg'])
                    adjusted_angle = (angle - 90)
                    if (adjusted_angle < 0):
                        adjusted_angle = adjusted_angle + 360
                    dist = data['d_mm']
                    if (dist is not None):
                        with self.scan_lock:
                            self.latest_scan[adjusted_angle] = dist
                    
                    if (not self.data_ready.is_set() and self.latest_scan.get(0) is not None and self.latest_scan.get(180) is not None):
                        self.logger.info("LIDAR READY!")
                        self.data_ready.set() # Signal that we have received data for downstream to function
                except asyncio.TimeoutError:
                    continue
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    break

        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.lidar.simple_scan(make_return_dict=False))
            tg.create_task(forward_data())