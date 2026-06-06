import threading

class Supervisor():
    def __init__(self):
        self.isstopped = False
        self.checks = []
    
    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()
    
    def _loop(self):
        while not self.isstopped:
            for check in self.checks:
                try:
                    if check():
                        print(f"{check.__name__} triggered!")
                except Exception as e:
                    print(f"Error in {check}: {e}")

            time.sleep(3)
        
    def add_check(self, func):
        self.checks.append(func)