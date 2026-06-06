from collections import deque

class MovingAverage:
    def __init__(self, size):
        """
        Initialize the buffer with a fixed size.
        Once the buffer is full, the oldest values are dropped.
        """
        self.buffer = deque(maxlen=size)

    def add(self, value):
        """Add a new measurement to the buffer."""
        if value is not None:
            self.buffer.append(value)

    def get_avg(self):
        """
        Calculate the current average. 
        Returns 0 if the buffer is empty.
        """
        if not self.is_full():
            return None
        return sum(self.buffer) / len(self.buffer)

    def is_full(self):
        """Check if we have a full set of data yet."""
        return len(self.buffer) == self.buffer.maxlen

    def clear(self):
        """Reset the buffer."""
        self.buffer.clear()