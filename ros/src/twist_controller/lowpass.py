from __future__ import division


class LowPassFilter(object):
    """First-order low pass filter"""

    def __init__(self, tau, ts=1.):
        """Initializes the low pass filter.

        Args:
            tau: time constant
            ts: sampling period
        """
        self.a = 1. / (tau / ts + 1.)
        #self.b = tau / ts / (tau / ts + 1.);
        self.b = 1. - self.a

        self.last_val = 0.
        self.ready = False

    def get(self):
        """Retrieves last value."""
        return self.last_val

    def filt(self, val):
        """Update the filter with a new value."""
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
