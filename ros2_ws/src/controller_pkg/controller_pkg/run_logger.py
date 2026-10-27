import csv, time, math

class RunLogger:
    STATES = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi']

    def __init__(self, filename=None):
        self.filename = filename or f'run_{int(time.time())}.csv'
        self._f = open(self.filename, 'w', newline='')
        self._w = csv.writer(self._f)
        self._n = len(self.STATES)
        self._w.writerow(['t'] + self.STATES + [f'{s}_ref' for s in self.STATES])
        self._t0 = None
        
    def log(self, t, state, ref):
        if self._t0 is None:
            self._t0 = t
        s = [float(v) for v in list(state)[:self._n]]
        r = [float(v) for v in list(ref)[:self._n]]
        r+= [math.nan] * (self._n - len(r))
        self._w.writerow([t - self._t0] + s + r)
        self._f.flush()

    def close(self):
        if not self._f.closed:
            self._f.close()
