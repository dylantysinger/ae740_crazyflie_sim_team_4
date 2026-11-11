import csv, time, math

class RunLogger:
    STATES = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi']

    def __init__(self, filename=None, mode='states', n_experts=None):
        self.filename = filename or f'{mode}_{int(time.time())}.csv'
        self.mode = mode
        self._t0 = None
        self._f = open(self.filename, 'w', newline='')
        self._w = csv.writer(self._f)
        if self.mode == 'states':
            self._n = len(self.STATES)
            self._w.writerow(['t'] + self.STATES + [f'{s}_ref' for s in self.STATES])
        elif self.mode == 'predErr':
            if n_experts is None:
                raise ValueError("predErr mode needs n_experts")
            self._n = n_experts
            self._w.writerow(['t']
                             + [f'err_{i+1}' for i in range(self._n)]   # prediction error per expert
                             + [f'p_{i+1}' for i in range(self._n)]     # selection probability per expert
                             + ['selected'])                            # expert actually in use
        else:
            raise ValueError(f"Unknown mode {self.mode}")

    def _rel_t(self, t):
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def log(self, t, state, ref):
        if self.mode != 'states':
            raise Exception(f"Incorrect mode {self.mode} for log()")
        s = [float(v) for v in list(state)[:self._n]]
        r = [float(v) for v in list(ref)[:self._n]]
        r += [math.nan] * (self._n - len(r))
        self._w.writerow([self._rel_t(t)] + s + r)
        self._f.flush()

    def log_predErr(self, t, errors, probs, selected_idx):
        if self.mode != 'predErr':
            raise Exception(f"Incorrect mode {self.mode} for log_predErr()")
        e = [float(v) for v in errors]
        p = [float(v) for v in probs]
        if len(e) != self._n or len(p) != self._n:
            raise ValueError(f"expected {self._n} experts, got {len(e)} errors / {len(p)} probs")
        self._w.writerow([self._rel_t(t)] + e + p + [int(selected_idx)])
        self._f.flush()

    def close(self):
        if not self._f.closed:
            self._f.close()
