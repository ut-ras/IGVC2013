class LowPassFilter:
    def __init__(self, alpha=.5, initval=0.0):
        self.alpha = alpha
        self.val = initval

    def update(self, newval):
        res = self.alpha*newval + (1 - self.alpha)*self.val
        self.val = res
        return res
