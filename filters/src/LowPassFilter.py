class LowPassFilter:
    def __init__(self, alpha=.5, initval=0.0):
        self.alpha = alpha
        self.val = initval

    def update(self, newval):
        res = alpha*newval + (1 - alpha)*self.val
        self.val = res
        return res
