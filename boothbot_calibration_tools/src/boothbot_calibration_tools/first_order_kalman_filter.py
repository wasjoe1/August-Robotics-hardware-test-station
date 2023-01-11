
class FirstOrderKalmanFilter(object):
    def __init__(self, Q, R, x0=0, p0=0, A=1, H=1):
        self.param = {
            "Q": Q,
            "R": R,
            "A": A,
            "H": H,
            "X": x0,
            "P": p0,
        }
        self.K = None

    @staticmethod
    def kf(z, X, P, Q, R, A=1, H=1, BU=0, W=0):
        X10 = A * X + BU + W
        P10 = A * P * A + Q

        K = P10 * H / (H * P10 * H + R)
        X1 = X10 + K * (z - H * X10)
        P1 = (1 - K * H) * P10
        return X1, P1, K

    def update_observed(self, z):
        x, p, k = self.kf(z, **self.param)
        self.param.update({
            "X": x,
            "P": p,
        })
        self.K = k
        return x
