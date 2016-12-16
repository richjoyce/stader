__all__ = ['ControlSurface', 'ControlSurfaceSecondOrder']


class ControlSurface(object):
    def __init__(self):
        self.commanded = 0.0
        self.angle = 0.0
        self.rate = 0.0
        self.acceleration = 0.0

    def update(self, dt, commanded):
        rate = (self.commanded-commanded)/dt
        self.acceleration = (self.rate-rate)/dt

        self.angle = commanded
        self.rate = rate
        self.commanded = commanded


class ControlSurfaceSecondOrder(ControlSurface):
    def __init__(self, natural_frequency, damping, rate_limit=None, displacement_limit=None):
        self.natural_frequency = natural_frequency
        self.damping = damping
        self.rate_limit = rate_limit
        self.displacement_limit = displacement_limit

        self.commanded = 0.0
        self.angle = 0.0
        self.rate = 0.0
        self.acceleration = 0.0

    def update(self, dt, commanded):
        drate = self.rate
        self.commanded = commanded

        daccel = ((-2*self.natural_frequency*self.damping)*self.rate
                  - (self.natural_frequency**2)*self.angle
                  + (self.natural_frequency**2)*self.commanded)

        self.acceleration = daccel
        self.rate += daccel*dt
        if self.rate_limit is not None:
            self.rate = max(min(self.rate, self.rate_limit), -self.rate_limit)

        self.angle += self.rate*dt
        if self.displacement_limit is not None:
            self.angle = max(min(self.angle, self.displacement_limit), -self.displacement_limit)
