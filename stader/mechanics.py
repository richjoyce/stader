import numpy as np
import scipy.signal

__all__ = ["Aircraft", "AircraftLateral", "AircraftLongitudinal"]

class Aircraft(object):
    def __init__(self, derivatives):
        self.lateral = AircraftLateral(derivatives)
        self.longitudinal = AircraftLongitudinal(derivatives)


    def update(self, dt, inputs={}):
        ulat = np.zeros(self.lateral._n_inputs)
        ulong = np.zeros(self.longitudinal._n_inputs)
        if 'elevator' in inputs:
            ulong[0] = inputs['elevator']
        if 'thrust' in inputs:
            ulong[1] = inputs['thrust']
        if 'aileron' in inputs:
            ulat[0] = inputs['aileron']
        if 'rudder' in inputs:
            ulat[1] = inputs['rudder']

        self.lateral.update(dt, ulat)
        self.longitudinal.update(dt, ulong)


    def __getattr__(self, attr):
        if hasattr(self.lateral, attr):
            return self.lateral.__getattribute__(attr)
        if hasattr(self.longitudinal, attr):
            return self.longitudinal.__getattribute__(attr)
        return self.__getattribute__(attr)


class AircraftDynamics(object):
    """
    Base aircraft dynamics class for lateral or longitudinal
    """

    def __init__(self, A, B, x0=None):
        self._A = A
        self._B = B
        self._n_states = A.shape[0]
        self._n_inputs = B.shape[1]
        if x0 is None:
            x0 = np.zeros(self._n_states)
        self._x = x0
        self._xdot = np.zeros(self._n_states)


    def update(self, dt, u=None):
        if u is None:
            u = np.zeros(self._n_inputs)
        self._xdot = dt*(self._A.dot(self._x) + self._B.dot(u))
        self._x += self._xdot


    def lti(self, C=None, D=None):
        if C is None:
            C = np.identity(self._n_states)
        if D is None:
            D = np.zeros((self._n_states, self._n_inputs))
        return scipy.signal.lti(self._A, self._B, C, D)


class AircraftLateral(AircraftDynamics):
    def __init__(self, derivatives):
        d = derivatives
        s = d['stability']
        # x = [ dv dp dr dphi dpsi dy ]
        A = np.array((
            (s['Y']['v'], s['Y']['p'], s['Y']['r']-d['U0'], d['g']*np.cos(np.deg2rad(d['theta0'])), 0, 0),
            (s['Lprime']['v'], s['Lprime']['p'], s['Lprime']['r'], d['g']*np.sin(np.deg2rad(d['theta0'])), 0, 0),
            (s['Nprime']['v'], s['Nprime']['p'], s['Nprime']['r'], 0, 0, 0),
            (0, 1, np.tan(d['theta0']), 0, 0, 0),
            (0, 0, 1/np.cos(d['theta0']), 0, 0, 0),
            (1, 0, 0, 0, d['U0'], 0)))

        # u = [ aileron rudder ]
        B = np.array(((s['Y']['delta_a'], s['Y']['delta_r']),
                      (s['Lprime']['delta_a'], s['Lprime']['delta_r']),
                      (s['Nprime']['delta_a'], s['Nprime']['delta_r']),
                      (0, 0),
                      (0, 0),
                      (0, 0)))
        super().__init__(A, B)
        self._update_attributes()


    def update(self, dt, u=None):
        super().update(dt, u)
        self._update_attributes()


    def _update_attributes(self):
        self.v = self._x[0]
        self.p = self._x[1]
        self.r = self._x[2]
        self.roll = self._x[3]
        self.yaw = self._x[4]
        self.y = self._x[5]


class AircraftLongitudinal(AircraftDynamics):
    def __init__(self, derivatives):
        d = derivatives
        self._derivatives = derivatives
        self.g = d['g']
        self.U0 = d['U0']
        self.h0 = d['h0']
        self.alpha0 = d['alpha0']
        s = d['stability']
        # x = [ du dw dq dtheta dz ]
        A = np.array((
            (s['X']['u'], s['X']['w'], s['X']['q'], -d['g']*np.cos(np.deg2rad(d['theta0'])), 0),
            (s['Z']['u'], s['Z']['w'], s['Z']['q']+d['U0'], -d['g']*np.sin(np.deg2rad(d['theta0'])), 0),
            (s['M']['u']+s['M']['wdot']*s['Z']['u'],
             s['M']['w']+s['M']['wdot']*s['Z']['w'],
             s['M']['q']+s['M']['wdot']*(s['Z']['q']+d['U0']), 0, 0),
            (0, 0, 1, 0, 0),
            (0, -1, 0, d['U0'], 0)))

        # u = [ elevator thrust ]
        B = np.array(((s['X']['delta_e'], s['X']['delta_th']),
                      (s['Z']['delta_e'], s['Z']['delta_th']),
                      (s['M']['delta_e'] + s['M']['wdot']*s['Z']['delta_e'],
                       s['M']['delta_th'] + s['M']['wdot']*s['Z']['delta_th']),
                      (0, 0),
                      (0, 0)))
        super().__init__(A, B)
        self._update_attributes()
        self.x = 0.0


    def update(self, dt, u=None):
        super().update(dt, u)
        self._update_attributes()
        self.x += (self.u + self.U0)*dt


    def _update_attributes(self):
        self.u = self._x[0]
        self.w = self._x[1]
        self.q = self._x[2]
        self.pitch = self._x[3]
        self.z = self._x[4]

        self.h = self.h0 + self.z
