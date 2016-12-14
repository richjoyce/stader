import os
import json
import numpy as np
import scipy
import scipy.linalg


__all__ = ["load_aircraft", "read_json", "calculate_stability"]


def load_aircraft(name):
    import pkg_resources
    name = os.path.join('data', name)
    name += '.json'
    filename = pkg_resources.resource_filename(__name__, name)
    return read_json(filename)


def read_json(json_filename):
    with open(json_filename) as f:
        d = json.load(f)
    b = d['body']
    if not 'v' in b['Lprime'] and 'beta' in b['Lprime']:
        b['Lprime']['v'] = b['Lprime']['beta']/d['U0']
    if not 'v' in b['Nprime'] and 'beta' in b['Nprime']:
        b['Nprime']['v'] = b['Nprime']['beta']/d['U0']
    if not 'delta_a' in b['Y'] and 'delta_a' in b['Ystar']:
        b['Y']['delta_a'] = b['Ystar']['delta_a']*d['U0']
    if not 'delta_r' in b['Y'] and 'delta_a' in b['Ystar']:
        b['Y']['delta_r'] = b['Ystar']['delta_r']*d['U0']

    d = calculate_stability(d)
    return d


def calculate_stability(derivatives):
    """
    Calculate the stability-axis derivatives with the body-axis derivatives.
    """
    d = derivatives
    if 'stability' not in d:
        d['stability'] = {}
    slat = calculate_stability_lateral(d['body'], np.deg2rad(d['alpha0']))
    slong = calculate_stability_longitudinal(d['body'], np.deg2rad(d['alpha0']))
    d['stability'].update(slat)
    d['stability'].update(slong)
    return d


def calculate_stability_lateral(body, alpha):
    """
    Given a dictionary of body-axis derivatives, calculate the stability-axis derivatives and return in a dictionary.
    """
    body_arr = _lateral_dictionary_to_array(body)
    Tstab = _lateral_stability_to_body_matrix(alpha)
    Tbody = np.linalg.inv(Tstab)
    stab_arr = Tbody.dot(body_arr)
    return _lateral_array_to_dictionary(stab_arr)


def calculate_body_lateral(stability, alpha):
    """
    Given a dictionary of body-axis derivatives, calculate the stability-axis derivatives and return in a dictionary.
    """
    stab_arr = _lateral_dictionary_to_array(stability)
    Tstab = _lateral_stability_to_stab_matrix(alpha)
    body_arr = Tstab.dot(stab_arr)
    return _lateral_array_to_dictionary(body_arr)


def calculate_stability_longitudinal(body, alpha):
    """
    Given a dictionary of body-axis derivatives, calculate the stability-axis derivatives and return in a dictionary.
    """
    body_arr = _longitudinal_dictionary_to_array(body)
    Tstab = _longitudinal_stability_to_body_matrix(alpha)
    Tbody = np.linalg.inv(Tstab)
    stab_arr = Tbody.dot(body_arr)
    return _longitudinal_array_to_dictionary(stab_arr)


def calculate_body_longitudinal(stability, alpha):
    """
    Given a dictionary of body-axis derivatives, calculate the stability-axis derivatives and return in a dictionary.
    """
    stab_arr = _longitudinal_dictionary_to_array(stability)
    Tstab = _longitudinal_stability_to_stab_matrix(alpha)
    body_arr = Tstab.dot(stab_arr)
    return _longitudinal_array_to_dictionary(body_arr)


def _lateral_stability_to_body_matrix(alpha):
    cosa = np.cos(alpha)
    sina = np.sin(alpha)
    # Y is dependent on Y only
    Y_vpr = np.array(((1, 0, 0),
                      (0, cosa, -sina),
                      (0, sina, cosa)))
    Y_delta = np.identity(2)
    # L, N are interdependent
    LN_vpr = np.array(((cosa, 0, 0, -sina, 0, 0),
                       (0, cosa**2, -sina*cosa, 0, -sina*cosa, sina**2),
                       (0, sina*cosa, cosa**2, 0, -sina**2, -sina*cosa),
                       (sina, 0, 0, cosa, 0, 0),
                       (0, sina*cosa, -sina**2, 0, cosa**2, -sina*cosa),
                       (0, sina**2, sina*cosa, 0, sina*cosa, cosa**2)))
    LN_delta = np.array(((cosa, 0, -sina, 0),
                         (0, cosa, 0, -sina),
                         (sina, 0, cosa, 0),
                         (0, sina, 0, cosa)))
    # I's are interdependent
    I_mat = np.array(((cosa**2, sina**2, 2*sina*cosa),
                      (sina**2, cosa**2, -2*sina*cosa),
                      (-sina*cosa, sina*cosa, cosa**2-sina**2)))

    return scipy.linalg.block_diag(Y_vpr, LN_vpr, Y_delta, LN_delta, I_mat)


def _lateral_dictionary_to_array(dictionary):
    d = dictionary
    arr = []
    for ax in ['Y', 'Lprime', 'Nprime']:
        arr.extend([d[ax]['v'], d[ax]['p'], d[ax]['r']])
    for ax in ['Y', 'Lprime', 'Nprime']:
        arr.extend([d[ax]['delta_a'], d[ax]['delta_r']])
    arr.extend([d['I']['x'], d['I']['z'], d['I']['xz']])
    return arr

def _lateral_array_to_dictionary(array):
    d = {}
    for ax in ['Y', 'Lprime', 'Nprime', 'I']:
        d[ax] = {}
    i = 0
    for ax in ['Y', 'Lprime', 'Nprime']:
        for vpr in ['v', 'p', 'r']:
            d[ax][vpr] = array[i]
            i += 1
    for ax in ['Y', 'Lprime', 'Nprime']:
        for delta in ['delta_a', 'delta_r']:
            d[ax][delta] = array[i]
            i += 1
    for xz in ['x', 'z', 'xz']:
        d['I'][xz] = array[i]
        i += 1
    return d

def _longitudinal_dictionary_to_array(dictionary):
    d = dictionary
    arr = []
    for ax in ['X', 'Z']:
        arr.extend([d[ax]['u'], d[ax]['w'], d[ax]['q']])
    for ax in ['M']:
        arr.extend([d[ax]['u'], d[ax]['w'], d[ax]['wdot'], d[ax]['q']])
    for ax in ['X', 'Z', 'M']:
        arr.extend([d[ax]['delta_e'], d[ax]['delta_th']])
    return arr

def _longitudinal_array_to_dictionary(array):
    d = {}
    for ax in ['X', 'Z', 'M']:
        d[ax] = {}
    i = 0
    for ax in ['X', 'Z']:
        for uwq in ['u', 'w', 'q']:
            d[ax][uwq] = array[i]
            i += 1
    for ax in ['M']:
        for uwwdotq in ['u', 'w', 'wdot', 'q']:
            d[ax][uwwdotq] = array[i]
            i += 1
    for ax in ['X', 'Z', 'M']:
        for delta in ['delta_e', 'delta_th']:
            d[ax][delta] = array[i]
            i += 1
    return d

def _longitudinal_stability_to_body_matrix(alpha):
    cosa = np.cos(alpha)
    sina = np.sin(alpha)
    # X, Z are interdependent
    XZ_uwq = np.array(((cosa**2, -sina*cosa, 0, -sina*cosa, sina**2, 0),
                       (sina*cosa, cosa**2, 0, -sina**2, -sina*cosa, 0),
                       (0, 0, cosa, 0, 0, -sina),
                       (sina*cosa, -sina**2, 0, cosa**2, -sina*cosa, 0),
                       (sina**2, sina*cosa, 0, sina*cosa, cosa**2, 0),
                       (0, 0, sina, 0, 0, cosa)))
    XZ_delta = np.array(((cosa, 0, -sina, 0),
                         (0, cosa, 0, -sina),
                         (sina, 0, cosa, 0),
                         (0, sina, 0, cosa)))
    # M is dependent on M only
    M_uwwdotq = np.array(((cosa, -sina, 0, 0),
                         (sina, cosa, 0, 0),
                         (0, 0, cosa, 0),
                         (0, 0, 0, 1)))
    M_delta = np.identity(2)

    return scipy.linalg.block_diag(XZ_uwq, M_uwwdotq, XZ_delta, M_delta)
