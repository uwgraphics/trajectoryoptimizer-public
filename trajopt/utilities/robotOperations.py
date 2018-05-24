import trajopt.utilities.adInterface as AD
import numpy as N
import math

sin = AD.MATH.sin
cos = AD.MATH.cos


#####
# straightforward way to deal with the linear algebra
# not necessarily efficient
def rotMatrix(axis, s, c):
    if axis == "Z" or axis == "z":
        return N.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    elif axis == "Y" or axis == "y":
        return N.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
    elif axis == "X" or axis == "x":
        return N.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
    else:
        print "Unsupported Axis:", axis
        raise NotImplementedError


def rotTransMatrix(axis, s, c, t):
    """
    build a rotate * translate matrix - MUCH faster for derivatives
    since we know there are a ton of zeros and can act accordingly
    :param axis: x y or z as a character
    :param s: sin of theta
    :param c: cos of theta
    :param t: translation (a 3 tuple)
    :return:
    """
    if axis == "Z" or axis == "z":
        return N.array([[c, -s, 0, AD.fastLC2(c, t[0], s, -t[1])],
                        [s, c, 0, AD.fastLC2(s, t[0], c, t[1])],
                        [0, 0, 1, t[2]],
                        [0, 0, 0, 1]])
    elif axis == "Y" or axis == "y":
        return N.array([[c, 0, s, AD.fastLC2(c, t[0], s, t[2])],
                        [0, 1, 0, t[1]],
                        [-s, 0, c, AD.fastLC2(s, -t[0], c, t[2])],
                        [0, 0, 0, 1]])
    elif axis == "X" or axis == "x":
        return N.array([[1, 0, 0, t[0]],
                        [0, c, -s, AD.fastLC2(c, t[1], s, -t[2])],
                        [0, s, c, AD.fastLC2(s, t[1], c, t[2])],
                        [0, 0, 0, 1]])
    else:
        print "Unsupported Axis:", axis
        raise NotImplementedError


def rot3(axis, s, c):
    """
    build a rotate * translate matrix - MUCH faster for derivatives
    since we know there are a ton of zeros and can act accordingly
    :param axis: x y or z as a character
    :param s: sin of theta
    :param c: cos of theta
    :param t: translation (a 3 tuple)
    :return:
    """
    if axis == "Z" or axis == "z":
        return N.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    elif axis == "Y" or axis == "y":
        return N.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    elif axis == "X" or axis == "x":
        return N.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    else:
        print "Unsupported Axis:", axis
        raise NotImplementedError


def eulerTupleTo3x3(t):
    """
    given an XYZ tuple, return a rotation matrix

    the order is ZYX because angles are in the parent system

    note: this does it the slow way, but that's OK, since this is only used at robot setup
    :param t: a tuple (x,y,z)
    :return: a 3x3 matrix
    """
    xm = rot3('X', math.sin(t[0]), math.cos(t[0]))
    ym = rot3('Y', math.sin(t[1]), math.cos(t[1]))
    zm = rot3('Z', math.sin(t[2]), math.cos(t[2]))
    #    xy = xm.dot(ym)
    #    return xy.dot(zm)
    zy = zm.dot(ym)
    return zy.dot(xm)


def rotTransMatrixNOAD(axis, s, c, t):
    """
    build a rotate * translate matrix - MUCH faster for derivatives
    since we know there are a ton of zeros and can act accordingly
    :param axis: x y or z as a character
    :param s: sin of theta
    :param c: cos of theta
    :param t: translation (a 3 tuple)
    :return:
    """
    if axis == "Z" or axis == "z":
        return N.array([[c, -s, 0, c * t[0] - s * t[1]],
                        [s, c, 0, s * t[0] + c * t[1]],
                        [0, 0, 1, t[2]],
                        [0, 0, 0, 1]])
    elif axis == "Y" or axis == "y":
        return N.array([[c, 0, s, c * t[0] + s * t[2]],
                        [0, 1, 0, t[1]],
                        [-s, 0, c, c * t[2] - s * -t[0]],
                        [0, 0, 0, 1]])
    elif axis == "X" or axis == "x":
        return N.array([[1, 0, 0, t[0]],
                        [0, c, -s, c * t[1] - s * t[2]],
                        [0, s, c, s * t[1] + c * t[2]],
                        [0, 0, 0, 1]])
    else:
        print "Unsupported Axis:", axis
        raise NotImplementedError


def transMatrix(vector):
    m = N.eye(4)
    m[0:3, 3] = vector[0:3]
    return m


def multV(matrix, vector):
    if len(vector) == 3:
        return N.dot(matrix, (vector[0], vector[1], vector[2], 1))[0:3]
    else:
        return N.dot(matrix, vector)


def translate(matrix, vector):
    matrix[0:3, 3] += vector[0:3]


def getTrans(matrix):
    return matrix[0:3, 3]


twopi = 2 * math.pi


def despin(a):
    """
    this makes an angle be +/- pi
    """
    na = ((a + math.pi) % twopi) - math.pi
    return na


def despinSeries(array):
    """
    does the de-spin on an array. rather than having an absolute boundary, flips things such that
    its close to the prior. this begins at the first value and goes from there. it does not change
    the first value.
    it modifies the array in place
    :param array: should be able to take a keyvector
    :return: the number of elements changed
    """
    nchanged = 0
    for i in range(1, len(array)):
        cur = array[i]
        prev = array[i - 1]
        for j in range(len(cur)):
            d = cur[j] - prev[j]
            if d > 3:
                cur[j] -= twopi
                nchanged += 1
            elif d < -3:
                cur[j] += twopi
                nchanged += 1
    return nchanged


# despin a series given as a vector - but we need to know the number vars and states
def despinArray(array, nvars):
    nstates = len(array) / nvars
    nchanged = 0
    for i in range(1, nstates):
        for j in range(nvars):
            idx = i * nvars + j
            d = array[idx] - array[(i - 1) * nvars + j]
            if d > 3:
                array[idx] -= twopi
                nchanged += 1
            elif d < -3:
                array[idx] += twopi
                nchanged += 1
    return nchanged


# naively assume that all variables are angles, and should be "de-spun"
def deSpinCB(dsv):
    for i in range(len(dsv)):
        dsv[i] = despin(dsv[i])


#
# if we model things with variables "s" and "c" compute the X and Y from them
# this should be just x = c and y = s, but we need to deal with normalization
def normSC(s, c):
    d2 = s * s + c * c
    if d2 > .001:
        d = AD.MATH.sqrt(d2)
    else:
        d = math.sqrt(d2)
    if d > .1:
        return s / d, c / d
    else:
        # things have vanished, so we basically we have no transform
        # we could just return the identity, but we want to fade to that
        # when d = .01, return s,1+c
        if d < .01:
            d = .01
        a = (d - .01) / .09  # .1-.01
        a1 = 1 - a
        return a * s / d + a1 * s, a1 * (1 + c) + a * c / d
