__author__ = 'gleicher'

"""
infrastructure for spacetime problems

at each time, there needs to be a STATE for the robot
these states are created by interpolating the KEYS

a state sequence is something that provides a state vector (for a robot)
at every frame in the movement. it might be stored as a keyvector
(or computed from a KeyVector). but it is very basic.
    Note: states (and therefore keyvectors) give array access to the states

a KEYVECTOR is a big vector that concatenates all of the keys
    in the future, it might be best if this isn't actually just a vector

    it might be a list or a numpy array
    it might have adnumbers or oovars in it
    but its a big flat list of length n*m  (keys * vars per state)

    there are utility routines for handling them

    to make things functional, its probably best if this is just a vector
    however, we need to keep n and m around with it, so it cannot be

    a key vector is designed so that it can act like an StateSequence


a KEYVARIABLES is a vector of the variables that are actually changing in the
    spacetime problem
    there is a trick that not everything in the keyvector will be allowed to change
        (for example, endpoints to interpolate)
    so we need to "expand" a set of KEYVARIABLES into a KEYVECTOR

the basic strategy is most likely to be:
1) build a KeyVector (like an initial configuration for a solver)
2) extract KeyVariables (a smaller vector subset)
3) inject KeyVariables into the initial KeyVector (to produce a new KeyVector)
4) use an interpolator, or the KeyVector directly as a state list

for now, injection / extraction creates new objects - which is wastefull for copying,
but gets around the statedness problems. in the future, this can be fixed if there are
efficiency concerns
"""
import numpy as N


class SSIterator:
    def __init__(self, SS):
        self.seq = SS
        self.pos = 0

    def next(self):
        v = self.seq[self.pos]
        if self.pos == self.seq.nkeys:
            raise StopIteration
        else:
            self.pos += 1
        return v


class StateSequence:
    def __init__(self, _nkeys, _nvars):
        self.nkeys = _nkeys
        self.nvars = _nvars

    def getState(self, k):
        raise NotImplementedError

    def __getitem__(self, item):
        raise NotImplementedError

    def __len__(self):
        raise NotImplementedError

    def __iter__(self):
        return SSIterator(self)

    def getSignal(self, k):
        """
        this gets the signal for one of the variables across the entire range of states
        """
        return N.array([s[k] for s in self])


class KeyVector(StateSequence):
    def __init__(self, _nkeys, _nvars, _vector=None):
        StateSequence.__init__(self, _nkeys, _nvars)
        self.lastExtractExcludeKeys = []
        if _vector is None:
            self.vector = N.zeros((self.nkeys * self.nvars))
        else:
            # try to be clever about converting correctly
            if len(_vector) == self.nkeys * self.nvars:
                self.vector = _vector
            else:
                try:  # see if we have a KeyVector - assume we do
                    if _vector.nkeys == _nkeys and _vector.nvars == _nvars:
                        self.vector = _vector.vector
                    else:
                        raise RuntimeError("wrong shaped KeyVector to copy")
                except:
                    raise RuntimeError("Cannot turn object into vector")

    def getKey(self, k):
        return self.vector[k * self.nvars: (k + 1) * self.nvars]

    def __len__(self):
        return self.nkeys

    def __getitem__(self, item):
        return self.getKey(item)

    # this works because we set a vector equal to a vector (or so we hope)
    def __setitem__(self, k, value):
        if k < 0:
            k += len(self)
        self.vector[k * self.nvars: (k + 1) * self.nvars] = value

    def setKeyToValue(self, key, value):
        self.vector[key * self.nvars: (key + 1) * self.nvars] = value

    def extract(self, excludeKeys=None):
        self.lastExtractExcludeKeys = excludeKeys
        if excludeKeys == None or excludeKeys == []:
            # special case, we can just copy the list
            return N.array(self.vector)
        elif type(excludeKeys) == list or type(excludeKeys) == tuple:
            # we assume these are the things to EXCLUDE
            selects = [i for i in range(self.nkeys) if i not in excludeKeys]
            return N.hstack([self.getKey(i) for i in selects])

    def lenExtract(self, excludeKeys=None):
        """
        gives the length of what the extracted vector should be
        """
        if excludeKeys == None or excludeKeys == []:
            return len(self.vector)
        else:
            return self.nvars * (self.nkeys - len(excludeKeys))

    def inject(self, keyvariables, excludeKeys=None):
        # this is a horrifically naive way to implement this...
        if excludeKeys == None or excludeKeys == []:
            return KeyVector(self.nkeys, self.nvars, keyvariables)
        else:
            klist = []
            kv = 0
            for i in range(self.nkeys):
                if i in excludeKeys:
                    klist.append(self.getKey(i))
                else:
                    klist.append(keyvariables[kv:kv + self.nvars])
                    kv += self.nvars
            return KeyVector(self.nkeys, self.nvars, N.hstack(klist))

    def lerp(self, keytimes):
        """
        does linear interpolation between the specified frame times
        fills all frames before keytimes[0] with keytime 0
        fills all frames after keytimes[-1] with keytime -1
        lerps the others
        """
        # fill the initial frames
        for t in range(0, keytimes[0]):
            self[t] = self[keytimes[0]]
        # fill the frames at the end
        for t in range(keytimes[-1] + 1, len(self)):
            self[t] = self[keytimes[-1]]
        # now take each pair to interpolate
        for ip in range(len(keytimes) - 1):
            a = keytimes[ip]
            b = keytimes[ip + 1]
            for t in range(a + 1, b):
                alpha = float(t - a) / float(b - a)
                self[t] = self[a] * (1 - alpha) + self[b] * alpha
