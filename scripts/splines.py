    #!/usr/bin/env python3
#
#   splines.py
#
#   TO IMPORT, ADD TO YOUR CODE:
#   from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5
#
#
#   This library provides the linear, cubic, and quintic spline trajectory
#   segment classes.  I separated from the main code, just so I don't
#   have to copy/paste it again and again....
#
#   The classes are:
#
#      seg = CubicSpline(p0, v0, pf, vf, T, space='Joint')
#      seg = Goto(       p0,     pf,     T, space='Joint')
#      seg = Hold(       p,              T, space='Joint')
#      seg = Stay(       p,                 space='Joint')
#
#      seg = QuinticSpline(p0, v0, a0, pf, vf, af, T, space='Joint')
#      seg = Goto5(        p0,         pf,         T, space='Joint')
#
#   Each spline object then provides a
#
#      (pos, vel) = seg.evaluate(t)
#      T          = seg.duration()
#      space      = seg.space()
#
#   That is, each segment can compute the position and velocity for
#   the specified time.  Note it can also report which "space" it
#   wants, but inthe end it is the user's responsibility to
#   interpret/use the pos/vel information accordingly.
#
#   The segments also assume a t=0 start time.  That is, when calling
#   evaluate(), please first subtract the start time of the segment!
#   Note: This has now been updated to account for the start time of each segment
#
#   The p0,pf,v0,vf,a0,af may be NumPy arrays.
#
import math
import numpy as np

#
#  Cubic Segment Objects
#
#  These compute/store the 4 spline parameters, along with the
#  duration and given space.  Note the space is purely for information
#  and doesn't change the computation.
#
class CubicSpline:
    # Initialize.
    def __init__(self, t0, p0, v0, pf, vf, T, space='Joint'):
        # Precompute the spline parameters.
        self.T = T
        self.a = p0
        self.b = v0
        self.c =  3*(pf-p0)/T**2 - vf/T    - 2*v0/T
        self.d = -2*(pf-p0)/T**3 + vf/T**2 +   v0/T**2
        self.t0 = t0
        # Save the space
        self.usespace = space

    # Return the segment's space
    def space(self):
        return self.usespace

    # Report the segment's duration (time length).
    def duration(self):
        return(self.T)

    # Compute the position/velocity for a given time.
    def evaluate(self, t):
        dt = t - self.t0
        # Compute and return the position and velocity.
        p = self.a + self.b * dt +   self.c * dt**2 +   self.d * dt**3
        v =          self.b     + 2*self.c * dt    + 3*self.d * dt**2
        return (p,v)

class Goto(CubicSpline):
    # Use zero initial/final velocities (of same size as positions).
    def __init__(self, t0, p0, pf, T, space='Joint'):
        CubicSpline.__init__(self, t0, p0, 0*p0, pf, 0*pf, T, space)

class Hold(Goto):
    # Use the same initial and final positions.
    def __init__(self, t0, p, T, space='Joint'):
        Goto.__init__(self, t0, p, p, T, space)

class Stay(Hold):
    # Use an infinite time (stay forever).
    def __init__(self, t0, p, space='Joint'):
        Hold.__init__(self, t0, p, math.inf, space)


#
#  Quintic Segment Objects
#
#  These compute/store the 6 spline parameters, along with the
#  duration and given space.  Note the space is purely for information
#  and doesn't change the computation.
#
class QuinticSpline:
    # Initialize.
    def __init__(self, t0, p0, v0, a0, pf, vf, af, T, space='Joint'):
        # Precompute the six spline parameters.
        self.T = T
        self.a = p0
        self.b = v0
        self.c = a0
        self.d = -10*p0/T**3 - 6*v0/T**2 - 3*a0/T    + 10*pf/T**3 - 4*vf/T**2 + 0.5*af/T
        self.e =  15*p0/T**4 + 8*v0/T**3 + 3*a0/T**2 - 15*pf/T**4 + 7*vf/T**3 -   1*af/T**2
        self.f =  -6*p0/T**5 - 3*v0/T**4 - 1*a0/T**3 +  6*pf/T**5 - 3*vf/T**4 + 0.5*af/T**3
        self.t0 = t0
        #print(t0)
        # Also save the space
        self.usespace = space

    # Return the segment's space
    def space(self):
        return self.usespace

    # Report the segment's duration (time length).
    def duration(self):
        return(self.T)
    
    # Report the segment's starting time.
    def start_time(self):
        return(self.t0)

    # Compute the position/velocity for a given time.
    def evaluate(self, t):
        # Compute and return the position and velocity.
        dt = t - self.t0
        p = self.a + self.b * dt +   self.c * dt**2 +   self.d * dt**3 +   self.e * dt**4 +   self.f * dt**5
        v =          self.b     + 2*self.c * dt    + 3*self.d * dt**2 + 4*self.e * dt**3 + 5*self.f * dt**4
        return (p,v)

class Goto5(QuinticSpline):
    # Use zero initial/final velocities/accelerations (same size as positions).
    def __init__(self, t0, p0, pf, T, space='Joint'):
        QuinticSpline.__init__(self, t0, p0, 0*p0, 0*p0, pf, 0*pf, 0*pf, T, space)
        

#
#  Linear Segment Objects
#
#  These compute/store the 2 spline parameters, along with the
#  duration and given space.  Note the space is purely for information
#  and doesn't change the computation.
#
class LinearSpline:
    # Initialize.
    def __init__(self, t0, p0, pf, T, space='Joint'):
        # Precompute the two spline parameters.
        self.T = T
        self.a = p0
        self.b = (pf-p0)/T
        self.t0 = t0
        # Also save the space
        self.usespace = space

    # Return the segment's space
    def space(self):
        return self.usespace

    # Report the segment's duration (time length).
    def duration(self):
        return(self.T)

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        dt = t - self.t0
        p = self.a + self.b * dt
        v =          self.b
        return (p,v)

class GotoLin(LinearSpline):
    # Uses discontinuous velocity values.
    def __init__(self, t0, p0, pf, T, space='Joint'):
        LinearSpline.__init__(self, t0, p0, pf, T, space)
        
        
#
#  Sinusoidal Object in only the X-direction
#
#  This object compute/store the 4 sine parameters, along with the
#  duration and given space.  Only the phase shift is calculated.
#  Note the space is only for information; it doesn't change the computation.
#
class SineX:
    # Initialize.
    def __init__(self, pos, t, amp, freq, T, space='Task'):
        # Precompute the sinusoidal parameters
        self.T = T                                  # Duration
        self.A = amp                                # Amplitude
        self.w = freq                               # Frequency                 
        self.tau = math.asin(pos[0]/amp) - freq*t   # Phase Shift
        #print('First Calc:\n', self.tau)
        #self.tau = 
        self.y = pos[1]                             # Y-Pos (since constant)
        self.z = pos[2]                             # Z-Pos (since constant)
        # Also save the space
        self.usespace = space

    # Return the segment's space
    def space(self):
        return self.usespace

    # Report the segment's duration (time length).
    def duration(self):
        return(self.T)

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        x = self.A*math.sin(self.w*t + self.tau)
        xdot = self.A*self.w*math.cos(self.w*t + self.tau)
        p = [x, self.y[0], self.z[0]]
        v = [xdot, 0.0, 0.0]
        return (p,v)

