'''
pidcontrol.py - PID Controller classes for Python

    For details see http://en.wikipedia.org/wiki/PID_controller

    Copyright (C) 2014 Bipeen Acharya, Fred Gisa, and Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

'''

import math

class PID_Controller(object):
    '''
    General PID control class. 
    '''

    def __init__(self, Kp, Ki, Kd):
        '''
        Constructs a new PID_Controller object.
        '''
        
        # Parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # State variables
        self.Eprev = 0
        self.Stdt = 0
        self.t = 0
    def tune(self,KpNew,KiNew,KdNew):
        self.Kp = KpNew
        self.Ki = KiNew
        self.Kd = KdNew
    def getCorrection(self, target, actual, dt=1):
        '''
        Returns current PID correction based on target value and actual value.
        '''
              
        E = target - actual
    
        # dE / dt
        dEdt = (E - self.Eprev) / dt if self.t > 0 else 0
        
        #if abs(dEdt) > 1: # XXX Why?
        #    dEdt = 0 # XXX Why?
        
        # Integral E / dt XXX not how you actually integrate
        self.Stdt += E*dt if self.t > 0 else 0# (E + self.Eprev)*dt if self.t > 0 else 0
   
        # Correcting

        correction = self.Kp*E + self.Ki*self.Stdt + self.Kd*dEdt
    
        # Update
        self.t += 1
        self.Eprev = E

        return correction
        
class Demand_PID_Controller(PID_Controller):
    '''
    A class to handle the interaction of demand (joystick, transmitter) and PID control.
    Control switches from demand to PID when demand falls below a given threshold.
    '''
    
    def __init__(self, Kp, Kd, Ki=0, demand_noise_threshold=.01):
        '''
        Creates a new Demand_PID_Controller.
        '''
        
        PID_Controller.__init__(self, Kp, Ki, Kd)
        
        # Noise threshold for demand
        self.noise_threshold = demand_noise_threshold

        self.prevAbsDemand = 1      # Handle initial condition
        self.target        = None

        # State variables
        self.Eprev = 0
        self.Stdt = 0
        self.t = 0
 

    def getCorrection(self, sensorValue, demandValue, timestep=1):
        '''
        Returns current PID correction based on sensor value and demand value.
        '''
        
        # Assume no correction
        correction = 0
        
        # If there is currently no demand
        if abs(demandValue) < self.noise_threshold:
        
            # But there was previously a demand
            if self.prevAbsDemand > self.noise_threshold:
            
                # Grab the current sensor value as the target
                self.target = sensorValue
                              
            # With no demand, always need a correction 
            correction = PID_Controller.getCorrection(self, self.target, sensorValue, timestep)
                    
        # Track previous climb demand, angle    
        self.prevAbsDemand = abs(demandValue)            
                                
        return correction

class GPS_PID_Controller(PID_Controller):
    '''
    A class to support Computer-Assisted piloting to a GPS location
    '''
    
    def __init__(self, Kp, Kd, Ki=0):
        '''
        Creates a new Auto_PID_Controller
        '''

        PID_Controller.__init__(self, Kp, Ki, Kd)

        # Noise threshold for demand
        self.noise_threshold = 0.01#demand_noise_threshold

        self.prevAbsDemand = 1 # Handle initial condition
        self.target        = None

        # State variables
        self.Eprev = 0.0
        self.Stdt = 0.0
        self.t = 0.0
 

    def getCorrection(self, targetValue, sensorValue, dt=1):
        '''
        Returns current Angle PID correction based on sensor and target values.
        '''

        # Assume no correction
        correction = 0
              
        E = targetValue - sensorValue
    
        dEdt = (E - self.Eprev) / dt if self.t > 0 else 0

        self.Stdt += E*dt if self.t > 0 else 0# (E + self.Eprev)*dt if self.t > 0 else 0
   
        correction = self.Kp*E + self.Ki*self.Stdt + self.Kd*dEdt
    
        self.t += 1
        self.Eprev = E
        
        return correction if abs(correction) < 1 else correction*1/abs(correction)

class Stability_PID_Controller(PID_Controller):
    '''
    A class to support pitch/roll stability.  K_i parameter and target angle are zero.
    '''
    
    def __init__(self, Kp, Kd, Ki=0):
        '''
        Creates a new Stability_PID_Controller.
        '''
        
        PID_Controller.__init__(self, Kp, Ki, Kd)
        
    def getCorrection(self, actualAngle, timestep=1):
        '''
        Returns current PID correction based on IMU angle in radians.
        '''

        return PID_Controller.getCorrection(self, 0, actualAngle, timestep)
        

class Yaw_PID_Controller(Demand_PID_Controller):
    '''
    A class for PID control of quadrotor yaw.
    Special handling is needed for yaw because of numerical instabilities when angle approaches Pi radians
    (180 degrees).
    '''
    
    def __init__(self, Kp, Kd, Ki,  demand_noise_threshold=.01):
        '''
        Creates a new Yaw_PID_Controller.  K_i is set to zero.
        '''
        
        #XXX Attempting to integrate Integral correction

        Demand_PID_Controller.__init__(self, Kp, Kd, Ki, demand_noise_threshold)
        
    def getCorrection(self, yawAngle, yawDemand, timestep=1):
        '''
        Returns current PID correction based on yaw angle in radians value and demand value in interval [-1,+1].
        '''
        
        correction =  Demand_PID_Controller.getCorrection(self, -yawAngle, yawDemand, timestep)

        return correction if abs(correction) < 10 else 0 # XXX This is a #@!&ty way to angle-correct. why in the world.

class Hover_PID_Controller(PID_Controller):
    '''
    A class for Hover-In-Place (position / altitude hold).
    '''
    
    def __init__(self, Kp, Kd=0, Ki=0, max_correction = 0.5):
        '''
        Creates a new Hover_PID_Controller.
        '''
 
        PID_Controller.__init__(self, Kp, Ki, Kd)
    
        # Previous position for velocity calculation
        self.position_prev = None

        # Magnitude of maximum correction
        self.max_correction = max_correction

        
    def getCorrection(self, position, target=None, timestep=1):
        '''
        Returns current PID correction based on position and stick demand.
        If no target is specified, zero velocity is used as target.
        '''

        # Velocity is not available on first invocation
        velocity = 0

        # Correction is not useful when there's a stick demand
        correction = 0

        # Compute velocity when possible
        if self.position_prev:
            velocity = (position - self.position_prev) / timestep

        # Track previous position
        self.position_prev = position

        # If there's a target, use it; otherwise use zero velocity as target
        correction = PID_Controller.getCorrection(self, target, position, timestep) \
                     if target \
                     else PID_Controller.getCorrection(self, 0, velocity, timestep) \

        # Keep within limits
        return min(max(correction, -self.max_correction), +self.max_correction)
