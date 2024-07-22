from random import random
import math


class Point2D:
    def __init__(self, x=0.0, y=0.0, h=0.0):
        self.x = x
        self.y = y
        self.h = h

    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def setH(self, h):
        self.h = h

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getH(self):
        return self.h
class Joint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.t = 0.0
        self.w = [0.0, 0.0, 0.0, 0.0]
def toRad(degree):
    return degree * math.pi
motor = Joint()
class Kinematic:
    """
    wheel front right = 1
    wheel front left = 2
    wheel back = 0
    #define WHEEL_RADIUS 0.063                     // [m]
    #define DISTANCE_WHEEL_TO_ROBOT_CENTRE 0.1826  // [m]
    """
    
    R_roda = 0.05
    lx = 0.228
    ly =  0.158
    kRoda = 2*math.pi*R_roda
    anglesM = [270, 30, 30]
    wheel = [0.0 ,0.0 ,0.0, 0.0]
    
    def __init__(self):
        self.globalAngle = 0.0
        self.imu_available = 0
        self.enc = [0, 0, 0,0]
        self.Venc = [0, 0, 0,0]
        self.encprev = [0, 0, 0,0]
        self.motorSpeeds = [0,0,0,0]

    def check_imu(self, imu_condition):
        self.imu_available = 1 if imu_condition else 0

    def toRad(self, degree):
        return degree * math.pi / 180

    def forward(self, outFor, m1, m2, m3, m4):
        outFor.setX((m1 + m2 + m3 + m4) * self.R_roda / 4)
        outFor.setY((-m1 + m2 + m3 - m4) * self.R_roda / 4)
        outFor.setH((-m1 + m2 - m3 + m4) * self.R_roda / (4 * (self.lx + self.ly)))

    def clip(self, value, min_value=-6.28, max_value=6.28):
        return max(min(value, max_value), min_value)
    def inverse(self,motor, vx, vy, vh):
        motor.w[0] = (vx + vy + (self.lx + self.ly) * vh)/self.R_roda
        motor.w[1] = (vx - vy - (self.lx + self.ly) * vh)/self.R_roda
        motor.w[2] = (vx - vy + (self.lx + self.ly) * vh)/self.R_roda
        motor.w[3] = (vx + vy - (self.lx + self.ly) * vh)/self.R_roda
        # for i in range(4):
        #     if(self.motorSpeeds[i] > 6.28): self.motorSpeeds[i] = 6.28
        #     elif(self.motorSpeeds[i] < -6.28): self.motorSpeeds[i] = -6.28
        #     elif(self.motorSpeeds[i] > -6.28 and self.motorSpeeds[i] < 6.28):
        #         self.motorSpeeds[i] = self.motorSpeeds[i]
        #     else: self.motorSpeeds[i] = 0
        
        # return motor

    def setCurrentPos(self, x, y, t):
        self.pos.x = x
        self.pos.y = y
        self.pos.t = t

    def angleNormalize(self, angle):
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle      
        
        