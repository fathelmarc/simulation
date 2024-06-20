from kinematic import *
from controller import Robot
from controller import InertialUnit
from pid import PID
import math
robot = Robot()
timestep = 64
max_speed = 6.28
motor_names = ['wheel2', 'wheel1', 
                'wheel4', 'wheel3']
motors = [robot.getDevice(name) for name in motor_names]
enc_names = ['wheel2sensor', 'wheel1sensor',
                'wheel4sensor', 'wheel3sensor']
encs = [robot.getDevice(name) for name in enc_names]
imu = robot.getDevice('inertial unit')
imu.enable(timestep)
for enc in encs:
    enc.enable(timestep)
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

pid_control = PID(1, 0, 0)
pidH_control = PID(1, 0, 0)

def toDeg(radian):
    return radian * 180/math.pi
input =Joint()
class Kuka(Kinematic):
    enc_values = [0,0,0,0]
    wheel_values = [0,0,0,0]
    last_enc_values = [0,0,0,0]
    diff = [0,0,0,0]
    gyro = [0,0,0]
    motorSpeeds = [0,0,0]
    encoder_unit = Kinematic.kRoda * 7
    def __init__(self):
        Kinematic().__init__()
    def setPos(self, x, y, h):
        while True:
            gyro = imu.getRollPitchYaw()
            for i in range(4):
                self.enc_values[i] = enc.getValue()
                tick = self.enc_values[i] - self.last_enc_values[i]
                if tick < 0.001:
                    tick = 0
                    self.enc_values[i] =  self.last_enc_values[i]
                self.wheel_values[i] = self.enc_values[i] * self.encoder_unit
                self.last_enc_values =self.enc_values

            pos = Point2D()
            self.forward(pos,self.wheel_values[0], self.wheel_values[1],
                        self.wheel_values[2], self.wheel_values[3])
            errorX = x - pos.getX()
            errorY = y - pos.getY()
            errorH = h - gyro[0]

            errorXY = math.sqrt(math.pow(errorX,2) + math.pow(errorY, 3))
            errorHead = math.atan2(errorX, errorY)

            controlXY = pid_control.calculatePID(errorXY, 6.28, False)
            controlHead = pidH_control.calculatePID(errorH, 6.28,True)

            vel = [0, 0, 0]
            vel[0] = controlXY * math.cos(errorHead)
            vel[1] = controlXY * math.sin(errorHead)
            vel[2] = controlHead

            self.inverse(input,vel[0], vel[1], vel[2],)
            print("\nx:", pos.getX(),
                "\ny:", pos.getY(),
                "\nh:", pos.getH(),
                "\ng:", gyro[0],
                "\nvelX:", vel[0])
            for i in range(4):
                motors[i].setVelocity(max(-6.28,min(input.w[i],6.28)))
            if errorX < 1 and errorY < 1 and errorX > -1 and errorY > -1:
                break

            """
            if(errorX < 1 && errorY < 1 && errorX > -1 && errorY > -1 || caseBot == 0){
                break;
            }
            """


if __name__ == "__main__":
    kinematic = Kinematic()
    encoder_unit = kinematic.kRoda * 7
    while robot.step(timestep) != -1:

        # compute angular vel and linear vel for robot
        robot_pos = Point2D()
        kuka = Kuka()
        kuka.setPos(10,0,0)
        kuka.setPos(10,10,0)
        kuka.setPos(0,10,0)
        kuka.setPos(0,0,0)
        # print("\nrobot in X: ", robot_pos.getX(),
        #       "\nrobot in Y: ", robot_pos.getY(),
        #       "\nrobot in h: ", robot_pos.getH(),
        #       "\nrobot in H: ", kuka.gyro[0],
        #       "\n\nrobot wheel dist:", kuka.wheel_values[0])
