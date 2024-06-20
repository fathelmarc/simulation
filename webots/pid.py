import time
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()  # Initialize previous time

    def calculatePID(self, error, max_value, is_angle):
        current_time = time.time()
        deltaT = current_time - self.prev_time
        if deltaT == 0:
            deltaT = 1e-6  # Assign a small value to deltaT to avoid division by zero
        self.prev_time = current_time

        self.e_proportional = error
        self.integral += error * deltaT
        self.e_derivative = (self.e_proportional - self.prev_error) / deltaT
        self.prev_error = error

        output = self.kp * self.e_proportional + self.ki * self.integral + self.kd * self.e_derivative

        # if is_angle:
        #     output = (output + 3.14159) % (2 * 3.14159) - 3.14159

        return max(min(output, max_value), -max_value)

# class PID:
#     def __init__(self, kp, ki, kd):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd

#         self.prev_error = 0.0
#         self.e_integral = 0.0
#         self.e_derivative = 0.0
#         self.e_proportional = 0.0
#         self.u = 0.0
#         self.prev_time = time.time()

#     def delta(self):
#         current_time = time.time()
#         delta_time = current_time - self.prev_time
#         self.prev_time = current_time   
#         return delta_time
    
#     def calculatePID(self, error, limit, condition):
#         if condition:
#             self.e_proportional = error
#             if self.e_proportional > 180:
#                 self.e_proportional -= 360
#             elif self.e_proportional < -180:
#                 self.e_proportional += 360
#         else:
#             self.e_proportional = error

#         deltaT = self.delta()
#         self.e_integral += self.e_proportional * deltaT
#         self.e_derivative = (self.e_proportional - self.prev_error) / deltaT
#         self.prev_error = self.e_proportional

#         self.u = self.kp * self.e_proportional + self.ki * self.e_integral + self.kd * self.e_derivative
        
#         return max(min(self.u, limit), -limit)
#         # return max(-limit, min(self.u, limit))