#!/usr/bin/env python3
import math

import rpyc

# Class for connecting and communicating with the robot
# Source: LEGO, https://pybricks.com/ev3-micropython
class Remote:
    
    def __init__(self):
        # Create a RPyC connection to the remote ev3dev device.
        # Use the hostname or IP address of the ev3dev device.
        # If this fails, verify your IP connectivty via ``ping X.X.X.X``
        conn = rpyc.classic.connect("ev3dev")
        print('connected')

        # import ev3dev2 on the remote ev3dev
        self.ev3dev2_motor = conn.modules['ev3dev2.motor']
        self.ev3dev2_sensor = conn.modules['ev3dev2.sensor.lego']
        self.ev3dev2_sound = conn.modules['ev3dev2.sound']

        self.motor_right = self.ev3dev2_motor.MediumMotor(
            self.ev3dev2_motor.OUTPUT_C)
        self.motor_left = self.ev3dev2_motor.MediumMotor(
            self.ev3dev2_motor.OUTPUT_D)
        self.tank = self.ev3dev2_motor.MoveTank(
            self.ev3dev2_motor.OUTPUT_A, self.ev3dev2_motor.OUTPUT_B)

        # dist_sensor = ev3dev2_sensor.ColorSensor()
        self.tank.gyro = self.ev3dev2_sensor.GyroSensor()
        self.tank.gyro.calibrate()

    def consume_balls(self):
        self.motor_right.run_forever(speed_sp=1000)
        self.motor_left.run_forever(speed_sp=-1000)

    def eject_balls(self):
        self.motor_right.run_forever(speed_sp=-600)
        self.motor_left.run_forever(speed_sp=600)

    def stop_balls_mec(self):
        self.motor_left.stop()
        self.motor_right.stop()

    def go_forward(self):
        self.tank.on(50, 50)

    def go_forward_rotations(self):
        self.tank.on_for_rotations(50, 50, 1)

    def go_back_rotations(self):
        self.tank.on_for_rotations(-50, -50, 1)

    def go_forward_distance(self, distance, speed):
        wheel_size = 17.59 
        rotations = ((distance)/wheel_size)*360
        self.tank.on_for_degrees(speed, speed, rotations)
    
    def drive_to_ball(self, angle, speed):
        self.tank.follow_gyro_angle(
            kp=11.3, ki=0.05, kd=3.2,
            speed=speed,
            target_angle=angle,
            follow_for=self.ev3dev2_motor.follow_for_forever,
        )

    def go_backwards(self):
        self.tank.on(0, -50)

    def go_left(self):
        self.tank.on(-100, 10)

    def go_right(self):
        self.tank.on(100, 5)

    def tank_turn_degrees(self, degrees, speed):
        self.tank.turn_degrees(
            speed=self.ev3dev2_motor.SpeedPercent(speed),
            target_angle=degrees,
            error_margin=3,
        )
    def tank_victory(self):
        # self.ev3dev2_sound.Sound.speak('Hello')
        self.tank.on_for_rotations(50, -50, 10)
            

    def stop_tank(self):
        self.tank.stop()
