#!/usr/bin/python3
import rospy
import RPi.GPIO as GPIO
from math import pi
from numpy import sign
from geometry_msgs.msg import Twist

class MotorController():
    def __init__(self):
        rospy.init_node("motor_control_node")
        self.radps_to_rpm_constant = 9.54929659642538
        self.motor_side = rospy.get_param(f"{rospy.get_name()}/motor_side", "left")
        self.robot_width = rospy.get_param("motor/robot_width",0.2)
        self.wheel_radius = rospy.get_param("motor/wheel_radius",0.025)
        self.gear_ratio = rospy.get_param("motor/gear_ratio",30)
        self.linear_limit = rospy.get_param("motor/linear_limit",1.0)
        self.angular_limit = rospy.get_param("motor/angular_limit",1.5)
        self.max_rpm = rospy.get_param(f"motor/max_rpm", 0)
        self.individual_control = rospy.get_param(f"motor/individual_control", False)
        self.pwm_control = rospy.get_param(f"motor/pwm_control", False)
        self.PWM_max_freq = rospy.get_param(f"motor/{self.motor_side}/PWM_max_freq", 255)
        self.inverted = rospy.get_param(f"motor/{self.motor_side}/inverted", 255)
        self.ENAPIN = rospy.get_param(f"motor/{self.motor_side}/ENAPIN", 0)
        self.ENBPIN = rospy.get_param(f"motor/{self.motor_side}/ENBPIN", 0)
        self.IN1PIN = rospy.get_param(f"motor/{self.motor_side}/IN1PIN", 0)
        self.IN2PIN = rospy.get_param(f"motor/{self.motor_side}/IN2PIN", 0)
        self.IN3PIN = rospy.get_param(f"motor/{self.motor_side}/IN3PIN", 0)
        self.IN4PIN = rospy.get_param(f"motor/{self.motor_side}/IN4PIN", 0)
        self.pwm_control_A = None
        self.pwm_control_B = None
        self.last_x = 0
        self.last_z = 0
        self.set_pinmode()
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)

    def set_pinmode(self):
        GPIO.setmode(GPIO.BOARD)
        if self.pwm_control:
            GPIO.setup(self.ENAPIN, GPIO.OUT, initial=GPIO.LOW)
            self.pwm_control_A = GPIO.PWM(self.ENAPIN, self.PWM_max_freq)
            self.pwm_control_A.start(0)
            if self.individual_control:
                GPIO.setup(self.ENBPIN, GPIO.OUT, initial=GPIO.LOW)
                self.pwm_control_B = GPIO.PWM(self.ENBPIN, self.PWM_max_freq)
                self.pwm_control_B.start(0)

        GPIO.setup(self.IN1PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2PIN, GPIO.OUT, initial=GPIO.LOW)
        if self.individual_control:
            GPIO.setup(self.IN3PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.IN4PIN, GPIO.OUT, initial=GPIO.LOW)

    def set_direction(self, direction):
        if direction == 1:
            GPIO.output(self.IN1PIN, GPIO.HIGH)
            GPIO.output(self.IN2PIN, GPIO.LOW)
            if self.individual_control:
                GPIO.output(self.IN3PIN, GPIO.HIGH)
                GPIO.output(self.IN4PIN, GPIO.LOW)
        elif direction == -1:
            GPIO.output(self.IN1PIN, GPIO.LOW)
            GPIO.output(self.IN2PIN, GPIO.HIGH)
            if self.individual_control:
                GPIO.output(self.IN3PIN, GPIO.LOW)
                GPIO.output(self.IN4PIN, GPIO.HIGH)
        else:
            GPIO.output(self.IN1PIN, GPIO.LOW)
            GPIO.output(self.IN2PIN, GPIO.LOW)
            if self.individual_control:
                GPIO.output(self.IN3PIN, GPIO.LOW)
                GPIO.output(self.IN4PIN, GPIO.LOW)   

    def map(self, value, from_low, from_high, to_low, to_high):
        value = min(max(value, from_low), from_high)
        return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
    
    def set_speed(self, freq):
        self.pwm_control_A.ChangeDutyCycle(freq)
        if self.individual_control:
            self.pwm_control_B.ChangeDutyCycle(freq)

    def write_to_motor(self, rpm):
        self.set_direction(sign(rpm))
        if self.pwm_control:
            pwm = self.map(abs(rpm),0,self.max_rpm,0,255)
            self.set_speed(pwm)

    def cmd_vel_cb(self, msg):
        x = msg.linear.x
        z = msg.angular.z
        if abs(x) >= self.linear_limit:
            x = sign(x) * self.linear_limit
        if abs(z) >= self.angular_limit:
            z = sign(z) * self.angular_limit
        if self.last_x != x or self.last_z != z:
            if self.motor_side == "right":
                radps = (x - (z*self.robot_width/2.0)) / self.wheel_radius
            else:
                radps = (x + (z*self.robot_width/2.0)) / self.wheel_radius
            radps = radps * self.gear_ratio
            rps = radps/(2* pi)
            # rps = -1* rps if self.inverted else rps
            rpm = rps * self.radps_to_rpm_constant
            self.write_to_motor(rpm)
            self.last_x = x
            self.last_z = z

if __name__ == "__main__":
    motor_control = MotorController()
    rospy.spin()    
    GPIO.cleanup()