#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import tf2_ros
import math
import RPi.GPIO as GPIO
import threading

class DifferentialMotorWithGPIO(Node):
    def __init__(self):
        super().__init__('differential_motor_gpio_node')

        # 로봇 파라미터
        self.wheel_radius = 0.1075  # 바퀴 반지름 (m)
        self.wheel_separation = 0.4  # 바퀴 간 거리 (m)
        self.ppr = 60  # S포트 펄스 당 회전수 계산용 (예: 60PPR)

        # GPIO 핀 정의
        self.gpio_pins = {
            'A_PWM': 18,
            'A_DIR': 23,
            'A_FEEDBACK': 17,  # S 포트 A
            'A_STOP': 27,
            'B_PWM': 13,
            'B_DIR': 24,
            'B_FEEDBACK': 22,  # S 포트 B
            'B_STOP': 26,
        }

        # GPIO 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pins['A_PWM'], GPIO.OUT)
        GPIO.setup(self.gpio_pins['B_PWM'], GPIO.OUT)
        GPIO.setup(self.gpio_pins['A_DIR'], GPIO.OUT)
        GPIO.setup(self.gpio_pins['B_DIR'], GPIO.OUT)
        GPIO.setup(self.gpio_pins['A_FEEDBACK'], GPIO.IN)
        GPIO.setup(self.gpio_pins['B_FEEDBACK'], GPIO.IN)

        self.pwm_A = GPIO.PWM(self.gpio_pins['A_PWM'], 1000)
        self.pwm_B = GPIO.PWM(self.gpio_pins['B_PWM'], 1000)
        self.pwm_A.start(0)
        self.pwm_B.start(0)

        # 위치 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.left_pulse_count = 0
        self.right_pulse_count = 0
        self.last_left_rpm = 0.0
        self.last_right_rpm = 0.0

        # 인터럽트 기반 펄스 카운터 등록
        GPIO.add_event_detect(self.gpio_pins['A_FEEDBACK'], GPIO.RISING, callback=self.left_pulse_callback)
        GPIO.add_event_detect(self.gpio_pins['B_FEEDBACK'], GPIO.RISING, callback=self.right_pulse_callback)

        # 퍼블리셔 및 타이머 설정
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20Hz

        self.get_logger().info('✅ GPIO 및 S포트 피드백 기반 제어 시작됨')

    def left_pulse_callback(self, channel):
        self.left_pulse_count += 1

    def right_pulse_callback(self, channel):
        self.right_pulse_count += 1

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_l = v - w * self.wheel_separation / 2.0
        v_r = v + w * self.wheel_separation / 2.0

        left_rpm = (v_l / (2 * math.pi * self.wheel_radius)) * 60.0
        right_rpm = (v_r / (2 * math.pi * self.wheel_radius)) * 60.0

        self.set_motor(self.gpio_pins['A_PWM'], self.gpio_pins['A_DIR'], left_rpm)
        self.set_motor(self.gpio_pins['B_PWM'], self.gpio_pins['B_DIR'], right_rpm)

    def set_motor(self, pwm_pin, dir_pin, rpm):
        direction = GPIO.HIGH if rpm >= 0 else GPIO.LOW
        GPIO.output(dir_pin, direction)
        duty = min(abs(rpm) * 0.5, 100)
        pwm = self.pwm_A if pwm_pin == self.gpio_pins['A_PWM'] else self.pwm_B
        pwm.ChangeDutyCycle(duty)

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # S 포트 펄스 → 회전수 계산
        left_revolutions = self.left_pulse_count / self.ppr
        right_revolutions = self.right_pulse_count / self.ppr

        # 회전수 → 각속도 (rad/s)
        omega_l = (left_revolutions / dt) * 2 * math.pi
        omega_r = (right_revolutions / dt) * 2 * math.pi

        # 펄스 카운터 리셋
        self.left_pulse_count = 0
        self.right_pulse_count = 0

        v_l = omega_l * self.wheel_radius
        v_r = omega_r * self.wheel_radius

        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.wheel_separation

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        q = quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.get_logger().info('🛑 노드 종료됨')
        self.pwm_A.stop()
        self.pwm_B.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialMotorWithGPIO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
