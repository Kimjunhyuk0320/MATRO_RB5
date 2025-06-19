#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import tf2_ros
import math

# import RPi.GPIO as GPIO  # 실제 하드웨어 사용 시 활성화

class DifferentialMotorWithGPIO(Node):
    def __init__(self):
        super().__init__('differential_motor_gpio_node')

        # 로봇 파라미터
        self.wheel_radius = 0.1075  # 바퀴 반지름 (m)
        self.wheel_separation = 0.4  # 바퀴 간 거리 (m)

        # ⛳ GPIO 핀 정의 (참고용, 실제 GPIO 사용 시만 활성화)
        self.gpio_pins = {
            'A_PWM': 18,
            'A_DIR': 23,
            'A_FEEDBACK': 17,
            'A_STOP': 27,
            'B_PWM': 13,
            'B_DIR': 24,
            'B_FEEDBACK': 22,
            'B_STOP': 26,
        }

        # ✅ GPIO 설정 주석 처리
        # GPIO.setmode(GPIO.BCM)
        # for pin in self.gpio_pins.values():
        #     GPIO.setup(pin, GPIO.OUT)  # 또는 GPIO.IN for feedback
        # self.pwm_A = GPIO.PWM(self.gpio_pins['A_PWM'], 1000)
        # self.pwm_B = GPIO.PWM(self.gpio_pins['B_PWM'], 1000)
        # self.pwm_A.start(0)
        # self.pwm_B.start(0)

        # 위치 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ✅ 고정 mock RPM
        self.mock_left_rpm = 16.0
        self.mock_right_rpm = 16.0

        # 퍼블리셔 및 타이머 설정
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)  # ✅ TF broadcaster 추가
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20Hz

        self.get_logger().info('✅ mock RPM 모드 시작됨 (GPIO 및 엔코더 주석 처리됨)')

    # ✅ 엔코더 기반 회전량 계산 (모터 드라이버 펄스 입력 기반) → 주석 처리
    # def read_encoder(self, dt):
    #     # 실제 펄스 수 기반 회전량 계산
    #     pulse_left = GPIO.input(self.gpio_pins['A_FEEDBACK'])
    #     pulse_right = GPIO.input(self.gpio_pins['B_FEEDBACK'])
    #     # 회전수로 변환 후 반환
    #     return left_rpm, right_rpm

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # mock RPM → 각속도(rad/s)
        omega_l = (self.mock_left_rpm / 60.0) * 2 * math.pi
        omega_r = (self.mock_right_rpm / 60.0) * 2 * math.pi

        # 바퀴 선속도
        v_l = omega_l * self.wheel_radius
        v_r = omega_r * self.wheel_radius

        # 로봇 중심 속도
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.wheel_separation

        # 위치 적분
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # 오도메트리 메시지 생성
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

        # ✅ TF transform broadcast (odom → base_link)
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
        self.get_logger().info('🛑 노드 종료됨 (mock 테스트)')
        # self.pwm_A.stop()
        # self.pwm_B.stop()
        # GPIO.cleanup()
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
