#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pigpio
import sys  # exit 대체용

class BLC160MotorTest(Node):
    def __init__(self):
        super().__init__('blc160_motor_test')

        # GPIO 핀 정의
        self.PWM_PIN = 18
        self.DIR_PIN = 23
        self.STOP_PIN = 27
        self.SPEED_PIN = 17   # S 포트 연결 핀

        self.PULSES_PER_REV = 20  # ✅ 사용자 환경에 맞게 수정 필요

        # pigpio 데몬에 연결
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("❌ pigpio 데몬 연결 실패")
            sys.exit(1)

        # 핀 모드 설정
        self.pi.set_mode(self.PWM_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.DIR_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.STOP_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.SPEED_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(self.SPEED_PIN, pigpio.PUD_DOWN)

        # 초기 출력값 설정
        self.pi.write(self.DIR_PIN, 1)  # 정방향
        self.pi.write(self.STOP_PIN, 1)  # STOP 해제
        self.pi.set_PWM_frequency(self.PWM_PIN, 1000)
        self.pi.set_PWM_dutycycle(self.PWM_PIN, 128)  # 50% 속도

        self.get_logger().info("✅ 모터 회전 시작 (정방향, 50% duty)")

        # 회전수 측정용 펄스 카운터
        self.pulse_count = 0

        # 인터럽트 등록: 펄스마다 콜백
        self.cb = self.pi.callback(self.SPEED_PIN, pigpio.RISING_EDGE, self._pulse_callback)

        # 1초마다 상태 출력
        self.timer = self.create_timer(1.0, self.timer_callback)

    def _pulse_callback(self, gpio, level, tick):
        self.pulse_count += 1

    def timer_callback(self):
        rpm = (self.pulse_count / self.PULSES_PER_REV) * 60.0
        freq = self.pi.get_PWM_frequency(self.PWM_PIN)
        duty = self.pi.get_PWM_dutycycle(self.PWM_PIN)

        self.get_logger().info(f"📡 PWM: {freq}Hz | Duty: {duty}/255 | RPM: {rpm:.2f}")
        self.pulse_count = 0

    def destroy_node(self):
        self.get_logger().info("🛑 종료: 모터 정지 및 pigpio 닫기")
        self.pi.set_PWM_dutycycle(self.PWM_PIN, 0)
        self.cb.cancel()
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BLC160MotorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
