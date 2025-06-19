from setuptools import setup, find_packages  # ⛳️ find_packages도 함께 import
import os
from glob import glob  # ✅ launch 파일 수집용

package_name = 'MATRO_RB5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # ✅ 하위 Python 모듈 포함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # ✅ 경로 통일
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junhyuk',
    maintainer_email='kimminuk0320@gmail.com',
    description='병상 자율이동 로봇 모듈 패키지',
    license='MIT',  # 또는 적절한 라이선스로 수정
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = MATRO_RB5.hello_node:main',
            'motor_controller_sample = MATRO_RB5.motor_controller_sample:main',
            'motor_controller = MATRO_RB5.motor_controller:main',
            'motor_test = MATRO_RB5.motor_test:main'
        ],
    },
)
