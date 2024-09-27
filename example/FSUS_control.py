# from robot import Robot
import sys
sys.path.append("../src")
sys.path.append("..")
# from dynamixel import Dynamixel
import numpy as np
import time
import mujoco.viewer
from simulation.interface import SimulatedRobot
import threading

# 导入依赖
import serial
from uservo import UartServoManager

SERVO_PORT_NAME =  '/dev/ttyUSB0' # 舵机串口号
SERVO_BAUDRATE = 115200 # 舵机的波特率
SERVO_ID = 0  # 舵机的ID号
# 初始化串口
uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,\
					 parity=serial.PARITY_NONE, stopbits=1,\
					 bytesize=8,timeout=0)
# 初始化舵机管理器
uservo = UartServoManager(uart)

# 设置舵机为阻尼模式
uservo.set_damping(SERVO_ID, 200)


def read_leader_position():
    global target_pos
    while True:
        target_pos[0] = (-(uservo.query_servo_angle(0))*3.14/180)
        target_pos[1] = uservo.query_servo_angle(1)*3.14/180
        target_pos[2] = (-(uservo.query_servo_angle(3)-90)*3.14/180)
        target_pos[4] = (-(uservo.query_servo_angle(4))*3.14/180)


# leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/tty.usbmodem57380045631').instantiate()
# leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 6, 7])
# leader.set_trigger_torque()

m = mujoco.MjModel.from_xml_path('../simulation/low_cost_robot/scene.xml')
d = mujoco.MjData(m)

r = SimulatedRobot(m, d)

target_pos = np.zeros(5)

# Start the thread for reading leader position
leader_thread = threading.Thread(target=read_leader_position)
leader_thread.daemon = True
leader_thread.start()

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():
        # Use the latest target_pos
        step_start = time.time()
        target_pos_local = target_pos.copy()
        # print(f'target pos copy {time.time() - step_start}')
        r.set_target_pos(target_pos_local)
        # print(f'set targtee pos copy {time.time() - step_start}')
        mujoco.mj_step(m, d)
        # print(f'mjstep {time.time() - step_start}')
        viewer.sync()
        # print(f'viewer sync {time.time() - step_start}')

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        # print(f'time until next step {time_until_next_step}')
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
