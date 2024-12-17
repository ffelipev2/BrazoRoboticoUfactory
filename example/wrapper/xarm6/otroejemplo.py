import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI
import cv2
from pyzbar.pyzbar import decode


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 200
        self._tcp_acc = 2000
        self._angle_speed = 30
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    # Robot Main Run
    def run(self,x,y,z,roll,pitch,yall):
        try:
            # Joint Motion
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_position(*[200.0, 0.0, 200.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1/4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            code = self._arm.set_position(*[200.0, 0.0, 90.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1/4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            code = self._arm.set_pause_time(1/4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[200.0, 0.0, 200.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1/4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[350.0, 110.0, 200.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1/4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[x,y,z,roll,pitch,yall], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            code = self._arm.set_pause_time(1/4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[200.0, 0.0, 200.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


if __name__ == '__main__':
    print('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.172', baud_checkset=False)
    cap = cv2.VideoCapture(0)  # Abre la cámara
    while True:
        ret, frame = cap.read()  # Captura un fotograma de la cámara
        if not ret:
            continue
        decoded_objects = decode(frame)  # Decodifica los códigos QR en el fotograma
        for obj in decoded_objects:
            data = obj.data.decode('utf-8')  # Decodifica los datos del QR
            print("Datos del código QR:", data)
            if data == "caja1":
                robot_main = RobotMain(arm)
                robot_main.run(x=340.0, y=190.0, z=125.0, roll=180.0, pitch=0.0, yall=90.0)
                time.sleep(5)
            elif data == "caja2":
                robot_main = RobotMain(arm)
                robot_main.run(x=340.0, y=126.0, z=125.0, roll=180.0, pitch=0.0, yall=90.0)
                time.sleep(5)
            elif data == "caja 3":
                robot_main = RobotMain(arm)
                robot_main.run(x=340.0, y=66.6, z=125.0, roll=180.0, pitch=0.0, yall=90.0)
                time.sleep(5)
        cv2.imshow("Lector de QR", frame)  # Muestra el fotograma con los códigos QR destacados
        if cv2.waitKey(1) == 27:  # Presiona Esc para salir
            break
    cap.release()
    cv2.destroyAllWindows()
