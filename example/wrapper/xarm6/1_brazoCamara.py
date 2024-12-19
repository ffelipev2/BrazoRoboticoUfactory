
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

tiempo = 0.5

class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
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
    def run(self):
        try:
            # Joint Motion
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_position(*[200.0, 0.0, 200.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(tiempo)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            code = self._arm.set_position(*[200.0, 0.0, 97.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(tiempo)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            code = self._arm.set_pause_time(tiempo)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[200.0, 0.0, 200.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(tiempo)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[350.0, 110.0, 200.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[350.0, 180.0, 145.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


import cv2
import time

def leerCamara():
    cap = cv2.VideoCapture(0)  # Iniciar la cámara web (cambia el índice si tienes varias cámaras)

    # Inicializar el detector de QR
    qr_detector = cv2.QRCodeDetector()

    while True:
        ret, frame = cap.read()  # Capturar un fotograma de la cámara
        if not ret:
            break

        # Detectar y decodificar el código QR
        qr_data, pts, qr_code = qr_detector.detectAndDecode(frame)  # Usar detectAndDecode

        if qr_data:
            qr_type = "QR"  # Los códigos leídos por esta función son siempre QR
            time.sleep(4)
            print(f'Tipo de QR: {qr_type}')
            print(f'Datos del QR: {qr_data}')

            if qr_data == "caja1":
                robot_main = RobotMain(arm)
                robot_main.run()
                break
            if qr_data == "caja2":
                robot_main = RobotMain(arm)
                robot_main.run()
                break

        cv2.imshow('QR Reader', frame)  # Mostrar el fotograma con el cuadro del QR resaltado

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()  # Liberar la cámara
    cv2.destroyAllWindows()  # Cerrar todas las ventanas


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.172', baud_checkset=False)
    leerCamara()
    #robot_main = RobotMain(arm)
    #robot_main.run()