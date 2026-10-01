import sys
import time
import os
import traceback
import threading
import webbrowser

from flask import Flask, Response, jsonify, render_template_string

# El SDK local xarm está junto a este archivo.
PROJECT_ROOT = os.path.abspath(os.path.dirname(__file__))

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from xarm import version
from xarm.wrapper import XArmAPI

os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')

import cv2
import numpy as np


# =========================
# DESACTIVAR WARNINGS OPENCV
# =========================
try:
    cv2.setLogLevel(2)
except Exception:
    try:
        cv2.utils.logging.setLogLevel(
            cv2.utils.logging.LOG_LEVEL_ERROR
        )
    except Exception:
        pass


def _iter_qr_points(points, min_area=10.0):
    if points is None:
        return

    points = np.asarray(points, dtype=np.float32)

    if points.ndim == 2 and points.shape == (4, 2):
        candidates = [points]
    elif points.ndim == 3:
        candidates = points
    else:
        return

    for candidate in candidates:
        candidate = np.asarray(candidate, dtype=np.float32).reshape(-1, 2)

        if candidate.shape != (4, 2):
            continue

        if not np.isfinite(candidate).all():
            continue

        if cv2.contourArea(candidate) <= min_area:
            continue

        yield candidate


def detectar_qr_seguro(qr_detector, gray):
    try:
        retval, points = qr_detector.detectMulti(gray)

        if retval:
            for candidate in _iter_qr_points(points):
                qr_points = candidate.reshape(1, 4, 2)

                try:
                    data, _ = qr_detector.decode(gray, qr_points)
                except cv2.error:
                    continue

                if data:
                    return data.strip(), qr_points

    except (AttributeError, cv2.error):
        pass

    try:
        retval, points = qr_detector.detect(gray)

        if retval:
            for candidate in _iter_qr_points(points):
                qr_points = candidate.reshape(1, 4, 2)

                try:
                    data, _ = qr_detector.decode(gray, qr_points)
                except cv2.error:
                    continue

                if data:
                    return data.strip(), qr_points

    except cv2.error:
        pass

    return "", None


def normalizar_qr(qr_data):
    return ''.join(qr_data.lower().split())


app = Flask(__name__)
estado_clasificacion = None


class EstadoClasificacion:
    """Estado compartido por la cámara, el brazo y la página Flask."""

    def __init__(self):
        self._lock = threading.Lock()
        self._contadores = {
            'caja1': 0,
            'caja2': 0,
            'caja3': 0,
        }
        self._ultima_clasificacion = None
        self._ultima_hora = None
        self._robot_ocupado = False
        self._estado = 'Iniciando cámara'
        self._fotograma_jpeg = None

    def registrar(self, caja):
        with self._lock:
            if self._robot_ocupado:
                return False

            self._contadores[caja] += 1
            self._ultima_clasificacion = caja
            self._ultima_hora = time.strftime('%H:%M:%S')
            self._robot_ocupado = True
            self._estado = f'Clasificando {caja.upper()}'
            return True

    def finalizar_movimiento(self):
        with self._lock:
            self._robot_ocupado = False
            self._estado = 'Listo para clasificar'

    def informar_error_camara(self, mensaje):
        with self._lock:
            self._estado = mensaje

    def marcar_camara_lista(self):
        with self._lock:
            if not self._robot_ocupado:
                self._estado = 'Listo para clasificar'

    def actualizar_fotograma(self, fotograma_jpeg):
        with self._lock:
            self._fotograma_jpeg = fotograma_jpeg

    def obtener_fotograma(self):
        with self._lock:
            return self._fotograma_jpeg

    def resumen(self):
        with self._lock:
            contadores = self._contadores.copy()
            return {
                'total': sum(contadores.values()),
                'cajas': contadores,
                'ultima_clasificacion': self._ultima_clasificacion,
                'ultima_hora': self._ultima_hora,
                'robot_ocupado': self._robot_ocupado,
                'estado': self._estado,
            }


PANEL_HTML = '''
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Clasificación de cajas</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, "Segoe UI", system-ui, sans-serif;
      background: #07101f;
      color: #f4f7fb;
      --panel: rgba(15, 27, 48, .86);
      --line: rgba(148, 163, 184, .16);
      --muted: #8fa3bf;
      --cyan: #22d3ee;
      --blue: #3b82f6;
      --violet: #8b5cf6;
      --amber: #f59e0b;
      --green: #34d399;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      padding: 28px;
      background:
        radial-gradient(circle at 10% 0%, rgba(37, 99, 235, .22), transparent 35%),
        radial-gradient(circle at 100% 100%, rgba(139, 92, 246, .15), transparent 36%),
        #07101f;
    }
    main { width: min(1380px, 100%); margin: auto; }
    header { display: flex; justify-content: space-between; gap: 24px; align-items: end; margin-bottom: 24px; }
    .eyebrow { margin: 0 0 7px; color: var(--cyan); font-size: .72rem; font-weight: 800; letter-spacing: .19em; text-transform: uppercase; }
    h1 { margin: 0; font-size: clamp(1.75rem, 3vw, 2.7rem); letter-spacing: -.045em; }
    .subtitulo { margin: 7px 0 0; color: var(--muted); }
    .cabecera-derecha { display: flex; align-items: center; gap: 12px; }
    .reloj { color: var(--muted); font-variant-numeric: tabular-nums; }
    .estado { display: inline-flex; align-items: center; gap: 9px; padding: 10px 14px; border: 1px solid var(--line); border-radius: 999px; background: rgba(15, 27, 48, .8); color: #d7e3f4; white-space: nowrap; box-shadow: 0 10px 35px rgba(0, 0, 0, .18); }
    .estado::before { content: ""; width: 9px; height: 9px; border-radius: 50%; background: var(--green); box-shadow: 0 0 0 5px rgba(52, 211, 153, .12); }
    .estado.ocupado::before { background: var(--amber); box-shadow: 0 0 0 5px rgba(245, 158, 11, .12); animation: pulso 1.2s infinite; }
    .contenido { display: grid; grid-template-columns: minmax(0, 1.7fr) minmax(330px, .85fr); gap: 22px; }
    .superficie { border: 1px solid var(--line); border-radius: 22px; background: var(--panel); box-shadow: 0 24px 70px rgba(0, 0, 0, .28); backdrop-filter: blur(14px); }
    .camara { overflow: hidden; }
    .barra-camara { display: flex; align-items: center; justify-content: space-between; padding: 15px 18px; border-bottom: 1px solid var(--line); }
    .titulo-camara { display: flex; align-items: center; gap: 10px; margin: 0; font-size: .92rem; color: #dbe7f7; }
    .indicador-live { display: inline-flex; align-items: center; gap: 7px; color: #fca5a5; font-size: .7rem; font-weight: 800; letter-spacing: .11em; }
    .indicador-live::before { content: ""; width: 7px; height: 7px; border-radius: 50%; background: #ef4444; box-shadow: 0 0 12px #ef4444; }
    .camara-marco { position: relative; background: #020610; }
    .camara img { display: block; width: 100%; aspect-ratio: 4 / 3; object-fit: contain; }
    .camara-etiqueta { position: absolute; left: 16px; bottom: 16px; padding: 8px 11px; border: 1px solid rgba(255,255,255,.15); border-radius: 9px; background: rgba(2, 6, 16, .7); color: #cbd9ed; font-size: .75rem; backdrop-filter: blur(7px); }
    .panel { display: flex; flex-direction: column; gap: 16px; }
    .total { position: relative; overflow: hidden; padding: 26px; border-radius: 22px; background: linear-gradient(135deg, #2563eb 0%, #6d28d9 100%); box-shadow: 0 22px 50px rgba(37, 99, 235, .22); }
    .total::after { content: ""; position: absolute; width: 170px; height: 170px; right: -60px; top: -75px; border: 28px solid rgba(255,255,255,.09); border-radius: 50%; }
    .total span { color: rgba(255,255,255,.78); font-size: .82rem; font-weight: 700; letter-spacing: .08em; text-transform: uppercase; }
    .total strong { position: relative; display: block; margin-top: 8px; font-size: clamp(4rem, 8vw, 6rem); line-height: .92; letter-spacing: -.07em; }
    .cards { display: grid; grid-template-columns: repeat(3, 1fr); gap: 11px; }
    .card { position: relative; overflow: hidden; min-width: 0; padding: 17px 12px 15px; text-align: center; }
    .card::before { content: ""; position: absolute; inset: 0 0 auto; height: 3px; background: var(--card-color); }
    .card:nth-child(1) { --card-color: var(--cyan); }
    .card:nth-child(2) { --card-color: var(--violet); }
    .card:nth-child(3) { --card-color: var(--amber); }
    .card span { display: block; color: var(--muted); font-size: .78rem; }
    .card strong { display: block; margin-top: 6px; font-size: 2.2rem; letter-spacing: -.05em; }
    .detalle { padding: 20px; }
    .detalle-titulo { display: flex; justify-content: space-between; align-items: center; margin-bottom: 16px; }
    .detalle h2 { margin: 0; font-size: .88rem; }
    .detalle-grid { display: grid; gap: 13px; }
    .dato { display: flex; justify-content: space-between; gap: 15px; padding-bottom: 13px; border-bottom: 1px solid var(--line); }
    .dato:last-child { padding: 0; border: 0; }
    .dato span { color: var(--muted); font-size: .82rem; }
    .dato strong { max-width: 60%; text-align: right; font-size: .82rem; font-weight: 650; }
    .pie { display: flex; justify-content: space-between; gap: 15px; margin-top: 18px; color: #6f839f; font-size: .72rem; }
    @keyframes pulso { 50% { opacity: .45; } }
    @media (max-width: 920px) { .contenido { grid-template-columns: 1fr; } .panel { display: grid; grid-template-columns: 1fr 1fr; } .cards, .detalle { grid-column: 1 / -1; } }
    @media (max-width: 620px) { body { padding: 16px; } header { align-items: flex-start; flex-direction: column; } .cabecera-derecha { width: 100%; justify-content: space-between; } .panel { display: flex; } .pie { flex-direction: column; } }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <p class="eyebrow">UFactory · Control Center</p>
        <h1>Clasificación inteligente</h1>
        <p class="subtitulo">Monitoreo en tiempo real del brazo y la estación QR</p>
      </div>
      <div class="cabecera-derecha">
        <span id="reloj" class="reloj">--:--:--</span>
        <span id="estado" class="estado">Conectando…</span>
      </div>
    </header>
    <section class="contenido">
      <section class="camara superficie">
        <div class="barra-camara">
          <h2 class="titulo-camara">Cámara principal · Dispositivo 0</h2>
          <span class="indicador-live">EN VIVO</span>
        </div>
        <div class="camara-marco">
          <img src="{{ url_for('video_feed') }}" alt="Vídeo en directo de la cámara">
          <span class="camara-etiqueta">Detección QR activa</span>
        </div>
      </section>
      <aside class="panel">
        <section class="total"><span>Total clasificado</span><strong id="total">0</strong></section>
        <section class="cards" aria-label="Contadores por caja">
          <article class="card superficie"><span>Caja 01</span><strong id="caja1">0</strong></article>
          <article class="card superficie"><span>Caja 02</span><strong id="caja2">0</strong></article>
          <article class="card superficie"><span>Caja 03</span><strong id="caja3">0</strong></article>
        </section>
        <section class="detalle superficie">
          <div class="detalle-titulo"><h2>Estado de la estación</h2></div>
          <div class="detalle-grid">
            <div class="dato"><span>Última clasificación</span><strong id="ultima">Sin registros</strong></div>
            <div class="dato"><span>Modo de trabajo</span><strong>Automático</strong></div>
            <div class="dato"><span>Controlador</span><strong>192.168.1.172</strong></div>
          </div>
        </section>
      </aside>
    </section>
    <footer class="pie"><span>Sistema de clasificación xArm · Cámara y control integrados</span><span>Detener servidor: Ctrl+C</span></footer>
  </main>
  <script>
    async function actualizar() {
      try {
        const respuesta = await fetch('/api/estado', { cache: 'no-store' });
        if (!respuesta.ok) throw new Error('Respuesta no válida');
        const datos = await respuesta.json();
        document.getElementById('total').textContent = datos.total;
        for (const caja of ['caja1', 'caja2', 'caja3']) document.getElementById(caja).textContent = datos.cajas[caja];
        const estado = document.getElementById('estado');
        estado.textContent = datos.estado;
        estado.classList.toggle('ocupado', datos.robot_ocupado);
        document.getElementById('ultima').textContent = datos.ultima_clasificacion
          ? `${datos.ultima_clasificacion.toUpperCase()} · ${datos.ultima_hora}`
          : 'Sin registros';
      } catch (error) {
        document.getElementById('estado').textContent = 'Sin conexión con el programa';
      }
    }
    function actualizarReloj() {
      document.getElementById('reloj').textContent = new Date().toLocaleTimeString('es-CL', { hour12: false });
    }
    actualizar();
    actualizarReloj();
    setInterval(actualizar, 1000);
    setInterval(actualizarReloj, 1000);
  </script>
</body>
</html>
'''


def transmitir_video(estado):
    while True:
        fotograma = estado.obtener_fotograma()
        if fotograma is None:
            time.sleep(0.05)
            continue

        yield (
            b'--fotograma\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + fotograma + b'\r\n'
        )
        time.sleep(0.03)


@app.route('/')
def inicio():
    return render_template_string(PANEL_HTML)


@app.route('/video_feed')
def video_feed():
    return Response(
        transmitir_video(estado_clasificacion),
        mimetype='multipart/x-mixed-replace; boundary=fotograma'
    )


@app.route('/api/estado')
def api_estado():
    return jsonify(estado_clasificacion.resumen())


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

    # =========================
    # INICIALIZAR ROBOT
    # =========================
    def _robot_init(self):

        self._arm.clean_warn()
        self._arm.clean_error()

        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)

        time.sleep(1)

        self._arm.register_error_warn_changed_callback(
            self._error_warn_changed_callback
        )

        self._arm.register_state_changed_callback(
            self._state_changed_callback
        )

        if hasattr(
            self._arm,
            'register_count_changed_callback'
        ):

            self._arm.register_count_changed_callback(
                self._count_changed_callback
            )

    # =========================
    # CALLBACKS
    # =========================
    def _error_warn_changed_callback(self, data):

        if data and data['error_code'] != 0:

            self.alive = False

            self.pprint(
                f"err={data['error_code']}, quit"
            )

            self._arm.release_error_warn_changed_callback(
                self._error_warn_changed_callback
            )

    def _state_changed_callback(self, data):

        if data and data['state'] == 4:

            self.alive = False

            self.pprint('state=4, quit')

            self._arm.release_state_changed_callback(
                self._state_changed_callback
            )

    def _count_changed_callback(self, data):

        if self.is_alive:

            self.pprint(
                f"counter val: {data['count']}"
            )

    # =========================
    # VERIFICAR ESTADO ROBOT
    # =========================
    def _check_code(self, code, label):

        if not self.is_alive or code != 0:

            self.alive = False

            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()

            self.pprint(
                '{}, code={}, connected={}, state={}, '
                'error={}, ret1={}, ret2={}'.format(
                    label,
                    code,
                    self._arm.connected,
                    self._arm.state,
                    self._arm.error_code,
                    ret1,
                    ret2
                )
            )

        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):

        try:

            stack_tuple = traceback.extract_stack(
                limit=2
            )[0]

            print(
                '[{}][{}] {}'.format(
                    time.strftime(
                        '%Y-%m-%d %H:%M:%S',
                        time.localtime(time.time())
                    ),
                    stack_tuple[1],
                    ' '.join(map(str, args))
                )
            )

        except Exception:

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

        if (
            self.alive and
            self._arm.connected and
            self._arm.error_code == 0
        ):

            if self._arm.state == 5:

                cnt = 0

                while (
                    self._arm.state == 5 and
                    cnt < 5
                ):
                    cnt += 1
                    time.sleep(0.1)

            return self._arm.state < 4

        return False
    # =========================
    # MOVIMIENTO ROBOT
    # =========================
    def run(self, x, y, z, roll, pitch, yaw):

        try:

            self.alive = True
            self._arm.motion_enable(True)
            self._arm.set_mode(0)
            self._arm.set_state(0)
            time.sleep(0.2)

            self._angle_speed = 30
            self._angle_acc = 200

            # =========================
            # POSICION INICIAL
            # =========================
            code = self._arm.set_position(
                *[200.0, 0.0, 200.0, 180.0, 0.0, 90.0],
                speed=self._tcp_speed,
                mvacc=self._tcp_acc,
                radius=0.0,
                wait=True
            )

            if not self._check_code(
                code,
                'set_position'
            ):
                return

            # =========================
            # ABRIR GRIPPER
            # =========================
            self._arm.open_lite6_gripper()
            time.sleep(1)

            # =========================
            # BAJAR A TOMAR PIEZA
            # =========================
            code = self._arm.set_position(
                *[200.0, 0.0, 90.0, 180.0, 0.0, 90.0],
                speed=self._tcp_speed,
                mvacc=self._tcp_acc,
                radius=0.0,
                wait=True
            )

            if not self._check_code(
                code,
                'set_position'
            ):
                return

            # =========================
            # CERRAR GRIPPER
            # =========================
            self._arm.close_lite6_gripper()
            time.sleep(1)

            # =========================
            # SUBIR
            # =========================
            code = self._arm.set_position(
                *[200.0, 0.0, 200.0, 180.0, 0.0, 90.0],
                speed=self._tcp_speed,
                mvacc=self._tcp_acc,
                radius=0.0,
                wait=True
            )

            if not self._check_code(
                code,
                'set_position'
            ):
                return

            # =========================
            # POSICION INTERMEDIA
            # =========================
            code = self._arm.set_position(
                *[350.0, 110.0, 200.0, 180.0, 0.0, 90.0],
                speed=self._tcp_speed,
                mvacc=self._tcp_acc,
                radius=0.0,
                wait=True
            )

            if not self._check_code(
                code,
                'set_position'
            ):
                return

            # =========================
            # IR A DESTINO
            # =========================
            code = self._arm.set_position(
                *[x, y, z, roll, pitch, yaw],
                speed=self._tcp_speed,
                mvacc=self._tcp_acc,
                radius=0.0,
                wait=True
            )

            if not self._check_code(
                code,
                'set_position'
            ):
                return

            # =========================
            # SOLTAR PIEZA
            # =========================
            self._arm.open_lite6_gripper()
            time.sleep(1)

            # =========================
            # VOLVER ARRIBA
            # =========================
            code = self._arm.set_position(
                *[200.0, 0.0, 200.0, 180.0, 0.0, 90.0],
                speed=self._tcp_speed,
                mvacc=self._tcp_acc,
                radius=0.0,
                wait=True
            )

            if not self._check_code(
                code,
                'set_position'
            ):
                return

            # =========================
            # DESACTIVAR EFECTOR LITE6
            # =========================
            # Detiene solamente el motor del gripper; no detiene el brazo.
            code = self._arm.stop_lite6_gripper()

            if not self._check_code(
                code,
                'stop_lite6_gripper'
            ):
                return

        except Exception as e:

            self.pprint(
                f'MainException: {e}'
            )

        self.pprint('Rutina terminada')

DESTINOS = {
    'caja1': {
        'x': 340.0, 'y': 190.0, 'z': 125.0,
        'roll': 180.0, 'pitch': 0.0, 'yaw': 90.0,
    },
    'caja2': {
        'x': 340.0, 'y': 126.0, 'z': 125.0,
        'roll': 180.0, 'pitch': 0.0, 'yaw': 90.0,
    },
    'caja3': {
        'x': 340.0, 'y': 66.6, 'z': 125.0,
        'roll': 180.0, 'pitch': 0.0, 'yaw': 90.0,
    },
}


def ejecutar_clasificacion(robot_main, estado, caja):
    """Ejecuta una sola rutina; nunca permite dos movimientos simultáneos."""
    try:
        robot_main.run(**DESTINOS[caja])
    finally:
        estado.finalizar_movimiento()


def procesar_camara(robot_main, estado, detener):
    """Lee la cámara, detecta QR y publica imágenes JPEG para Flask."""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        estado.informar_error_camara('No se pudo abrir la cámara 0')
        return

    estado.marcar_camara_lista()
    qr_detector = cv2.QRCodeDetector()
    ultimo_qr = ''
    ultimo_tiempo = 0

    try:
        while not detener.is_set():
            ret, frame = cap.read()
            if not ret or frame is None:
                estado.informar_error_camara('No se recibe imagen de la cámara')
                time.sleep(0.1)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            qr_data, pts = detectar_qr_seguro(qr_detector, gray)
            qr_data = normalizar_qr(qr_data) if qr_data else ''

            if (
                qr_data
                and (
                    qr_data != ultimo_qr
                    or time.time() - ultimo_tiempo > 5
                )
            ):
                ultimo_qr = qr_data
                ultimo_tiempo = time.time()
                print('QR detectado:', qr_data)

                if qr_data in DESTINOS:
                    if estado.registrar(qr_data):
                        threading.Thread(
                            target=ejecutar_clasificacion,
                            args=(robot_main, estado, qr_data),
                            daemon=True,
                        ).start()
                    else:
                        print('Brazo ocupado; QR ignorado:', qr_data)
                else:
                    print('QR no reconocido:', qr_data)

            if pts is not None:
                pts = pts.astype(int)
                for i in range(len(pts[0])):
                    pt1 = tuple(pts[0][i])
                    pt2 = tuple(pts[0][(i + 1) % len(pts[0])])
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            correcto, buffer = cv2.imencode('.jpg', frame)
            if correcto:
                estado.actualizar_fotograma(buffer.tobytes())
    finally:
        cap.release()


# =========================================================
# MAIN
# =========================================================
if __name__ == '__main__':
    print(f'xArm-Python-SDK Version: {version.__version__}')

    arm = XArmAPI('192.168.1.172', baud_checkset=False)
    robot_main = RobotMain(arm)
    estado_clasificacion = EstadoClasificacion()
    detener = threading.Event()

    hilo_camara = threading.Thread(
        target=procesar_camara,
        args=(robot_main, estado_clasificacion, detener),
        daemon=True,
    )
    hilo_camara.start()

    url_panel = 'http://127.0.0.1:5000'
    print(f'Vista de cámara y clasificación: {url_panel}')
    threading.Timer(0.8, webbrowser.open, args=(url_panel,)).start()

    try:
        app.run(
            host='127.0.0.1',
            port=5000,
            debug=False,
            threaded=True,
            use_reloader=False,
        )
    finally:
        detener.set()
        hilo_camara.join(timeout=2)
        arm.disconnect()
