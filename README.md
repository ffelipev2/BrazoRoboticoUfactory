# Brazo robótico UFactory

Proyecto mínimo para controlar el brazo UFactory/Lite6 mediante códigos QR y una cámara USB.

## Contenido

- `brazoCamara2.0.py`: aplicación única con cámara, lectura QR, control del brazo y panel web.
- `xarm/`: SDK local requerido para comunicarse con el controlador del brazo.
- `requirements.txt`: dependencias de Python necesarias.

## Instalación

Desde esta carpeta, con Python 3:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Antes de ejecutar, revise la IP del brazo en `brazoCamara2.0.py` (actualmente `192.168.1.172`) y confirme que la cámara esté disponible como índice `0`.

```powershell
python .\brazoCamara2.0.py
```

Al ejecutarlo se abre una sola vista en el navegador con la cámara, los contadores, la última clasificación y el estado de la estación. También puede abrirla manualmente en `http://127.0.0.1:5000`.

Después de depositar cada caja y regresar a la posición inicial, el programa detiene únicamente el motor del efector Lite6 mediante `stop_lite6_gripper()`. Los servos de las articulaciones del brazo permanecen habilitados.

Para detener el programa, vuelva a la terminal y pulse `Ctrl+C`.
