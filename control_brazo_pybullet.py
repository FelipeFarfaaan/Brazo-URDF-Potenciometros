"""
================================================================
 CONTROL DEL BRAZO EN PYBULLET - control_brazo_pybullet.py
================================================================
 Recibe los valores de 3 potenciometros desde la ESP32 por UART
 y controla el brazo (brazo.urdf) en tiempo real.

 Si la ESP32 no esta conectada, se activa un MODO DEMO con
 sliders virtuales dentro de la propia ventana de PyBullet,
 util para probar el brazo antes de tener el hardware listo.

 Requisitos:
   pip install pybullet pyserial

 IMPORTANTE:
   - Este script debe estar en la MISMA carpeta que "brazo.urdf"
   - Cierra Thonny (u otro programa que use el puerto serie)
     antes de ejecutar este script
   - Cambia PUERTO_SERIE segun tu sistema operativo:
       Windows -> "COM4", "COM5", etc. (ver Administrador de dispositivos)
       Linux   -> "/dev/ttyUSB0"
       Mac     -> "/dev/cu.usbserial-XXXX"
================================================================
"""

import time

import pybullet as p
import pybullet_data
import serial

# ---------- CONFIGURACION ----------
PUERTO_SERIE = "COM3"      # <-- puerto confirmado en Thonny
BAUDRATE = 115200

# Limites tomados directamente de brazo.urdf
LIM_J1 = (-2.5, 2.5)        # joint_1  (revolute, base)
LIM_J2 = (-2.0, 2.0)        # joint_2  (revolute, hombro)
LIM_GRIPPER = (0.0, 0.15)   # joint_gripper (prismatic)
LIM_DEDO = (0.0, 0.05)      # joint_dedo_izq / joint_dedo_der (prismatic)

ADC_MIN, ADC_MAX = 0, 4095

FUERZA = {
    "joint_1": 100,
    "joint_2": 80,
    "joint_gripper": 30,
    "joint_dedo_izq": 20,
    "joint_dedo_der": 20,
}


def mapear(valor, in_min, in_max, out_min, out_max):
    """Convierte un valor de un rango de entrada a un rango de salida (con recorte)."""
    valor = max(in_min, min(in_max, valor))
    return out_min + (valor - in_min) * (out_max - out_min) / (in_max - in_min)


# ---------- CONEXION SERIE (con respaldo a modo demo) ----------
esp32_conectada = False
ser = None
try:
    print(f"Conectando a la ESP32 en {PUERTO_SERIE}...")
    ser = serial.Serial(PUERTO_SERIE, BAUDRATE, timeout=0.05)
    time.sleep(2)  # tiempo para que la ESP32 reinicie tras abrir el puerto
    esp32_conectada = True
    print("ESP32 conectada correctamente.")
except Exception as e:
    print(f"No se pudo conectar a la ESP32 ({e}).")
    print("Se activa el MODO DEMO con sliders virtuales en PyBullet.")

# ---------- PYBULLET ----------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

robot_id = p.loadURDF("brazo.urdf", [0, 0, 0.15], useFixedBase=True)

# Mapear nombre de cada joint a su indice (evita depender del orden)
joint_index = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    nombre = info[1].decode("utf-8")
    joint_index[nombre] = i
    print(f"Joint {i}: {nombre}")

# Sliders virtuales para el modo demo (en las mismas unidades del URDF)
if not esp32_conectada:
    slider_j1 = p.addUserDebugParameter("joint_1 (base)", *LIM_J1, 0)
    slider_j2 = p.addUserDebugParameter("joint_2 (hombro)", *LIM_J2, 0)
    slider_g = p.addUserDebugParameter("pinza (abrir/cerrar)", *LIM_GRIPPER, 0)


def mover_joint(nombre, posicion):
    p.setJointMotorControl2(
        robot_id, joint_index[nombre],
        controlMode=p.POSITION_CONTROL,
        targetPosition=posicion,
        force=FUERZA[nombre],
        positionGain=0.3, velocityGain=1.0,
    )


def aplicar_pose(q1, q2, gripper):
    dedo = mapear(gripper, *LIM_GRIPPER, *LIM_DEDO)
    mover_joint("joint_1", q1)
    mover_joint("joint_2", q2)
    mover_joint("joint_gripper", gripper)
    mover_joint("joint_dedo_izq", dedo)
    mover_joint("joint_dedo_der", dedo)


print("\nSimulacion en marcha. Cierra la ventana de PyBullet para salir.\n")

try:
    while True:
        if esp32_conectada:
            try:
                if ser.in_waiting > 0:
                    linea = ser.readline().decode("utf-8", errors="ignore").strip()
                    if linea.startswith("DATA,"):
                        _, v1, v2, v3 = linea.split(",")
                        v1, v2, v3 = int(v1), int(v2), int(v3)

                        q1 = mapear(v1, ADC_MIN, ADC_MAX, *LIM_J1)
                        q2 = mapear(v2, ADC_MIN, ADC_MAX, *LIM_J2)
                        gripper = mapear(v3, ADC_MIN, ADC_MAX, *LIM_GRIPPER)

                        aplicar_pose(q1, q2, gripper)
            except Exception as e:
                print(f"Error leyendo el puerto serie: {e}")
        else:
            q1 = p.readUserDebugParameter(slider_j1)
            q2 = p.readUserDebugParameter(slider_j2)
            gripper = p.readUserDebugParameter(slider_g)
            aplicar_pose(q1, q2, gripper)

        p.stepSimulation()
        time.sleep(1 / 240)

except KeyboardInterrupt:
    print("\nSimulacion detenida por el usuario.")
finally:
    if ser:
        ser.close()
    p.disconnect()
    print("Conexiones cerradas.")
