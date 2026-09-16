# Control del Brazo Robótico con Potenciómetros (ESP32 + PyBullet)

Actividad del curso de Microcontroladores (UMNG) — control en tiempo real del brazo
definido en `brazo.urdf` usando 3 potenciómetros conectados a un ESP32.

## Funcionamiento

Cada potenciómetro entrega una señal analógica de 0 a 3.3V que el ESP32 lee por
uno de sus canales ADC (resolución 12 bits, 0–4095). El ESP32 corre MicroPython
y envía continuamente los 3 valores por el mismo cable USB (formato de texto
`DATA,v1,v2,v3`). En la PC, un script en Python (`pyserial` + `pybullet`) lee esas
líneas, las convierte (mapea) a los rangos reales de cada articulación definidos
en el URDF, y se las aplica al robot simulado con `setJointMotorControl2()`.

```
3 potenciómetros → ESP32 (ADC, MicroPython) → UART/USB 115200 baudios
    → PC: script Python (pyserial) → mapeo de valores → PyBullet (brazo.urdf)
```

| Potenciómetro | Articulación controlada | Tipo      | Rango                  |
|---------------|--------------------------|-----------|-------------------------|
| POT 1 (GPIO34) | `joint_1` (base)         | Rotacional | -2.5 a 2.5 rad          |
| POT 2 (GPIO35) | `joint_2` (hombro)       | Rotacional | -2.0 a 2.0 rad          |
| POT 3 (GPIO32) | `joint_gripper` (pinza)  | Prismático | 0.0 a 0.15 m            |

Los dos dedos de la pinza (`joint_dedo_izq`, `joint_dedo_der`) se mueven en espejo,
de forma proporcional a `joint_gripper`, así que un solo potenciómetro controla la
apertura/cierre completo de la pinza.

## Cableado

Cada potenciómetro:
- Pata izquierda → 3V3
- Pata derecha → GND
- Cursor (pata del medio) → pin ADC correspondiente (ver tabla arriba)

Se usaron pines del ADC1 del ESP32 para no interferir con el WiFi/Bluetooth.

## Archivos

- `brazo.urdf` — modelo del robot (provisto por el profesor).
- `esp32_potenciometros.py` — código MicroPython que corre en el ESP32.
- `control_brazo_pybullet.py` — script de PC que recibe los datos y controla la
  simulación. Si no detecta la ESP32 conectada, activa un modo demo con sliders
  virtuales para probar el brazo sin hardware.

## Cómo ejecutar

1. Cargar `esp32_potenciometros.py` en el ESP32 desde Thonny y ejecutarlo (o
   guardarlo como `main.py` en la placa para que arranque solo).
2. Cerrar Thonny por completo para liberar el puerto COM.
3. Instalar dependencias en la PC:
   ```bash
   pip install pybullet pyserial
   ```
4. Ajustar `PUERTO_SERIE` en `control_brazo_pybullet.py` según el puerto COM
   asignado al ESP32.
5. Ejecutar:
   ```bash
   python control_brazo_pybullet.py
   ```
6. Mover los potenciómetros y observar el brazo responder en tiempo real en la
   ventana de PyBullet.

## Evidencia

- Video: [`brazo.mp4`](./brazo.mp4)
- Capturas del cableado, la consola con los datos `DATA,...` y el brazo en
  distintas poses (pinza abierta/cerrada): [agregar aquí]
