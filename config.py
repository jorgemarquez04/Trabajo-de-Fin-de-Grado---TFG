"""
ARCHIVO: config.py
-------------------------------------------------------------------------------------------

RESUMEN: Archivo de parametrización global. Centraliza la IP del robot, la posición HOME, 
límites de seguridad, parámetros de movimiento, velocidad de servoL/speedJ/speedL, 
configuración de gestos, modo de orientación y salida digital de la garra/ventosa.

GUÍA DE LECTURA:
Este archivo no ejecuta lógica de control; define constantes. Conviene estudiarlo primero porque
casi todos los módulos importan `config as cfg`. Puntos de atención: `HOME_JOINTS` define el
arranque seguro; `PIXEL_TO_METER_X/Z` y `MAX_OFFSET_X/Z` determinan el rango del seguimiento libre;
`SERVO_*` afecta a servoL; `TOOL_ORIENT_*` gobierna el modo O; `GRIPPER_*` gobierna la salida
digital.
"""

# =============================================================================================
# ROBOT / CÁMARA
# =============================================================================================

# Direccion IP del robot UR3e, robot_controller.py usa esta IP para conectarse por RTDE 
ROBOT_HOST = "158.42.215.182"

# Nombre/ruta del modelo MediaPipe Hands usado para detectar la mano
# Debe existir en la misma carpeta del proyecto 
MODEL_PATH = "hand_landmarker.task"

# Índice de la cámara que se abre con OpenCV, normalmente 0
CAMERA_ID = 0

# Modo de simulación del código
# True permite probar cámara, interfaz y lógica sin enviar ordenes, False activa comunicación real
DRY_RUN = True



# =============================================================================================
# POSICIÓN HOME
# =============================================================================================

# Posición Home del robot expresada en coordenadas articulares, en radianes
HOME_JOINTS = [

    -0.039243,  # q[0] base

    -1.383261,  # q[1] hombro

    -1.542052,  # q[2] codo

    -0.110935,  # q[3] muñeca 1

    1.534736,   # q[4] muñeca 2

    -0.042171,  # q[5] muñeca 3

]

# Velocidad articular usada al mover el robot hacia HOME con move_J
HOME_SPEED_RAD_S = 0.25     # rad/s

# Aceleración articular usada al mover el robot hacia HOME
HOME_ACCEL_RAD_S2 = 0.35    # rad/s^2



# =============================================================================================
# LÍMITE BLANDO DE SEGURIDAD ARTICULAR
# =============================================================================================

# Activa o desactiva la comprobación del límite articular blando.
    # True  -> el programa vigila una articulación concreta.
    # False -> no se aplica este límite adicional.
ENABLE_SOFT_JOINT_LIMIT = True

# Índice de la articulación protegida, la tercera q[2]
SAFETY_JOINT_INDEX = 2

# Valor articular considerado peligroso para la articulación vigilada
SAFETY_JOINT_LIMIT_RAD = -0.526825  # rad

# Tiempo límite aplicado sobre la articulación
    # "min" -> se bloquea si q[index] baja demasiado
    # "max" -> se bloquea si q[index] sube demasiado
SAFETY_JOINT_LIMIT_TYPE = "max"

# Margen de seguridad aplicado antes de llegar al valor de peligro
SAFETY_JOINT_MARGIN_RAD = 0.04



# =============================================================================================
# INTERFAZ VIRTUAL HÍBRIDA
# =============================================================================================

# Tiempo que la mano debe permanecer dentro del botón virtual para activarlo
BUTTON_DWELL_TIME_S = 1.5   # seg

# Anchura de los botones superior e inferior de la interfaz
BUTTON_WIDTH_PX = 150   # px

# Altura de los botones superior e inferior de la interfaz
BUTTON_HEIGHT_PX = 56

# Anchura de los botones superior e inferior de la interfaz 
SIDE_BUTTON_WIDTH_PX = 56   # px

# Altura de los botones superior e inferior de la interfaz
SIDE_BUTTON_HEIGHT_PX = 150

# Tamaño de los triándulos de las esquinas de la interfaz
CORNER_TRIANGLE_SIZE_PX = 95

# Margen entre los botones virtuales y el borde de la imagen
BUTTON_MARGIN_PX = 18

# Tiempo máximo permitido sin detectar mano antes de detener el control
INTERFACE_HAND_LOST_TIMEOUT_S = 0.20



# =============================================================================================
# MOVIMIENTO CONSTANTE EN PANELES
# =============================================================================================

# Velocidad cartesiana aplicada cuando se activa el panel lateral izquierdo o derecho
# Mueve el TCP en el eje X 
TCP_PANEL_X_SPEED_M_S = 0.025   # m/s

# Velocidad cartesiana aplicada cuando se activa el panel superior o inferior
# Mueve el TCP en el eje Z
TCP_PANEL_Z_SPEED_M_S = 0.020   # m/s

# Componente X de la velocidad usada en los triángulos diagonales
TCP_PANEL_DIAGONAL_X_SPEED_M_S = 0.020

# Componente Z de la velocidad usada en los triángulos diagonales
TCP_PANEL_DIAGONAL_Z_SPEED_M_S = 0.018



# =============================================================================================
# MOVIMIENTO LIBRE POSICIONAL EN ZONA CENTRAL
# =============================================================================================

# Conversión desplazamiento horizontal de la mano en la imagen / desplazamiento real del TCP en X
PIXEL_TO_METER_X = 0.00105  # m/px

# Conversión desplazamiento vertical de la mano en la imagen / desplazamiento real del TCP en Z
PIXEL_TO_METER_Z = 0.00110

# Ganancia global aplicada al desplazamiento mano -> robot
# Valores mayores aumentan sensibilidad; valores menores la reducen
HAND_TO_ROBOT_GAIN = 1.0

# Activa o desactiva el controlde profundidad Y mediante MediaPipe
ENABLE_Y_DEPTH = False

# Modo cálculo de profundidad preparado para el caso de activar Y
DEPTH_MODE = "mediapipe_z"

# Conversión de la coordenada Z normalizada de MediaPipe a metros en Y
MEDIAPIPE_Z_TO_METER_Y = 0.0

# Ganancia adicional del movimiento en profundidad
DEPTH_GAIN = 0.0

# Signo del movimiento el profundidad
DEPTH_SIGN = -1.0

# Desplazamiento máxmimo permitido en X respecto a la pose calibrada
# Limita el alcance del control libre para evitar movmimientos excesivos
MAX_OFFSET_X_M = 0.140

# Desplazamiento máxmimo permitido en Y respecto a la pose calibrada
MAX_OFFSET_Y_M = 0.000

# Desplazamiento máxmimo permitido en Z respecto a la pose calibrada
MAX_OFFSET_Z_M = 0.150

# Signo aplicado al movimiento libre X
FREE_X_SIGN = 1.0

# Signo aplicado al movimiento libre Z
FREE_Z_SIGN = 1.0

# Factor del filtro exponencial EMA aplicado al offset de la mano
# Valores altos reaccionan más rápido pero filtran menos
# Valores más bajos suavizan más pero aumenta el retardo
EMA_ALPHA = 0.30

# Zona muerta mínima de desplazamiento
# Movimientos menores que este valor se consideran ruido y se congelan
DEADBAND_M = 0.0010    # metros

# Velocidad mínima considerada movimiento real por el filtro de congelación
# Si el movimiento es menor, el filtro tiende a dejar el robot quieto
FREEZE_SPEED_M_S = 0.0005   # m/s

# Tiempo durante el que debe antenerse el movimiento pequeño para congelarlo
FREEZE_TIME_S = 0.10    # seg

# Máximo cambio permitido en la pose objetivo por cada ciclo de control
# Evita saltos bruscos cuando la detección de la mano cambia de golpe
MAX_TARGET_STEP_M = 0.0070  # metros/ciclo



# =============================================================================================
# GESTOS DE ROTACIÓN
# =============================================================================================

# Margen usado para decidir si un dedo está bajado
# MediaPipe usa coordenadas normalizadas donde 'y' aumenta hacia abajo
# Si la punta del dedo tiene 'y' mayor a articulación de referencia + margen, se considera bajado
FINGER_DOWN_MARGIN = 0.03

# Define qué ocurre si el índice y el meñique están bajados a la vez
    # True -> orden de STOP, no se gira la base
GESTURE_STOP_ON_BOTH_FINGERS = True



# =============================================================================================
# GIRO DE BASE CON moveJ ASÍNCRONO
# =============================================================================================

# Incremento articular largo enviado a la base cuando se detecta el gesto
# El robot empieza a moverse hacia ese objetivo y se detiene al soltar el gesto
GESTURE_BASE_LONG_STEP_RAD = 1.80   # rad

# Velocidad articular del movimiento de base lanzado con moveJ
GESTURE_MOVEJ_SPEED_RAD_S = 0.75    # rad/s

# Aceleración articular del movimiento de base lanzado con moveJ
GESTURE_MOVEJ_ACCEL_RAD_S2 = 1.00   # rad/s^2

# Aceleración de parada usada con stopJ al soltar el gesto de base
GESTURE_STOPJ_ACCEL_RAD_S2 = 1.30   # rad/s^2



# =============================================================================================
# RTDE / SERVOL
# =============================================================================================

# Periodo principal de control para enviar referencia al robot
CONTROL_PERIOD_S = 0.02     # 0.02seg equivale a unos 50Hz

# Aceleración configurada para los movimientos servoL
# Unidad habitutal de RTDE para servoL -> m/s^2 en componente cartesiana
SERVO_ACC = 0.08

# Velocidad configurada para los movimientos servoL
SERVO_VEL = 0.045   # m/s

# Ganancia interna del servoL
# Valores altos siguen más rápido el objetivo, pero pueden hacer el movimiento más nervioso
SERVO_GAIN = 140

# Timpo de anticipación del servoL
# Valores mayores suavizan más, pero aumentan el retardo
SERVO_LOOKAHEAD_TIME = 0.14    # seg 



# =============================================================================================
# ANTIGUO MODO MUÑECA CON TECLA W (EXPERIMENTAL) -> EN DESUSO
# =============================================================================================

# Activa/desactiva el control automático de muñeca en el modo FREE
# Se deja en False porque la orientación final se controla con el modo O.
ENABLE_WRIST_ROLL_CONTROL = False

# Tecla que activa o desactiva el modo muñeca antiguo.
WRIST_MODE_KEY = "w"

# Índice articular usado para girar la muñeca en el modo W.
# q[5] corresponde a wrist_3_joint, giro final de la herramienta.
WRIST_JOINT_INDEX = 5

# Zona muerta angular respecto al roll de la mano calibrado al entrar en modo W.
# Si la mano gira menos que este valor, la muñeca no se mueve..
WRIST_ROLL_DEADZONE_DEG = 10.0      # grados

# Ángulo de roll de mano a partir del cual se alcanza la velocidad máxima.
WRIST_ROLL_MAX_DEG = 45.0

# Velocidad máxima de giro de muñeca en modo W
WRIST_MAX_SPEED_RAD_S = 0.35    # rad/s

# Aceleración usada por speedJ en el modo W
WRIST_SPEEDJ_ACCEL_RAD_S2 = 0.60    # rad/s^2

# Tiempo de aplicación de cada orden speedJ en modo W
WRIST_SPEEDJ_TIME_S = 0.08  # seg

# Suavizado de la velocidad de muñeca en modo W
# Valores altos reaccionan rápido; valores bajos suavizan más
WRIST_SPEED_EMA_ALPHA = 0.25

# Signo del giro de muñeca en modo W.
WRIST_SIGN = 1.0



# =============================================================================================
# GESTOS DE CONTROL
# =============================================================================================

# Tiempo mínimo que debe mantenerse el gesto de 'PAZ' para calibrar
GESTURE_DWELL_PEACE_CALIBRATE_S = 0.8   # seg

# Tiempo mínimo que debe mantenerse el gesto de 'PERFECTO' para activar/desactivar modo orientación
GESTURE_DWELL_PERFECT_WRIST_S = 0.8

# Tiempo mínimo que debe mantenerse el gesto de 'PUÑO' para conmutar la garra/ventosa
GESTURE_DWELL_FIST_S = 0.4

# Tiempo mínimo que debe mantenerse el gesto de 'GROSEO' para activar parada
GESTURE_DWELL_RUDE_STOP_S = 0.4



# =============================================================================================
# MODO ORIEMTACIÓN HERRAMIENTA 'o'
# =============================================================================================

# Tecla que activa/desactiva el modo orientación de herramienta
TOOL_ORIENT_MODE_KEY = "o"

# Articulación usada para la inclinación principal de la herramienta
TOOL_ORIENT_PITCH_JOINT_INDEX = 3   # q[3] = wrist_1_joint.

# Articulación usada para la orientación lateral de la herramienta
TOOL_ORIENT_SIDE_JOINT_INDEX = 4    # q[4] = wrist_2_joint.

# Articulación usada para el giro de la herramienta sobre sí misma
TOOL_ORIENT_ROLL_JOINT_INDEX = 5    # q[5] = wrist_3_joint

# Velocidad máxima del eje pitch en modo orientación
# En la versión final se activa mediante flechas arriba/abajo
TOOL_ORIENT_PITCH_MAX_SPEED_RAD_S = 0.22    # rad/s

# Velocidad máxima del eje side en modo orientación
# En la versión final se activa mediante flechas izquierda/derecha
TOOL_ORIENT_SIDE_MAX_SPEED_RAD_S = 0.22

# Velocidad máxima del eje roll en modo orientación
# Solo se permite cuando la mano está dentro del círculo central del modo 'o'
TOOL_ORIENT_ROLL_MAX_SPEED_RAD_S = 0.25

# Aceleración usada por speedJ en el modo orientación.
TOOL_ORIENT_ACCEL_RAD_S2 = 0.50     # rad/s^2

# Tiempo de aplicación de cada orden speedJ en el modo orientació
TOOL_ORIENT_TIME_S = 0.08   # seg

# Rango antiguo de desplazamiento vertical de mano para pitc
# En la versión final con flechas ya no es el elemento principal,
# pero se conserva porque las funciones de conversión antiguas siguen existiendo.
TOOL_ORIENT_HAND_Y_RANGE_PX = 130.0     # px

# Zona muerta antigua del desplazamiento vertical de mano para pitch
TOOL_ORIENT_HAND_Y_DEADZONE_PX = 18.0

# Rango antiguo de desplazamiento horizontal de mano para side
# En la versión final con flechas se conserva por compatibilidad
TOOL_ORIENT_HAND_X_RANGE_PX = 150.0

# Zona muerta antigua del desplazamiento horizontal de mano para side
TOOL_ORIENT_HAND_X_DEADZONE_PX = 22.0

# Zona muerta angular para el roll de herramienta
# Si la palma gira menos que este valor respecto al roll neutro, no se mueve q[5]
TOOL_ORIENT_ROLL_DEADZONE_DEG = 10.0    # grados

# Ángulo de roll de palma a partir del cual se alcanza la velocidad máxima de roll
TOOL_ORIENT_ROLL_MAX_DEG = 45.0

# Suavizado de velocidades en modo orientación
# En pitch y side se corta a cero si no hay flecha activa para evitar deriva
TOOL_ORIENT_SPEED_EMA_ALPHA = 0.25

# Signo del eje pitch en modo orientación.
TOOL_ORIENT_PITCH_SIGN = 1.0

# Signo del eje side en modo orientación.
TOOL_ORIENT_SIDE_SIGN = 1.0

# Signo del eje roll en modo orientación.
TOOL_ORIENT_ROLL_SIGN = 1.0



# =============================================================================================
# CONTROL GARRA / VENTOSA POR SALIDA DIGITAL UR
# =============================================================================================

# Activa o desactiva el control de salida digital para garra o ventosa
ENABLE_GRIPPER_OUTPUT = True

# Número de salida digutal estándar del UR asociada a la herramienta
GRIPPER_OUTPUT_ID = 0

# Valor lógico que se envía para activar/cerrar la herramienta
# True activa la salida. Si físicamente funciona al revés, cambiar a False
GRIPPER_CLOSE_VALUE = True

# Seguridad al detener el sistema
# True -> Al pulsar HOME, espacio, gesto groseo o salir -> se desactiva la salida
GRIPPER_OPEN_ON_STOP = True



# =============================================================================================
# FLECHAS LOCALES DEL MODO ORIENTACIÓN 'o'
# =============================================================================================

# Tiempo que la mano debe mantenerse dentro de la flecha para activarla
TOOL_ARROW_DWELL_TIME_S = 0.35  # seg

# Distancia desde el origen local del modo 'o' hasta el centro de cada flecha
# El origen local se crea en la posición de la mano al centrar en modo 'o'
TOOL_ARROW_OFFSET_PX = 105  # px

# Anchura de la zona activa de cada flecha
TOOL_ARROW_WIDTH_PX = 80

# Altura de la zona activa de cada flecha
TOOL_ARROW_HEIGHT_PX = 60

# Radio del círculo central del modo 'o'
TOOL_ROLL_CENTER_RADIUS_PX = 42

# Tiempo que la mano debe mantenerse dentro del círculo central
# antes de permitir el roll de herramienta.
TOOL_ROLL_CENTER_DWELL_TIME_S = 0.8



# =============================================================================================
# GUÍA VISUAL DE USUARIO
# =============================================================================================

# Activa o desactiva que se muestre la guía de control al iniciar el programa
SHOW_USER_GUIDE = True

# Ruta de la imagen de guía.
# Si está en la misma carpeta del proyecto, basta con el nombre del archivo.
USER_GUIDE_IMAGE_PATH = "guia_control_robot.png"

# Tamaño fijo de visualización en formato DIN A4 vertical.
# DIN A4 tiene proporción aproximada 1 : 1.414
USER_GUIDE_DISPLAY_WIDTH_PX = 730
USER_GUIDE_DISPLAY_HEIGHT_PX = 1032