"""
ARCHIVO: main_hybrid_control.py
-------------------------------------------------------------------------------------------

Este archivo es el núcleo del proyecto. Integra todos los módulos:

    - HandTracker: lee cámara, detecta mano y reconoce gestos.
    - TrajectoryMapper: convierte desplazamiento de mano en offset cartesiano del robot.
    - EmaFilter y DeadbandFreezeFilter: suavizan la señal y eliminan microtemblores.
    - VirtualInterface: dibuja botones/paneles virtuales y detecta entradas.
    - RobotController: envía órdenes RTDE al UR3e.

La lógica general del programa es una máquina de estados:

    1. Arranca cámara, pero no mueve robot.
    2. El usuario pulsa h -> se conecta al robot y va a HOME.
    3. El usuario calibra con c o gesto de PAZ.
    4. En modo FREE, la mano mueve el TCP con servoL.
    5. Los paneles virtuales activan movimientos por speedL.
    6. Índice/meñique bajados giran la base con moveJ asíncrono.
    7. Gesto PERFECTO o tecla O activa modo orientación de herramienta.
    8. Puño activa/desactiva la salida digital de garra/ventosa.
    9. Barra espaciadora o gesto grosero bloquean el control.
    10. h permite reset global y vuelta a HOME.

"""

import time
import cv2
import msvcrt
import config as cfg

from filters import EmaFilter, DeadbandFreezeFilter
from hand_tracker import HandTracker
from robot_controller import RobotController
from trajectory_mapper import TrajectoryMapper
from virtual_interface import (
    VirtualInterface,
    draw_tool_orient_overlay,
    get_tool_arrow_candidate,
    is_tool_roll_center_active,
)




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def read_key():
    """Lee teclado desde OpenCV y consola Windows. Devuelve una tecla en minúscula o None."""

    cv_key = cv2.waitKey(1) & 0xFF

    if cv_key != 255:
        try:
            return chr(cv_key).lower()
        except Exception:
            return None

    if msvcrt.kbhit():
        return msvcrt.getwch().lower()

    return None




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def limit_target_step(previous_pose, target_pose_raw, max_step_m):
    """Limita el salto máximo en X/Y/Z entre la pose anterior y la nueva pose objetivo."""

    previous_pose = list(previous_pose)
    target_pose_raw = list(target_pose_raw)

    target_pose = previous_pose.copy()

    for i in range(3):

        delta = target_pose_raw[i] - previous_pose[i]

        if delta > max_step_m:
            delta = max_step_m

        elif delta < -max_step_m:
            delta = -max_step_m

        target_pose[i] = previous_pose[i] + delta

    target_pose[3:6] = target_pose_raw[3:6]

    return target_pose




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def normalize_angle_deg(angle):
    """Normaliza un ángulo al rango [-180, 180] para evitar saltos al comparar giros."""

    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def roll_error_to_wrist_speed(roll_error_deg):
    """Convierte error de roll de mano en velocidad articular para el modo W antiguo."""

    abs_error = abs(float(roll_error_deg))

    if abs_error < cfg.WRIST_ROLL_DEADZONE_DEG:
        return 0.0

    usable_error = abs_error - cfg.WRIST_ROLL_DEADZONE_DEG

    usable_max = max(
        cfg.WRIST_ROLL_MAX_DEG - cfg.WRIST_ROLL_DEADZONE_DEG,
        1.0,
    )

    ratio = min(usable_error / usable_max, 1.0)

    speed = ratio * cfg.WRIST_MAX_SPEED_RAD_S

    if roll_error_deg < 0:
        speed = -speed

    return cfg.WRIST_SIGN * speed




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def hand_y_error_to_tool_pitch_speed(y_error_px):
    """Convierte desplazamiento vertical de mano en velocidad de pitch del modo 'O' continuo antiguo."""

    abs_error = abs(float(y_error_px))

    if abs_error < cfg.TOOL_ORIENT_HAND_Y_DEADZONE_PX:
        return 0.0

    usable_error = abs_error - cfg.TOOL_ORIENT_HAND_Y_DEADZONE_PX

    usable_max = max(
        cfg.TOOL_ORIENT_HAND_Y_RANGE_PX - cfg.TOOL_ORIENT_HAND_Y_DEADZONE_PX,
        1.0,
    )

    ratio = min(usable_error / usable_max, 1.0)

    speed = ratio * cfg.TOOL_ORIENT_PITCH_MAX_SPEED_RAD_S

    if y_error_px < 0:
        speed = -speed

    return cfg.TOOL_ORIENT_PITCH_SIGN * speed




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def hand_x_error_to_tool_side_speed(x_error_px):
    """Convierte desplazamiento horizontal de mano en velocidad lateral del modo 'O' continuo antiguo."""

    abs_error = abs(float(x_error_px))

    if abs_error < cfg.TOOL_ORIENT_HAND_X_DEADZONE_PX:
        return 0.0

    usable_error = abs_error - cfg.TOOL_ORIENT_HAND_X_DEADZONE_PX

    usable_max = max(
        cfg.TOOL_ORIENT_HAND_X_RANGE_PX - cfg.TOOL_ORIENT_HAND_X_DEADZONE_PX,
        1.0,

    )

    ratio = min(usable_error / usable_max, 1.0)

    speed = ratio * cfg.TOOL_ORIENT_SIDE_MAX_SPEED_RAD_S

    if x_error_px < 0:
        speed = -speed

    return cfg.TOOL_ORIENT_SIDE_SIGN * speed




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def roll_error_to_tool_roll_speed(roll_error_deg):
    """Convierte giro de palma en velocidad de roll de herramienta para el modo 'O' final."""

    abs_error = abs(float(roll_error_deg))

    if abs_error < cfg.TOOL_ORIENT_ROLL_DEADZONE_DEG:
        return 0.0

    usable_error = abs_error - cfg.TOOL_ORIENT_ROLL_DEADZONE_DEG

    usable_max = max(
        cfg.TOOL_ORIENT_ROLL_MAX_DEG - cfg.TOOL_ORIENT_ROLL_DEADZONE_DEG,
        1.0,
    )

    ratio = min(usable_error / usable_max, 1.0)

    speed = ratio * cfg.TOOL_ORIENT_ROLL_MAX_SPEED_RAD_S

    if roll_error_deg < 0:
        speed = -speed

    return cfg.TOOL_ORIENT_ROLL_SIGN * speed




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def get_wrist_roll_gesture(hand_features, neutral_roll_deg):
    """Devuelve WRIST_LEFT, WRIST_RIGHT o NONE para el modo 'W' antiguo."""

    if not cfg.ENABLE_WRIST_ROLL_CONTROL:
        return "NONE"

    if neutral_roll_deg is None:
        return "NONE"

    current_roll_deg = hand_features.get("palm_roll_deg", None)

    if current_roll_deg is None:
        return "NONE"

    roll_error_deg = normalize_angle_deg(current_roll_deg - neutral_roll_deg)

    if abs(roll_error_deg) < cfg.WRIST_ROLL_DEADZONE_DEG:
        return "NONE"

    if roll_error_deg > 0:
        return "WRIST_RIGHT"

    return "WRIST_LEFT"




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def create_robot():
    """Crea RobotController con los parámetros RTDE definidos en config.py."""

    return RobotController(
        host=cfg.ROBOT_HOST,
        dry_run=cfg.DRY_RUN,
        servo_acc=cfg.SERVO_ACC,
        servo_vel=cfg.SERVO_VEL,
        servo_period_s=cfg.CONTROL_PERIOD_S,
        lookahead_time=cfg.SERVO_LOOKAHEAD_TIME,
        gain=cfg.SERVO_GAIN,
    )




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def send_button_command(robot, command):
    """Traduce comandos de la interfaz virtual en movimientos speedL del TCP."""

    if command == VirtualInterface.BASE_LEFT:
        robot.speed_tcp_xz(
            vx_m_s=-cfg.TCP_PANEL_X_SPEED_M_S,
            vz_m_s=0.0,
        )

    elif command == VirtualInterface.BASE_RIGHT:
        robot.speed_tcp_xz(
            vx_m_s=cfg.TCP_PANEL_X_SPEED_M_S,
            vz_m_s=0.0,
        )

    elif command == VirtualInterface.Z_UP:
        robot.speed_tcp_z(cfg.TCP_PANEL_Z_SPEED_M_S)

    elif command == VirtualInterface.Z_DOWN:
        robot.speed_tcp_z(-cfg.TCP_PANEL_Z_SPEED_M_S)

    elif command == VirtualInterface.MIX_LEFT_UP:
        robot.speed_tcp_xz(
            vx_m_s=-cfg.TCP_PANEL_DIAGONAL_X_SPEED_M_S,
            vz_m_s=cfg.TCP_PANEL_DIAGONAL_Z_SPEED_M_S,
        )

    elif command == VirtualInterface.MIX_RIGHT_UP:
        robot.speed_tcp_xz(
            vx_m_s=cfg.TCP_PANEL_DIAGONAL_X_SPEED_M_S,
            vz_m_s=cfg.TCP_PANEL_DIAGONAL_Z_SPEED_M_S,
        )

    elif command == VirtualInterface.MIX_LEFT_DOWN:
        robot.speed_tcp_xz(
            vx_m_s=-cfg.TCP_PANEL_DIAGONAL_X_SPEED_M_S,
            vz_m_s=-cfg.TCP_PANEL_DIAGONAL_Z_SPEED_M_S,
        )

    elif command == VirtualInterface.MIX_RIGHT_DOWN:
        robot.speed_tcp_xz(
            vx_m_s=cfg.TCP_PANEL_DIAGONAL_X_SPEED_M_S,
            vz_m_s=-cfg.TCP_PANEL_DIAGONAL_Z_SPEED_M_S,
        )

    else:
        robot.stop_speed()




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def gesture_held_since(is_active, start_time, dwell_s):
    """Comprueba si un gesto lleva activo al menos dwell_s segundos."""

    now = time.time()

    if is_active:
        if start_time is None:
            start_time = now

        if now - start_time >= dwell_s:
            return True, start_time

        return False, start_time

    return False, None




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def draw_rude_warning_overlay(frame):
    """Dibuja un aviso visual grande cuando se activa la parada por gesto grosero."""

    h, w = frame.shape[:2]

    overlay = frame.copy()

    panel_w = int(w * 0.78)
    panel_h = int(h * 0.42)

    x1 = (w - panel_w) // 2
    y1 = (h - panel_h) // 2
    x2 = x1 + panel_w
    y2 = y1 + panel_h

    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 80), -1)
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 255), 4)

    alpha = 0.72

    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    face_center = (w // 2, y1 + 95)
    face_r = 48

    cv2.circle(frame, face_center, face_r, (0, 0, 255), -1)
    cv2.circle(frame, face_center, face_r, (255, 255, 255), 3)

    cv2.line(
        frame,
        (face_center[0] - 25, face_center[1] - 18),
        (face_center[0] - 5, face_center[1] - 8),
        (255, 255, 255),
        4,
    )

    cv2.line(
        frame,
        (face_center[0] + 25, face_center[1] - 18),
        (face_center[0] + 5, face_center[1] - 8),
        (255, 255, 255),
        4,
    )

    cv2.circle(frame, (face_center[0] - 15, face_center[1] - 2), 4, (255, 255, 255), -1)
    cv2.circle(frame, (face_center[0] + 15, face_center[1] - 2), 4, (255, 255, 255), -1)

    cv2.ellipse(
        frame,
        (face_center[0], face_center[1] + 22),
        (18, 10),
        0,
        20,
        160,
        (255, 255, 255),
        3,
    )

    main_text = "¡NO SEAS GROSERO!"

    sub_text = "PARADA DE EMERGENCIA ACTIVADA"

    main_scale = 1.35
    sub_scale = 0.85

    (tw, th), _ = cv2.getTextSize(main_text, cv2.FONT_HERSHEY_DUPLEX, main_scale, 3)

    tx = (w - tw) // 2
    ty = y1 + 185

    cv2.putText(
        frame,
        main_text,
        (tx, ty),
        cv2.FONT_HERSHEY_DUPLEX,
        main_scale,
        (0, 255, 255),
        8,
        cv2.LINE_AA,
    )

    cv2.putText(
        frame,
        main_text,
        (tx, ty),
        cv2.FONT_HERSHEY_DUPLEX,
        main_scale,
        (255, 255, 255),
        3,
        cv2.LINE_AA,
    )

    (sw, sh), _ = cv2.getTextSize(sub_text, cv2.FONT_HERSHEY_SIMPLEX, sub_scale, 2)

    sx = (w - sw) // 2
    sy = ty + 45

    cv2.putText(
        frame,
        sub_text,
        (sx, sy),
        cv2.FONT_HERSHEY_SIMPLEX,
        sub_scale,
        (0, 255, 255),
        5,
        cv2.LINE_AA,
    )

    cv2.putText(
        frame,
        sub_text,
        (sx, sy),
        cv2.FONT_HERSHEY_SIMPLEX,
        sub_scale,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def show_user_guide_window():
    """
    Carga y muestra la guía visual de control del robot.

    La imagen se fuerza a formato DIN A4 vertical para que se vea grande,
    clara y con proporción constante.
    """

    if not cfg.SHOW_USER_GUIDE:
        return

    guide = cv2.imread(cfg.USER_GUIDE_IMAGE_PATH)

    if guide is None:
        print("[WARN] No se pudo cargar la guía de usuario:")
        print(" ruta =", cfg.USER_GUIDE_IMAGE_PATH)
        return

    # Tamaño DIN A4 vertical definido en config.py
    display_w = int(cfg.USER_GUIDE_DISPLAY_WIDTH_PX)
    display_h = int(cfg.USER_GUIDE_DISPLAY_HEIGHT_PX)

    # Redimensionamos la imagen al tamaño fijo A4.
    guide = cv2.resize(
        guide,
        (display_w, display_h),
        interpolation=cv2.INTER_AREA,
    )

    cv2.namedWindow("Guia de control del robot", cv2.WINDOW_NORMAL)

    # Forzamos también el tamaño de la ventana.
    cv2.resizeWindow(
        "Guia de control del robot",
        display_w,
        display_h,
    )

    cv2.imshow("Guia de control del robot", guide)

    # Guía a la izquierda.
    cv2.moveWindow("Guia de control del robot", 20, 40)




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def main():
    """Ejecuta el bucle principal de cámara, gestos, estados y control del robot."""

    # ----------------------
    # ARRANQUE DEL PROGRAMA
    # ----------------------
    print("=== Control híbrido UR3 - versión limpia ===")
    print("")
    print("Controles:")
    print("  h -> mover robot a HOME")
    print("  c -> calibrar mano + robot")
    print("  q -> salir")
    print("")
    print("DRY_RUN:", cfg.DRY_RUN)
    print("")

    # Mostramos la guía visual de controles al iniciar el programa.
    show_user_guide_window()

    # --------------------------------
    # CREACIÓN DEL TRACKER DE LA MANO
    # --------------------------------
    # Abre la cámara y carga de MediaPipe Hands
    tracker = HandTracker(
        model_path=cfg.MODEL_PATH,
        camera_id=cfg.CAMERA_ID,
    )

    # Leemos un primer frame para conocer resolución de cámara
    frame, _ = tracker.read()
    h, w, _ = frame.shape

    cv2.namedWindow("Control híbrido UR3", cv2.WINDOW_NORMAL)

    cv2.resizeWindow(
        "Control híbrido UR3",
        760,
        570,
    )

    cv2.moveWindow("Control híbrido UR3", 830, 40)

    # -------------------------------
    # CREACIÓN DE LA INTERFAZ VISUAL
    # -------------------------------
    # Define botones, triángulos y dwell time
    interface = VirtualInterface(
        frame_width=w,
        frame_height=h,
        button_width_px=cfg.BUTTON_WIDTH_PX,
        button_height_px=cfg.BUTTON_HEIGHT_PX,
        side_button_width_px=cfg.SIDE_BUTTON_WIDTH_PX,
        side_button_height_px=cfg.SIDE_BUTTON_HEIGHT_PX,
        corner_triangle_size_px=cfg.CORNER_TRIANGLE_SIZE_PX,
        button_margin_px=cfg.BUTTON_MARGIN_PX,
        dwell_time_s=cfg.BUTTON_DWELL_TIME_S,
    )


    # ----------------------------------
    # CREACIÓN DEL MAPPER MANO -> ROBOT
    # ----------------------------------
    # Convierte desplazamiento de mano en píxeles a offset del robot en metros
    mapper = TrajectoryMapper(
        pixel_to_meter_x=cfg.PIXEL_TO_METER_X,
        pixel_to_meter_z=cfg.PIXEL_TO_METER_Z,
        gain=cfg.HAND_TO_ROBOT_GAIN,
        max_offset_x=cfg.MAX_OFFSET_X_M,
        max_offset_y=cfg.MAX_OFFSET_Y_M,
        max_offset_z=cfg.MAX_OFFSET_Z_M,
        enable_y_depth=cfg.ENABLE_Y_DEPTH,
        depth_mode=cfg.DEPTH_MODE,
        mediapipe_z_to_meter_y=cfg.MEDIAPIPE_Z_TO_METER_Y,
        depth_gain=cfg.DEPTH_GAIN,
        depth_sign=cfg.DEPTH_SIGN,
    )


    # --------------------
    # FILTROS DE LA SEÑAL
    # --------------------
    # EMA suaviza y DeadbandFreeze elimina microtemblores
    ema = EmaFilter(cfg.EMA_ALPHA)

    freeze = DeadbandFreezeFilter(
        deadband_m=cfg.DEADBAND_M,
        freeze_speed_m_s=cfg.FREEZE_SPEED_M_S,
        freeze_time_s=cfg.FREEZE_TIME_S,
    )


    # --------------------------------
    # VARIABLES DE ESTADO DEL SISTEMA
    # --------------------------------
    # El robot empieza desconectado. Se crea al pulsar 'H'
    robot = None
    home_done = False
    calibrated = False

    # Bloqueo de parada manual o gesto grosero
    manual_stop_latched = False
    stop_reason = None

    # Temporizador dwell de gesto
    peace_gesture_start_time = None
    perfect_gesture_start_time = None
    fist_gesture_start_time = None
    rude_gesture_start_time = None

    # Memorias de transición y depuración
    last_hand_seen = time.time()
    last_state = None
    last_command_printed = None
    last_target_pose = None
    last_free_control_mode = None
    last_gesture_step_time = 0.0
    active_base_gesture = None

    # Variables del modo muñeca 'W' antiguo
    wrist_mode_enabled = False
    wrist_neutral_roll_deg = None
    wrist_filtered_speed = 0.0

    # Variables del modo muñeca orientación 'O'
    tool_orient_mode_enabled = False
    tool_orient_origin_x_px = None
    tool_orient_origin_y_px = None
    tool_orient_neutral_roll_deg = None
    tool_orient_pitch_filtered_speed = 0.0
    tool_orient_side_filtered_speed = 0.0
    tool_orient_roll_filtered_speed = 0.0

    # Variables de felchas locales del modo 'O'
    tool_arrow_candidate = "NONE"
    tool_arrow_active_command = "NONE"
    tool_arrow_candidate_since = None
    tool_arrow_progress = 0.0

    # Variables del círculo local del modo'O'
    tool_roll_center_since = None
    tool_roll_center_progress = 0.0
    tool_roll_center_enabled = False

    # Estado de salida digital de garra/ventosa
    gripper_closed = False
    gripper_fist_armed = True
    neutral_roll_deg = None
    safety_stop_sent = False
    safety_recovery_mode = False


    # ----------------
    # BUCLE PRINCIPAL
    # ----------------
    try:

        while True:

            # -------------------------------
            # 1. LEER CÁMARA Y DETECTAR MANO
            # -------------------------------
            frame, hand_features = tracker.read()

            # Tiempo de iteración
            now = time.time()

            # Tecla pulsada en esta iteración
            key = read_key()


            # -----------------------
            # 2. RESET GLOBAL / HOME
            # -----------------------
            # 'H' reinicia todo, detiene mocimientos, desactiva modos, borra calibración y mueve a HOME
            if key == "h":
                print("")
                print("=== RESET GLOBAL: VOLVIENDO A HOME ===")

                if robot is None:
                    robot = create_robot()

                    if cfg.ENABLE_GRIPPER_OUTPUT and cfg.GRIPPER_OPEN_ON_STOP:

                        robot.set_digital_output(
                            cfg.GRIPPER_OUTPUT_ID,
                            not cfg.GRIPPER_CLOSE_VALUE,
                        )

                        gripper_closed = False
                        gripper_fist_armed = True
                        tool_arrow_candidate = "NONE"
                        tool_arrow_active_command = "NONE"
                        tool_arrow_candidate_since = None
                        tool_arrow_progress = 0.0
                        tool_roll_center_since = None
                        tool_roll_center_progress = 0.0
                        tool_roll_center_enabled = False

                else:
                    robot.stop_motion()
                    time.sleep(0.08)

                wrist_mode_enabled = False
                wrist_neutral_roll_deg = None
                wrist_filtered_speed = 0.0
                tool_orient_mode_enabled = False
                tool_orient_origin_x_px = None
                tool_orient_origin_y_px = None
                tool_orient_neutral_roll_deg = None
                tool_orient_pitch_filtered_speed = 0.0
                tool_orient_side_filtered_speed = 0.0
                tool_orient_roll_filtered_speed = 0.0
                calibrated = False
                manual_stop_latched = False
                stop_reason = None
                active_base_gesture = None
                last_free_control_mode = None
                last_state = None
                last_command_printed = None
                last_target_pose = None
                neutral_roll_deg = None
                safety_stop_sent = False
                safety_recovery_mode = False
                peace_gesture_start_time = None
                perfect_gesture_start_time = None
                fist_gesture_start_time = None
                rude_gesture_start_time = None
                mapper.hand_origin_px = None
                mapper.palm_origin_z_norm = None
                mapper.robot_origin_pose = None
                interface.reset()

                ema.reset([0.0, 0.0, 0.0])
                freeze.reset([0.0, 0.0, 0.0])

                print("Moviendo robot a HOME...")

                robot.move_home(
                    home_joints=cfg.HOME_JOINTS,
                    speed_rad_s=cfg.HOME_SPEED_RAD_S,
                    accel_rad_s2=cfg.HOME_ACCEL_RAD_S2,
                )

                home_done = True
                calibrated = False

                print("Robot en HOME.")
                print("Coloca la mano neutra y pulsa C o haz el gesto de PAZ.")
                print("")

                continue


            # --------------------
            # 3. SALIDA DEL PROGRAMA
            # --------------------
            if key == "q":
                break


            # -----------------------------
            # 4. PARADA MANUAL CON ESPACIO
            # -----------------------------
            if key == " ":

                if robot is not None:
                    robot.stop_motion()

                    if cfg.ENABLE_GRIPPER_OUTPUT and cfg.GRIPPER_OPEN_ON_STOP:
                        robot.set_digital_output(
                            cfg.GRIPPER_OUTPUT_ID,
                            not cfg.GRIPPER_CLOSE_VALUE,
                        )

                        gripper_closed = False
                        gripper_fist_armed = True
                        tool_arrow_candidate = "NONE"
                        tool_arrow_active_command = "NONE"
                        tool_arrow_candidate_since = None
                        tool_arrow_progress = 0.0

                calibrated = False
                manual_stop_latched = True
                stop_reason = "MANUAL"
                wrist_mode_enabled = False
                wrist_filtered_speed = 0.0
                tool_orient_mode_enabled = False
                tool_orient_origin_x_px = None
                tool_orient_origin_y_px = None
                tool_orient_neutral_roll_deg = None
                tool_orient_pitch_filtered_speed = 0.0
                tool_orient_side_filtered_speed = 0.0
                tool_orient_roll_filtered_speed = 0.0
                tool_arrow_candidate = "NONE"
                tool_arrow_active_command = "NONE"
                tool_arrow_candidate_since = None
                tool_arrow_progress = 0.0
                tool_roll_center_since = None
                tool_roll_center_progress = 0.0
                tool_roll_center_enabled = False
                last_free_control_mode = None
                active_base_gesture = None
                interface.reset()

                print("")
                print("PARADA DE EMERGENCIA MANUAL ACTIVADA")
                print("Control bloqueado. Pulsa h y vuelve a calibrar con PAZ.")
                print("")


            # --------------------------
            # 5. TOGGLE MODO MUÑECA 'W'
            # --------------------------
            if key == cfg.WRIST_MODE_KEY and calibrated and hand_features is not None:
                wrist_mode_enabled = not wrist_mode_enabled

                if wrist_mode_enabled:

                    robot.stop_motion()
                    time.sleep(0.08)
                    wrist_neutral_roll_deg = hand_features.get("palm_roll_deg", None)
                    wrist_filtered_speed = 0.0
                    interface.reset()
                    last_state = None
                    last_free_control_mode = None
                    active_base_gesture = None

                    print("")
                    print("MODO MUÑECA ACTIVADO")
                    print(
                        "Roll neutro muñeca:",
                        None if wrist_neutral_roll_deg is None else round(wrist_neutral_roll_deg, 2),
                    )
                    print("")

                else:

                    robot.stop_speed()
                    time.sleep(0.08)
                    robot_pose = robot.get_tcp_pose()
                    mapper.calibrate_origin(
                        hand_features=hand_features,
                        robot_pose=robot_pose,
                    )

                    ema.reset([0.0, 0.0, 0.0])
                    freeze.reset([0.0, 0.0, 0.0])
                    last_target_pose = robot_pose.copy()
                    last_state = None
                    last_free_control_mode = None
                    wrist_filtered_speed = 0.0

                    print("")
                    print("MODO MUÑECA DESACTIVADO")
                    print("Recalibración suave al volver al control normal.")
                    print("")


            # ------------------
            # 6. CENTRO DE MANO
            # ------------------
            center_px = None

            if hand_features is not None:
                center_px = hand_features["center_px"]
                last_hand_seen = now


            # -----------------------------------------
            # 7. TOGGLE MODO ORIENTACIÓN 'O' CON TECLA
            # -----------------------------------------
            if key == cfg.TOOL_ORIENT_MODE_KEY and calibrated and hand_features is not None:
                tool_orient_mode_enabled = not tool_orient_mode_enabled

                if tool_orient_mode_enabled:

                    robot.stop_motion()
                    time.sleep(0.08)
                    tool_orient_origin_x_px = float(hand_features["center_px"][0])
                    tool_orient_origin_y_px = float(hand_features["center_px"][1])
                    tool_orient_neutral_roll_deg = hand_features.get("palm_roll_deg", None)
                    tool_orient_pitch_filtered_speed = 0.0
                    tool_orient_side_filtered_speed = 0.0
                    tool_orient_roll_filtered_speed = 0.0
                    wrist_mode_enabled = False
                    wrist_filtered_speed = 0.0
                    interface.reset()
                    last_state = None
                    last_free_control_mode = None
                    active_base_gesture = None

                    print("")
                    print("MODO ORIENTACIÓN HERRAMIENTA ACTIVADO")
                    print("Mano arriba/abajo      -> inclina herramienta")
                    print("Mano izquierda/derecha -> orienta lateral")
                    print("Inclinar palma         -> gira herramienta")
                    print("")

                else:

                    robot.stop_speed()
                    time.sleep(0.08)
                    robot_pose = robot.get_tcp_pose()
                    mapper.calibrate_origin(
                        hand_features=hand_features,
                        robot_pose=robot_pose,
                    )
                    ema.reset([0.0, 0.0, 0.0])
                    freeze.reset([0.0, 0.0, 0.0])
                    last_target_pose = robot_pose.copy()
                    last_state = None
                    last_free_control_mode = None
                    tool_orient_mode_enabled = False
                    tool_orient_origin_x_px = None
                    tool_orient_origin_y_px = None
                    tool_orient_neutral_roll_deg = None
                    tool_orient_pitch_filtered_speed = 0.0
                    tool_orient_side_filtered_speed = 0.0
                    tool_orient_roll_filtered_speed = 0.0

                    print("")
                    print("MODO ORIENTACIÓN HERRAMIENTA DESACTIVADO")
                    print("Recalibración suave al volver al control normal.")
                    print("")


            # ------------------------------
            # 8. CALIBRACIÓN MANUAL CON 'C'
            # ------------------------------
            if key == "c" and not calibrated:

                if not home_done:
                    print("Primero pulsa h para mover el robot a HOME.")

                elif hand_features is None:
                    print("No hay mano detectada. Coloca la mano y vuelve a pulsar c.")

                else:
                    if robot is None:
                        robot = create_robot()

                    robot_pose = robot.get_tcp_pose()
                    mapper.calibrate_origin(
                        hand_features=hand_features,
                        robot_pose=robot_pose,
                    )

                    ema.reset([0.0, 0.0, 0.0])
                    freeze.reset([0.0, 0.0, 0.0])
                    interface.reset()
                    calibrated = True
                    last_state = None
                    last_command_printed = None
                    last_target_pose = robot_pose.copy()
                    last_free_control_mode = None
                    neutral_roll_deg = hand_features.get("palm_roll_deg", None)
                    active_wrist_gesture = None

                    print("")
                    print("Calibración realizada.")
                    print("Origen mano px:", hand_features["center_px"])
                    print("Z MediaPipe origen:", round(hand_features["palm_z_norm"], 6))
                    print("Origen robot:", [round(float(v), 4) for v in robot_pose])
                    print("Roll mano origen:", None if neutral_roll_deg is None else round(neutral_roll_deg, 2))
                    print("")


            # --------------------------------
            # 9. CALIBRACIÓN CON GESTO DE PAZ
            # --------------------------------
            peace_calibrate_triggered, peace_gesture_start_time = gesture_held_since(
                hand_features is not None
                and home_done
                and not calibrated
                and not manual_stop_latched
                and hand_features.get("peace_gesture", False),
                peace_gesture_start_time,
                cfg.GESTURE_DWELL_PEACE_CALIBRATE_S,
            )

            if peace_calibrate_triggered:

                if robot is None:
                    robot = create_robot()

                robot_pose = robot.get_tcp_pose()
                mapper.calibrate_origin(
                    hand_features=hand_features,
                    robot_pose=robot_pose,
                )

                ema.reset([0.0, 0.0, 0.0])
                freeze.reset([0.0, 0.0, 0.0])
                interface.reset()
                calibrated = True
                last_state = None
                last_command_printed = None
                last_target_pose = robot_pose.copy()
                last_free_control_mode = None
                neutral_roll_deg = hand_features.get("palm_roll_deg", None)
                active_base_gesture = None
                peace_gesture_start_time = None

                print("")
                print("CALIBRACIÓN REALIZADA CON GESTO DE PAZ")
                print("Control habilitado.")
                print("Origen mano px:", hand_features["center_px"])
                print("Z MediaPipe origen:", round(hand_features["palm_z_norm"], 6))
                print("Origen robot:", [round(float(v), 4) for v in robot_pose])
                print("Roll mano origen:", None if neutral_roll_deg is None else round(neutral_roll_deg, 2))
                print("")


            # -------------------------------------------
            # 10. SISTEMA BLOQUEADO SI NO ESTÁ CALIBRADO
            # -------------------------------------------
            if not calibrated:

                interface.draw_locked(
                    frame,
                    center_px=center_px,
                    home_done=home_done,
                )

                if manual_stop_latched and stop_reason == "RUDE":
                    draw_rude_warning_overlay(frame)

                cv2.imshow("Control híbrido UR3", frame)

                continue


            # -----------------
            # 11. MANO PERDIDA
            # ------------------
            if hand_features is None:
                safety_stop_sent = False

                if (
                    now - last_hand_seen > cfg.INTERFACE_HAND_LOST_TIMEOUT_S
                    and robot is not None
                ):

                    robot.stop_motion()

                interface.draw(
                    frame,
                    center_px=None,
                    state=VirtualInterface.NO_HAND,
                    command=VirtualInterface.STOP,
                    progress=0.0,
                )

                cv2.imshow("Control híbrido UR3", frame)

                continue


            # ----------------------------
            # 12. GESTO GROSERO -> PARADA
            # ----------------------------
            rude_stop_triggered, rude_gesture_start_time = gesture_held_since(
                hand_features is not None
                and hand_features.get("rude_gesture", False),
                rude_gesture_start_time,
                cfg.GESTURE_DWELL_RUDE_STOP_S,
            )

            if rude_stop_triggered:

                if robot is not None:
                    robot.stop_motion()

                    if cfg.ENABLE_GRIPPER_OUTPUT and cfg.GRIPPER_OPEN_ON_STOP:
                        robot.set_digital_output(
                            cfg.GRIPPER_OUTPUT_ID,
                            not cfg.GRIPPER_CLOSE_VALUE,
                        )

                        gripper_closed = False
                        gripper_fist_armed = True
                        tool_arrow_candidate = "NONE"
                        tool_arrow_active_command = "NONE"
                        tool_arrow_candidate_since = None
                        tool_arrow_progress = 0.0
                        tool_roll_center_since = None
                        tool_roll_center_progress = 0.0
                        tool_roll_center_enabled = False

                calibrated = False
                manual_stop_latched = True
                stop_reason = "RUDE"
                wrist_mode_enabled = False
                wrist_filtered_speed = 0.0
                last_free_control_mode = None
                active_base_gesture = None
                tool_orient_mode_enabled = False
                tool_orient_origin_x_px = None
                tool_orient_origin_y_px = None
                tool_orient_neutral_roll_deg = None
                tool_orient_pitch_filtered_speed = 0.0
                tool_orient_side_filtered_speed = 0.0
                tool_orient_roll_filtered_speed = 0.0
                interface.reset()
                rude_gesture_start_time = None

                print("")
                print("GESTO GROSERO DETECTADO")
                print("Parada manual activada. Pulsa h y vuelve a calibrar con PAZ.")
                print("")

                draw_rude_warning_overlay(frame)

                cv2.imshow("Control híbrido UR3", frame)

                continue


            # --------------------------------------
            # 13. GESTO PERFECTO -> TOGGLE MODO 'O'
            # --------------------------------------
            perfect_triggered, perfect_gesture_start_time = gesture_held_since(
                hand_features is not None
                and calibrated
                and not manual_stop_latched
                and hand_features.get("perfect_gesture", False),
                perfect_gesture_start_time,
                cfg.GESTURE_DWELL_PERFECT_WRIST_S,
            )

            if perfect_triggered:

                tool_orient_mode_enabled = not tool_orient_mode_enabled
                perfect_gesture_start_time = None

                if tool_orient_mode_enabled:

                    robot.stop_motion()
                    time.sleep(0.08)
                    tool_orient_origin_x_px = float(hand_features["center_px"][0])
                    tool_orient_origin_y_px = float(hand_features["center_px"][1])
                    tool_orient_neutral_roll_deg = hand_features.get("palm_roll_deg", None)
                    tool_orient_pitch_filtered_speed = 0.0
                    tool_orient_side_filtered_speed = 0.0
                    tool_orient_roll_filtered_speed = 0.0
                    wrist_mode_enabled = False
                    wrist_filtered_speed = 0.0
                    interface.reset()
                    last_state = None
                    last_free_control_mode = None
                    active_base_gesture = None

                    print("")
                    print("MODO ORIENTACIÓN HERRAMIENTA ACTIVADO CON GESTO PERFECTO")
                    print("Mano arriba/abajo      -> inclina herramienta")
                    print("Mano izquierda/derecha -> orienta lateral")
                    print("Inclinar palma         -> gira herramienta")
                    print("")

                else:

                    robot.stop_speed()
                    time.sleep(0.08)
                    robot_pose = robot.get_tcp_pose()
                    mapper.calibrate_origin(
                        hand_features=hand_features,
                        robot_pose=robot_pose,
                    )
                    ema.reset([0.0, 0.0, 0.0])
                    freeze.reset([0.0, 0.0, 0.0])
                    last_target_pose = robot_pose.copy()
                    last_state = None
                    last_free_control_mode = None
                    tool_orient_origin_x_px = None
                    tool_orient_origin_y_px = None
                    tool_orient_neutral_roll_deg = None
                    tool_orient_pitch_filtered_speed = 0.0
                    tool_orient_side_filtered_speed = 0.0
                    tool_orient_roll_filtered_speed = 0.0

                    print("")
                    print("MODO ORIENTACIÓN HERRAMIENTA DESACTIVADO CON GESTO PERFECTO")
                    print("Recalibración suave al volver al control normal.")
                    print("")


            # --------------------------------------
            # 14. PUÑO -> INTERRUPTOR GARRA/VENTOSA
            # --------------------------------------
            fist_is_active = (
                hand_features is not None
                and calibrated
                and not manual_stop_latched
                and hand_features.get("fist_gesture", False)
            )

            fist_triggered, fist_gesture_start_time = gesture_held_since(
                fist_is_active and gripper_fist_armed,
                fist_gesture_start_time,
                cfg.GESTURE_DWELL_FIST_S,
            )

            if cfg.ENABLE_GRIPPER_OUTPUT and robot is not None:

                if fist_triggered:

                    gripper_closed = not gripper_closed

                    output_value = (
                        cfg.GRIPPER_CLOSE_VALUE
                        if gripper_closed
                        else not cfg.GRIPPER_CLOSE_VALUE
                    )

                    robot.set_digital_output(
                        cfg.GRIPPER_OUTPUT_ID,
                        output_value,
                    )

                    gripper_fist_armed = False
                    fist_gesture_start_time = None

                    print("")

                    if gripper_closed:
                        print("GARRA ACTIVADA POR PUÑO")

                    else:
                        print("GARRA DESACTIVADA POR PUÑO")

                    print("")

                if not fist_is_active:

                    gripper_fist_armed = True
                    fist_gesture_start_time = None


            # ----------------------------------
            # 15. EJECUCIÓN DEL MODO MUÑECA 'W'
            # ----------------------------------
            if wrist_mode_enabled:

                current_roll_deg = hand_features.get("palm_roll_deg", None)

                if wrist_neutral_roll_deg is None or current_roll_deg is None:

                    robot.stop_speed()
                    wrist_filtered_speed = 0.0

                    cv2.putText(
                        frame,
                        "WRIST MODE - SIN ROLL VALIDO",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (0, 0, 255),
                        2,
                    )

                    cv2.imshow("Control híbrido UR3", frame)

                    continue

                roll_error_deg = normalize_angle_deg(
                    current_roll_deg - wrist_neutral_roll_deg
                )

                raw_speed = roll_error_to_wrist_speed(roll_error_deg)

                wrist_filtered_speed = (
                    cfg.WRIST_SPEED_EMA_ALPHA * raw_speed
                    + (1.0 - cfg.WRIST_SPEED_EMA_ALPHA) * wrist_filtered_speed
                )

                robot.speed_joint(
                    joint_index=cfg.WRIST_JOINT_INDEX,
                    joint_speed_rad_s=wrist_filtered_speed,
                    accel_rad_s2=cfg.WRIST_SPEEDJ_ACCEL_RAD_S2,
                    time_s=cfg.WRIST_SPEEDJ_TIME_S,
                )

                cv2.putText(
                    frame,
                    "WRIST MODE ACTIVO - pulsa W para salir",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.72,
                    (255, 255, 0),
                    2,
                )

                cv2.putText(
                    frame,
                    f"roll error = {roll_error_deg:.1f} deg",
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 0),
                    2,
                )

                cv2.putText(
                    frame,
                    f"q[5] speed = {wrist_filtered_speed:.3f} rad/s",
                    (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 0),
                    2,
                )

                cv2.imshow("Control híbrido UR3", frame)

                continue


            # ---------------------------------------
            # 15. EJECUCIÓN DEL MODO ORIENTACIÓN 'O'
            # ---------------------------------------
            if tool_orient_mode_enabled:

                # Si por algún motivo se ha activado el modo O sin guardar origen,
                # detenemos velocidades y mostramos aviso.
                if (
                    tool_orient_origin_x_px is None
                    or tool_orient_origin_y_px is None
                ):

                    robot.stop_speed()
                    tool_orient_pitch_filtered_speed = 0.0
                    tool_orient_side_filtered_speed = 0.0
                    tool_orient_roll_filtered_speed = 0.0
                    tool_roll_center_since = None
                    tool_roll_center_progress = 0.0
                    tool_roll_center_enabled = False

                    cv2.putText(
                        frame,
                        "TOOL ORIENT MODE - SIN ORIGEN",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (0, 0, 255),
                        2,
                    )

                    cv2.imshow("Control híbrido UR3", frame)
                    continue

                # Punto fijo donde estaba la mano al entrar en modo O.
                # Las flechas y el círculo central se dibujan alrededor de este punto.
                anchor_px = [
                    float(tool_orient_origin_x_px),
                    float(tool_orient_origin_y_px),
                ]

                # ---------------------------------------------------
                # 15.1. DWELL TIME DE LAS FLECHAS DEL MODO O
                # ---------------------------------------------------
                arrow_candidate_now = get_tool_arrow_candidate(
                    center_px=hand_features["center_px"],
                    anchor_px=anchor_px,
                )

                now_arrow = time.time()

                if arrow_candidate_now == "NONE":
                    tool_arrow_candidate = "NONE"
                    tool_arrow_active_command = "NONE"
                    tool_arrow_candidate_since = None
                    tool_arrow_progress = 0.0

                else:
                    if arrow_candidate_now != tool_arrow_candidate:
                        tool_arrow_candidate = arrow_candidate_now
                        tool_arrow_candidate_since = now_arrow
                        tool_arrow_active_command = "NONE"
                        tool_arrow_progress = 0.0

                    else:
                        elapsed = now_arrow - tool_arrow_candidate_since

                        tool_arrow_progress = min(
                            elapsed / cfg.TOOL_ARROW_DWELL_TIME_S,
                            1.0,
                        )

                        if elapsed >= cfg.TOOL_ARROW_DWELL_TIME_S:
                            tool_arrow_active_command = tool_arrow_candidate

                # ---------------------------------------------------
                # 15.2. VELOCIDAD PITCH/SIDE SEGÚN FLECHA ACTIVA
                # ---------------------------------------------------
                raw_pitch_speed = 0.0
                raw_side_speed = 0.0

                # Flechas invertidas respecto a la versión anterior.
                if tool_arrow_active_command == "TOOL_UP":
                    raw_pitch_speed = (
                        cfg.TOOL_ORIENT_PITCH_MAX_SPEED_RAD_S
                        * cfg.TOOL_ORIENT_PITCH_SIGN
                    )

                elif tool_arrow_active_command == "TOOL_DOWN":
                    raw_pitch_speed = (
                        -cfg.TOOL_ORIENT_PITCH_MAX_SPEED_RAD_S
                        * cfg.TOOL_ORIENT_PITCH_SIGN
                    )

                elif tool_arrow_active_command == "TOOL_LEFT":
                    raw_side_speed = (
                        cfg.TOOL_ORIENT_SIDE_MAX_SPEED_RAD_S
                        * cfg.TOOL_ORIENT_SIDE_SIGN
                    )

                elif tool_arrow_active_command == "TOOL_RIGHT":
                    raw_side_speed = (
                        -cfg.TOOL_ORIENT_SIDE_MAX_SPEED_RAD_S
                        * cfg.TOOL_ORIENT_SIDE_SIGN
                    )

                # ---------------------------------------------------
                # 15.3. DWELL TIME DEL CÍRCULO CENTRAL PARA ROLL
                # ---------------------------------------------------
                roll_center_active = is_tool_roll_center_active(
                    center_px=hand_features["center_px"],
                    anchor_px=anchor_px,
                )

                now_roll_center = time.time()

                if roll_center_active:
                    if tool_roll_center_since is None:
                        tool_roll_center_since = now_roll_center
                        tool_roll_center_progress = 0.0
                        tool_roll_center_enabled = False
                    else:
                        elapsed_roll_center = now_roll_center - tool_roll_center_since

                        tool_roll_center_progress = min(
                            elapsed_roll_center / cfg.TOOL_ROLL_CENTER_DWELL_TIME_S,
                            1.0,
                        )

                        if elapsed_roll_center >= cfg.TOOL_ROLL_CENTER_DWELL_TIME_S:
                            tool_roll_center_enabled = True

                else:
                    tool_roll_center_since = None
                    tool_roll_center_progress = 0.0
                    tool_roll_center_enabled = False

                # ---------------------------------------------------
                # 15.4. VELOCIDAD ROLL SOLO SI EL CÍRCULO ESTÁ HABILITADO
                # ---------------------------------------------------
                current_roll_deg = hand_features.get("palm_roll_deg", None)

                if (
                    not roll_center_active
                    or not tool_roll_center_enabled
                    or tool_orient_neutral_roll_deg is None
                    or current_roll_deg is None
                ):
                    raw_roll_speed = 0.0
                    roll_error_deg = 0.0

                else:
                    roll_error_deg = normalize_angle_deg(
                        current_roll_deg - tool_orient_neutral_roll_deg
                    )

                    # Zona muerta reforzada para evitar que el robot copie
                    # pequeñas oscilaciones de MediaPipe con la mano quieta.
                    if abs(roll_error_deg) < cfg.TOOL_ORIENT_ROLL_DEADZONE_DEG:
                        raw_roll_speed = 0.0
                        roll_error_deg = 0.0
                    else:
                        raw_roll_speed = roll_error_to_tool_roll_speed(roll_error_deg)

                # ---------------------------------------------------
                # 15.5. FILTRADO DE VELOCIDADES
                # ---------------------------------------------------
                alpha = cfg.TOOL_ORIENT_SPEED_EMA_ALPHA

                if raw_pitch_speed == 0.0:
                    tool_orient_pitch_filtered_speed = 0.0
                else:
                    tool_orient_pitch_filtered_speed = (
                        alpha * raw_pitch_speed
                        + (1.0 - alpha) * tool_orient_pitch_filtered_speed
                    )

                if raw_side_speed == 0.0:
                    tool_orient_side_filtered_speed = 0.0
                else:
                    tool_orient_side_filtered_speed = (
                        alpha * raw_side_speed
                        + (1.0 - alpha) * tool_orient_side_filtered_speed
                    )

                if raw_roll_speed == 0.0:
                    tool_orient_roll_filtered_speed = 0.0
                else:
                    tool_orient_roll_filtered_speed = (
                        alpha * raw_roll_speed
                        + (1.0 - alpha) * tool_orient_roll_filtered_speed
                    )

                # ---------------------------------------------------
                # 15.6. ENVÍO DE VELOCIDADES ARTICULARES AL ROBOT
                # ---------------------------------------------------
                qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                qd[int(cfg.TOOL_ORIENT_PITCH_JOINT_INDEX)] = tool_orient_pitch_filtered_speed
                qd[int(cfg.TOOL_ORIENT_SIDE_JOINT_INDEX)] = tool_orient_side_filtered_speed
                qd[int(cfg.TOOL_ORIENT_ROLL_JOINT_INDEX)] = tool_orient_roll_filtered_speed

                all_zero = (
                    tool_orient_pitch_filtered_speed == 0.0
                    and tool_orient_side_filtered_speed == 0.0
                    and tool_orient_roll_filtered_speed == 0.0
                )

                if all_zero:
                    robot.stop_speed()
                else:
                    robot.speed_joints(
                        joint_speeds_rad_s=qd,
                        accel_rad_s2=cfg.TOOL_ORIENT_ACCEL_RAD_S2,
                        time_s=cfg.TOOL_ORIENT_TIME_S,
                    )

                # ---------------------------------------------------
                # 15.7. DIBUJO DE LA INTERFAZ DEL MODO O
                # ---------------------------------------------------
                draw_tool_orient_overlay(
                    frame,
                    anchor_px=anchor_px,
                    candidate_command=tool_arrow_candidate,
                    active_command=tool_arrow_active_command,
                    progress=tool_arrow_progress,
                )

                cv2.putText(
                    frame,
                    "TOOL ORIENT MODE - flechas + roll con dwell en centro",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.64,
                    (255, 255, 0),
                    2,
                )

                cv2.putText(
                    frame,
                    (
                        f"ARROW={tool_arrow_active_command} | "
                        f"CENTER={roll_center_active} "
                        f"{tool_roll_center_progress * 100:.0f}% | "
                        f"ROLL_EN={tool_roll_center_enabled} | "
                        f"roll={roll_error_deg:.1f}deg"
                    ),
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (255, 255, 0),
                    2,
                )

                cv2.imshow("Control híbrido UR3", frame)
                continue


            # ----------------------------
            # 16. INTERFAZ VIRTUAL NORMAL
            # -----------------------------
            state, command, progress = interface.update(center_px)


            # -------------------------------
            # 17. TRANSICIONES ENTRE ESTADOS
            # -------------------------------
            if state != last_state:
                previous_state = last_state

                if robot is not None:

                    if previous_state == VirtualInterface.BUTTON_ACTIVE:
                        robot.stop_speed()
                        time.sleep(0.05)

                    elif previous_state == VirtualInterface.FREE:

                        if last_free_control_mode == "GESTURE":
                            robot.stop_joint_motion(cfg.GESTURE_STOPJ_ACCEL_RAD_S2)

                        elif last_free_control_mode == "WRIST":
                            robot.stop_joint_motion(cfg.WRIST_STOPJ_ACCEL_RAD_S2)

                        else:
                            robot.stop_servo()

                        time.sleep(0.05)

                    elif previous_state == VirtualInterface.WAITING_BUTTON:
                        pass

                    else:
                        robot.stop_speed()
                        time.sleep(0.05)

                if (
                    state == VirtualInterface.FREE
                    and previous_state
                    in [
                        VirtualInterface.WAITING_BUTTON,
                        VirtualInterface.BUTTON_ACTIVE,
                    ]
                    and hand_features is not None
                    and robot is not None
                ):

                    robot_pose = robot.get_tcp_pose()

                    mapper.calibrate_origin(
                        hand_features=hand_features,
                        robot_pose=robot_pose,
                    )

                    ema.reset([0.0, 0.0, 0.0])
                    freeze.reset([0.0, 0.0, 0.0])
                    last_target_pose = robot_pose.copy()
                    last_free_control_mode = None
                    neutral_roll_deg = hand_features.get("palm_roll_deg", None)
                    active_wrist_gesture = None

                    print("Recalibración suave al volver a FREE.")

                last_state = state

            if command != last_command_printed:
                print(f"STATE={state} | CMD={command}")
                last_command_printed = command


            # ------------------------
            # 17. BOTÓN/PANEL ACTIVA
            # ------------------------
            if state == VirtualInterface.BUTTON_ACTIVE:

                last_free_control_mode = None

                safety_limit_reached = (
                    cfg.ENABLE_SOFT_JOINT_LIMIT
                    and robot.is_soft_joint_limit_reached(
                        joint_index=cfg.SAFETY_JOINT_INDEX,
                        limit_rad=cfg.SAFETY_JOINT_LIMIT_RAD,
                        limit_type=cfg.SAFETY_JOINT_LIMIT_TYPE,
                        margin_rad=cfg.SAFETY_JOINT_MARGIN_RAD,
                    )
                )

                if safety_limit_reached:
                    safety_recovery_mode = True

                    if not safety_stop_sent:
                        robot.stop_speed()
                        safety_stop_sent = True
                        print("Límite articular alcanzado en panel. Movimiento detenido una vez.")

                    cv2.putText(
                        frame,
                        "SOFT LIMIT - PANEL DETENIDO",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.70,
                        (0, 0, 255),
                        2,
                    )

                else:

                    safety_stop_sent = False
                    send_button_command(robot, command)

                    cv2.putText(
                        frame,
                        f"BUTTON ACTIVE: {command}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (0, 255, 0),
                        2,
                    )


            # ------------------------------
            # 18. ESPERANDO DWELL DEL BOTÓN
            # ------------------------------
            elif state == VirtualInterface.WAITING_BUTTON:

                last_free_control_mode = None

                cv2.putText(
                    frame,
                    f"WAITING BUTTON: {interface.current_candidate} | {progress * 100:.0f}%",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 165, 255),
                    2,
                )


            # --------------
            # 19. MODO FREE
            # --------------
            elif state == VirtualInterface.FREE:

                rotation_gesture = hand_features.get("rotation_gesture", "NONE")

                wrist_gesture = get_wrist_roll_gesture(
                    hand_features=hand_features,
                    neutral_roll_deg=neutral_roll_deg,
                )

                if rotation_gesture == "BASE_LEFT":

                    if last_free_control_mode != "GESTURE" or active_base_gesture != "BASE_LEFT":

                        robot.stop_motion()
                        time.sleep(0.08)

                        robot.move_base_increment(
                            delta_q0_rad=-cfg.GESTURE_BASE_LONG_STEP_RAD,
                            speed_rad_s=cfg.GESTURE_MOVEJ_SPEED_RAD_S,
                            accel_rad_s2=cfg.GESTURE_MOVEJ_ACCEL_RAD_S2,
                            asynchronous=True,
                        )

                        active_base_gesture = "BASE_LEFT"
                        last_free_control_mode = "GESTURE"

                    cv2.putText(
                        frame,
                        "GESTURE BASE LEFT async moveJ",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.70,
                        (0, 255, 255),
                        2,
                    )

                elif rotation_gesture == "BASE_RIGHT":

                    if last_free_control_mode != "GESTURE" or active_base_gesture != "BASE_RIGHT":

                        robot.stop_motion()
                        time.sleep(0.08)

                        robot.move_base_increment(
                            delta_q0_rad=cfg.GESTURE_BASE_LONG_STEP_RAD,
                            speed_rad_s=cfg.GESTURE_MOVEJ_SPEED_RAD_S,
                            accel_rad_s2=cfg.GESTURE_MOVEJ_ACCEL_RAD_S2,
                            asynchronous=True,
                        )

                        active_base_gesture = "BASE_RIGHT"
                        last_free_control_mode = "GESTURE"

                    cv2.putText(
                        frame,
                        "GESTURE BASE RIGHT async moveJ",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.70,
                        (0, 255, 255),
                        2,
                    )

                elif rotation_gesture == "STOP":

                    if last_free_control_mode == "GESTURE":
                        robot.stop_joint_motion(cfg.GESTURE_STOPJ_ACCEL_RAD_S2)

                    else:
                        robot.stop_motion()

                    active_base_gesture = None
                    last_free_control_mode = "GESTURE"

                    cv2.putText(
                        frame,
                        "GESTURE STOP",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.70,
                        (0, 0, 255),
                        2,
                    )

                else:

                    if (
                        last_free_control_mode in ["GESTURE", "WRIST"]
                        and robot is not None
                        and hand_features is not None
                    ):

                        if last_free_control_mode == "WRIST":
                            robot.stop_joint_motion(cfg.WRIST_STOPJ_ACCEL_RAD_S2)

                        else:
                            robot.stop_joint_motion(cfg.GESTURE_STOPJ_ACCEL_RAD_S2)

                        time.sleep(0.12)
                        robot_pose = robot.get_tcp_pose()

                        active_base_gesture = None
                        active_wrist_gesture = None

                        neutral_roll_deg = hand_features.get("palm_roll_deg", neutral_roll_deg)

                        mapper.calibrate_origin(
                            hand_features=hand_features,
                            robot_pose=robot_pose,
                        )

                        ema.reset([0.0, 0.0, 0.0])
                        freeze.reset([0.0, 0.0, 0.0])

                        last_target_pose = robot_pose.copy()

                        print("Recalibración suave al volver a servoL.")

                        robot_pose = robot.get_tcp_pose()
                        active_base_gesture = None

                        mapper.calibrate_origin(
                            hand_features=hand_features,
                            robot_pose=robot_pose,
                        )

                        ema.reset([0.0, 0.0, 0.0])
                        freeze.reset([0.0, 0.0, 0.0])

                        last_target_pose = robot_pose.copy()

                        print("Recalibración suave al soltar gesto.")

                    last_free_control_mode = "SERVO"
                    raw_offset = mapper.hand_features_to_robot_offset_m(hand_features)
                    smooth_offset = ema.update(raw_offset)
                    filtered_offset = freeze.update(smooth_offset)

                    safety_limit_reached = (
                        cfg.ENABLE_SOFT_JOINT_LIMIT
                        and robot.is_soft_joint_limit_reached(
                            joint_index=cfg.SAFETY_JOINT_INDEX,
                            limit_rad=cfg.SAFETY_JOINT_LIMIT_RAD,
                            limit_type=cfg.SAFETY_JOINT_LIMIT_TYPE,
                            margin_rad=cfg.SAFETY_JOINT_MARGIN_RAD,
                        )

                    )

                    if safety_limit_reached:

                        safety_recovery_mode = True

                        if not safety_stop_sent:

                            robot.stop_servo()
                            safety_stop_sent = True

                            print("Límite articular alcanzado en FREE. Recalibrando en posición actual.")

                            robot_pose = robot.get_tcp_pose()

                            mapper.calibrate_origin(
                                hand_features=hand_features,
                                robot_pose=robot_pose,
                            )

                            ema.reset([0.0, 0.0, 0.0])
                            freeze.reset([0.0, 0.0, 0.0])

                            last_target_pose = robot_pose.copy()

                        cv2.putText(
                            frame,
                            "SOFT LIMIT ACTIVO - FREE RECALIBRADO",
                            (20, 75),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            (0, 0, 255),
                            2,
                        )

                    else:

                        if safety_recovery_mode:

                            robot_pose = robot.get_tcp_pose()

                            mapper.calibrate_origin(
                                hand_features=hand_features,
                                robot_pose=robot_pose,
                            )

                            ema.reset([0.0, 0.0, 0.0])
                            freeze.reset([0.0, 0.0, 0.0])

                            last_target_pose = robot_pose.copy()

                            print("Robot fuera del límite. Recuperación finalizada y FREE recalibrado.")

                        safety_stop_sent = False
                        safety_recovery_mode = False

                    target_pose_raw = mapper.target_pose_from_offset(filtered_offset)

                    if last_target_pose is None:
                        last_target_pose = robot.get_tcp_pose()

                    target_pose = limit_target_step(
                        previous_pose=last_target_pose,
                        target_pose_raw=target_pose_raw,
                        max_step_m=cfg.MAX_TARGET_STEP_M,
                    )

                    last_target_pose = target_pose.copy()

                    robot.servo_to_pose(target_pose)

                    cv2.putText(
                        frame,
                        (
                            f"FREE servoL | "
                            f"X={filtered_offset[0] * 100:.1f}cm "
                            f"Y={filtered_offset[1] * 100:.1f}cm "
                            f"Z={filtered_offset[2] * 100:.1f}cm"
                        ),
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (0, 255, 255),
                        2,
                    )

            else:

                last_free_control_mode = None

                if robot is not None:
                    robot.stop_motion()

            interface.draw(
                frame,
                center_px=center_px,
                state=state,
                command=command,
                progress=progress,
            )

            cv2.imshow("Control híbrido UR3", frame)


    # --------------
    # CIERRE SEGURO
    # --------------
    finally:

        if robot is not None:

            if cfg.ENABLE_GRIPPER_OUTPUT and cfg.GRIPPER_OPEN_ON_STOP:

                robot.set_digital_output(
                    cfg.GRIPPER_OUTPUT_ID,
                    not cfg.GRIPPER_CLOSE_VALUE,
                )

        tracker.release()

        cv2.destroyAllWindows()

        if robot is not None:
            robot.disconnect()

        print("Programa finalizado de forma segura.")


# -----------------
# PUNTO DE ENTRADA
# -----------------
if __name__ == "__main__":

    main()