"""
ARCHIVO: simple_trajectory_backup.py
-------------------------------------------------------------------------------------------

RESUMEN:
Este archivo es una versión simple o de respaldo del control por seguimiento de mano.

A diferencia de main_hybrid_control.py, este script no incluye:
    - interfaz virtual;
    - botones en pantalla;
    - gesto de PAZ;
    - gesto de puño;
    - modo orientación de herramienta;
    - control de base por dedos;
    - parada por gesto grosero;
    - HOME global.

Su objetivo es mucho más básico:

    1. Leer la cámara.
    2. Detectar la mano con MediaPipe.
    3. Calibrar un origen con la tecla 'c'.
    4. Convertir el movimiento de la mano en movimiento del robot.
    5. Suavizar la señal con filtros.
    6. Enviar poses objetivo al robot mediante servoL.

Es un archivo útil como backup porque permite comprobar que el seguimiento básico
mano -> robot funciona sin toda la lógica avanzada del programa principal.
"""

import time
import cv2
import config as cfg

from filters import EmaFilter, DeadbandFreezeFilter
from hand_tracker import HandTracker
from trajectory_mapper import TrajectoryMapper
from robot_controller import RobotController


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def limit_target_step(previous_pose, target_pose_raw, max_step_m):
    """
    Limita el salto máximo entre una pose objetivo y la siguiente.

    Esta función es una protección de suavidad.

    Aunque los filtros suavizan el movimiento de la mano, puede ocurrir que por ruido,
    pérdida momentánea de detección o una mala calibración, la pose objetivo cambie
    demasiado rápido.

    Para evitar que el robot reciba un salto brusco, esta función limita cuánto puede
    cambiar X, Y y Z en cada ciclo.

    Parámetros
    ----------
    -> previous_pose:
        Última pose objetivo enviada al robot.

        Formato:
            [x, y, z, rx, ry, rz]

    -> target_pose_raw:
        Nueva pose objetivo calculada por el mapper.

        Formato:
            [x, y, z, rx, ry, rz]

    -> max_step_m:
        Máximo cambio permitido por ciclo en metros para X, Y y Z.

    Retorna
    -------
    -> list
        Pose objetivo limitada.
    """

    # Convertimos las poses a listas normales de Python.
    # Esto evita problemas si vienen como arrays de NumPy u otro tipo iterable.
    previous_pose = list(previous_pose)
    target_pose_raw = list(target_pose_raw)

    # Empezamos copiando la pose anterior.
    #
    # Luego solo modificaremos X, Y y Z con el límite aplicado.
    target_pose = previous_pose.copy()

    # Recorremos las tres primeras componentes:
    #   0 -> X
    #   1 -> Y
    #   2 -> Z
    # No tocamos orientación todavía.
    for i in range(3):

        # Calculamos cuánto quiere cambiar la nueva pose respecto a la anterior.
        delta = target_pose_raw[i] - previous_pose[i]

        # Si el cambio positivo es demasiado grande, lo recortamos.
        if delta > max_step_m:
            delta = max_step_m

        # Si el cambio negativo es demasiado grande, lo recortamos también.
        elif delta < -max_step_m:
            delta = -max_step_m

        # Aplicamos el cambio limitado.
        target_pose[i] = previous_pose[i] + delta

    # La orientación se copia directamente de la pose calculada.
    # En este proyecto, normalmente la orientación se mantiene constante porque
    # trajectory_mapper.py copia rx, ry, rz desde la pose de origen.
    target_pose[3:6] = target_pose_raw[3:6]

    # Devolvemos la pose final limitada.
    return target_pose


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def main():
    """
    Función principal del programa.

    Aquí se crean todos los objetos principales y se ejecuta el bucle de control.

    Flujo general:

        1. Crear cámara/HandTracker.
        2. Crear RobotController.
        3. Crear TrajectoryMapper.
        4. Crear filtros.
        5. Entrar en bucle:
            - leer cámara;
            - detectar mano;
            - leer teclado;
            - calibrar con 'c';
            - si está calibrado, seguir la mano;
            - si se pierde la mano, parar robot;
            - mostrar imagen.
        6. Al salir, liberar recursos.
    """

    print("=== Seguimiento de trayectoria simple - BACKUP ===")
    print("")
    print("Controles:")
    print("  c -> calibrar origen mano/robot")
    print("  q -> salir")
    print("")
    print("DRY_RUN:", cfg.DRY_RUN)
    print("")

    # ---------------------------------------------------------------------------------------
    # 1. CREACIÓN DEL TRACKER DE MANO
    # ---------------------------------------------------------------------------------------
    # HandTracker se encarga de:
    #   - abrir la cámara;
    #   - leer frames;
    #   - detectar landmarks de la mano;
    #   - devolver hand_features.
    tracker = HandTracker(
        model_path=cfg.MODEL_PATH,
        camera_id=cfg.CAMERA_ID,
    )

    # ---------------------------------------------------------------------------------------
    # 2. CREACIÓN DEL CONTROLADOR DEL ROBOT
    # ---------------------------------------------------------------------------------------
    # RobotController encapsula toda la comunicación RTDE con el UR3e.
    # · Si cfg.DRY_RUN = True:
    #       no se conecta al robot real, solo imprime comandos.
    # · Si cfg.DRY_RUN = False:
    #       conecta con el robot real usando la IP configurada.
    robot = RobotController(
        host=cfg.ROBOT_HOST,
        dry_run=cfg.DRY_RUN,
        servo_acc=cfg.SERVO_ACC,
        servo_vel=cfg.SERVO_VEL,
        servo_period_s=cfg.CONTROL_PERIOD_S,
        lookahead_time=cfg.SERVO_LOOKAHEAD_TIME,
        gain=cfg.SERVO_GAIN,
    )

    # ---------------------------------------------------------------------------------------
    # 3. CREACIÓN DEL MAPPER MANO -> ROBOT
    # ---------------------------------------------------------------------------------------
    # TrajectoryMapper convierte:
    #   desplazamiento de mano en píxeles -> desplazamiento del robot en metros
    # Para funcionar necesita calibrarse primero con la tecla 'c'.
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

    # ---------------------------------------------------------------------------------------
    # 4. CREACIÓN DE FILTROS
    # ---------------------------------------------------------------------------------------
    # -> EmaFilter: suaviza la señal.
    # -> DeadbandFreezeFilter: elimina microtemblores y congela pequeños movimientos.
    # La cadena será:
    #   raw_offset -> ema -> freeze -> target_pose
    ema = EmaFilter(cfg.EMA_ALPHA)

    freeze = DeadbandFreezeFilter(
        deadband_m=cfg.DEADBAND_M,
        freeze_speed_m_s=cfg.FREEZE_SPEED_M_S,
        freeze_time_s=cfg.FREEZE_TIME_S,
    )

    # ---------------------------------------------------------------------------------------
    # 5. VARIABLES DE ESTADO
    # ---------------------------------------------------------------------------------------
    # Guarda el último instante en el que se detectó la mano.
    # Sirve para parar el robot si se pierde la mano durante demasiado tiempo.
    last_hand_seen = time.time()

    # Guarda la última pose objetivo enviada al robot.
    # Se usa en limit_target_step() para limitar saltos entre ciclos.
    last_target_pose = None

    try:
        # ===================================================================================
        # BUCLE PRINCIPAL
        # ===================================================================================
        # Este bucle se ejecuta continuamente hasta que el usuario pulsa 'q'.
        while True:

            # -------------------------------------------------------------------------------
            # 1. LEER CÁMARA Y DETECTAR MANO
            # -------------------------------------------------------------------------------
            # -> frame:
            #       imagen procesada para mostrar.
            # > hand_features:
            #       diccionario con datos de la mano si se detecta. None si no hay mano.
            frame, hand_features = tracker.read()

            # Tiempo actual del ciclo.
            now = time.time()

            # -------------------------------------------------------------------------------
            # 2. LEER TECLADO
            # -------------------------------------------------------------------------------
            # cv2.waitKey(1) espera 1 ms y devuelve la tecla pulsada en la ventana OpenCV.
            key = cv2.waitKey(1) & 0xFF

            # Salir del programa con 'q'.
            if key == ord("q"):
                break

            # -------------------------------------------------------------------------------
            # 3. CASO: HAY MANO DETECTADA
            # -------------------------------------------------------------------------------
            if hand_features is not None:

                # Actualizamos el instante de última mano detectada.
                last_hand_seen = now

                # ---------------------------------------------------------------------------
                # 3.1. CALIBRACIÓN CON TECLA C
                # ---------------------------------------------------------------------------
                # Al pulsar 'c', se guarda:
                #   - posición actual de la mano;
                #   - pose actual del robot.
                # A partir de ahí, el movimiento será relativo a ese origen.
                if key == ord("c"):

                    # Leemos pose actual del robot.
                    robot_pose = robot.get_tcp_pose()

                    # Guardamos el origen mano/robot en el mapper.
                    mapper.calibrate_origin(
                        hand_features=hand_features,
                        robot_pose=robot_pose,
                    )

                    # Reiniciamos filtros a cero.
                    # Esto evita que arrastren valores antiguos de una calibración previa.
                    ema.reset([0.0, 0.0, 0.0])
                    freeze.reset([0.0, 0.0, 0.0])

                    # La última pose objetivo pasa a ser la pose actual del robot.
                    # Así limit_target_step() empieza desde una referencia coherente.
                    last_target_pose = robot_pose.copy()

                    print("")
                    print("Calibración realizada.")
                    print("Origen mano px:", hand_features["center_px"])
                    print("Z MediaPipe origen:", round(hand_features["palm_z_norm"], 6))
                    print("Origen robot:", [round(float(v), 4) for v in robot_pose])
                    print("")

                # ---------------------------------------------------------------------------
                # 3.2. SEGUIMIENTO SI EL MAPPER ESTÁ CALIBRADO
                # ---------------------------------------------------------------------------
                # mapper.is_calibrated es True cuando ya se ha pulsado 'c' correctamente.
                if mapper.is_calibrated:

                    # -----------------------------------------------------------------------
                    # A. CONVERTIR MANO EN OFFSET DEL ROBOT
                    # -----------------------------------------------------------------------
                    # raw_offset es el desplazamiento bruto en metros: [dx, dy, dz]
                    # calculado a partir del desplazamiento de la mano en píxeles.
                    raw_offset = mapper.hand_features_to_robot_offset_m(hand_features)

                    # -----------------------------------------------------------------------
                    # B. SUAVIZAR OFFSET
                    # -----------------------------------------------------------------------
                    # EMA reduce cambios bruscos entre frames.
                    smooth_offset = ema.update(raw_offset)

                    # -----------------------------------------------------------------------
                    # C. ELIMINAR MICROTEMBLORES
                    # -----------------------------------------------------------------------
                    # freeze elimina movimientos pequeños debidos al ruido de visión.
                    filtered_offset = freeze.update(smooth_offset)

                    # -----------------------------------------------------------------------
                    # D. CREAR POSE OBJETIVO BRUTA
                    # -----------------------------------------------------------------------
                    # El mapper suma el offset filtrado a la pose de origen del robot.
                    # Resultado: target_pose_raw = [x, y, z, rx, ry, rz]
                    target_pose_raw = mapper.target_pose_from_offset(filtered_offset)

                    # -----------------------------------------------------------------------
                    # E. INICIALIZAR last_target_pose SI ES NECESARIO
                    # -----------------------------------------------------------------------
                    # Esto puede ocurrir si el valor no se había inicializado todavía.
                    if last_target_pose is None:
                        last_target_pose = mapper.robot_origin_pose.tolist()

                    # -----------------------------------------------------------------------
                    # F. LIMITAR SALTO ENTRE POSES
                    # -----------------------------------------------------------------------
                    # Aunque el offset ya está filtrado, limitamos el cambio máximo por ciclo.
                    # Esto evita que el robot reciba una referencia demasiado lejana de golpe.
                    target_pose = limit_target_step(
                        previous_pose=last_target_pose,
                        target_pose_raw=target_pose_raw,
                        max_step_m=cfg.MAX_TARGET_STEP_M,
                    )

                    # Guardamos la pose objetivo actual para el siguiente ciclo.
                    last_target_pose = target_pose.copy()

                    # -----------------------------------------------------------------------
                    # G. ENVIAR POSE AL ROBOT CON servoL
                    # -----------------------------------------------------------------------
                    # servoL hace seguimiento cartesiano continuo.
                    # Este script llama a servo_to_pose() en cada ciclo mientras la mano
                    # siga detectada y el mapper esté calibrado.
                    robot.servo_to_pose(target_pose)

                    # -----------------------------------------------------------------------
                    # H. MOSTRAR OFFSET EN PANTALLA
                    # -----------------------------------------------------------------------
                    # filtered_offset está en metros.
                    # Multiplicamos por 100 para mostrar centímetros.
                    cv2.putText(
                        frame,
                        (
                            f"X={filtered_offset[0] * 100:.1f} cm | "
                            f"Y={filtered_offset[1] * 100:.1f} cm | "
                            f"Z={filtered_offset[2] * 100:.1f} cm"
                        ),
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.70,
                        (0, 255, 255),
                        2,
                    )

                # ---------------------------------------------------------------------------
                # 3.3. HAY MANO PERO TODAVÍA NO SE HA CALIBRADO
                # ---------------------------------------------------------------------------
                else:
                    cv2.putText(
                        frame,
                        "Pulsa 'c' para calibrar",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (0, 255, 255),
                        2,
                    )

            # -------------------------------------------------------------------------------
            # 4. CASO: NO HAY MANO DETECTADA
            # -------------------------------------------------------------------------------
            else:

                # Si hace demasiado tiempo que no se ve la mano, paramos el robot.
                # Esto evita que el robot siga moviéndose con la última referencia recibida
                # cuando el usuario ha sacado la mano de la imagen.
                if now - last_hand_seen > cfg.HAND_LOST_TIMEOUT_S:
                    robot.stop_motion()

                # Mostramos aviso visual.
                cv2.putText(
                    frame,
                    "MANO NO DETECTADA",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2,
                )

            # -------------------------------------------------------------------------------
            # 5. MOSTRAR IMAGEN
            # -------------------------------------------------------------------------------
            cv2.imshow("Seguimiento simple - BACKUP", frame)

    # =======================================================================================
    # CIERRE SEGURO
    # =======================================================================================
    # finally se ejecuta siempre:
    #   - si pulsas q;
    #   - si hay un error;
    #   - si se interrumpe el programa.
    finally:

        # Liberamos cámara.
        tracker.release()

        # Cerramos ventanas de OpenCV.
        cv2.destroyAllWindows()

        # Desconectamos robot.
        # RobotController.disconnect() ya intenta parar movimiento y cerrar RTDE.
        robot.disconnect()

        print("Programa finalizado de forma segura.")


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
if __name__ == "__main__":
    """
    Punto de entrada del script.

    Esta condición significa:

        "Ejecuta main() solo si este archivo se lanza directamente."

    Si este archivo se importase desde otro módulo, main() no se ejecutaría
    automáticamente.
    """

    main()