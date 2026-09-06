"""
ARCHIVO: hand_tracker.py
-------------------------------------------------------------------------------------------

RESUMEN:
Este módulo se encarga de toda la parte de visión artificial del proyecto.

Sus responsabilidades principales son:

    1. Abrir la cámara.
    2. Leer frames en tiempo real.
    3. Detectar la mano con MediaPipe Hands.
    4. Extraer los landmarks de la mano.
    5. Calcular características útiles:
        - centro de la palma;
        - ancho aproximado de la palma;
        - profundidad MediaPipe;
        - roll o inclinación de la mano;
        - dedos levantados/bajados;
        - gestos especiales.
    6. Dibujar sobre la imagen:
        - esqueleto de la mano;
        - centro de la palma;
        - panel de gesto detectado;
        - ejes visuales de referencia.
    7. Devolver al programa principal:
        - frame procesado;
        - diccionario hand_features con toda la información útil.

Este archivo NO mueve el robot directamente.
Solo interpreta la mano y entrega información al resto del sistema.

El movimiento real se decide después en main_hybrid_control.py.
"""

import cv2
import math
import mediapipe as mp
import numpy as np
import config as cfg

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# Landmarks usados para representar la palma
# MediaPipe devuelve 21 landmarks numerados de 0 a 20, en este proyecto usamos 5:
#   0  -> muñeca
#   5  -> base del dedo índice
#   9  -> base del dedo medio
#   13 -> base del dedo anular
#   17 -> base del meñique
PALM_IDS = [0, 5, 9, 13, 17]




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def landmark_distance_2d(a, b):
    """
    Calcula la distancia 2D entre dos landmarks de MediaPipe
    
    MediaPipe da las coordenadas normalizadas:
        · x-> posición horizontal entre 0 y 1
        · y-> posición vertical entre 0 y 1

    Esta función calcula la distancia en ese espacio normalizado

    Parámetros
    ----------
    -> a, b:
        Landmarks de MediaPipe

    Retorna
    -------
    -> float:
        Distancia euclídea entre ambos puntos
    """

    # Diferencia horizontal entre los dos landmarks
    dx = a.x - b.x      

    # Diferencia vertical entre los dos landmarks
    dy = a.y - b.y

    # Se eleva a 0.5 en lugar de usar math.sqrt, pero es equivalente
    return float((dx * dx + dy * dy) ** 0.5)




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def finger_is_up(hand, tip_id, pip_id, margin=0.025):
    """
    Determina si el dedo está levantado

    En la imagen, el eje Y crece hacia abajo:
    - 'y' pequeño -> punto más arriba
    - 'y' grande -> punto más abajo

    Por lo tanto, para saber si un dedo está levantado, comprobamos si la punta del dedo
    está por encima de su articulación PIP (articulación interfalángica proximal)

    Parámetros
    ----------
    -> hand:
        Lista de 21 landmarks de la mano
    
    -> tip_id:
        Índice del landmark de la punta del dedo
    
    -> pip_id:
        Índice del landmark de la articulación PIP del dedo

    -> margin:
        Margen de seguridad para evitar falsos positivos

    Retorna
    -------
    -> bool
        True si la punta está lo suficiente por encima de la articulación
    """

    # Si la punta tiene coordenada 'y' menor que la articulación
    # significa que está más arriba en la imagen
    # Restamos margin para exigir claramente que esté arriba
    return hand[tip_id].y < hand[pip_id].y - margin




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def finger_is_down(hand, tip_id, pip_id, margin=0.025):
    """
    Determina si un dedo está bajado o recogido.

    Es la lógica contraria a finger_is_up(). 
    Como el eje Y crece hacia abajo, si la punta del dedo tiene una Y mayor
    que la articulación PIP, la punta está más abajo en la imagen.

    Parámetros
    ----------
    -> hand:
        Lista de landmarks de la mano.

    -> tip_id:
        Landmark de la punta del dedo.

    -> pip_id:
        Landmark de la articulación PIP.

    -> margin:
        Margen para evitar detecciones dudosas.

    Retorna
    -------
    -> bool
        True si el dedo está suficientemente bajado.
    """
    
    return hand[tip_id].y > hand[pip_id].y + margin




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def detect_extra_gestures(hand):
    """
    Detecta gestos especiales a partir de los landmarks de la mano.
    Esta función no mueve el robot. Solo interpreta la postura de la mano
    y devuelve variables booleanas

    Gestos principales detectados:
        - peace_gesture: 
            gesto de paz, usado para calibrar.

        - perfect_gesture:
            círculo pulgar-índice con otros dedos extendidos,
            usado para activar/desactivar el modo O.

        - fist_gesture:
            puño cerrado, usado como interruptor de garra/ventosa.

        - rude_gesture:
            gesto grosero, usado como parada manual visual.

        - ok_calibration_gesture:
            gesto OK genérico; se conserva como detección auxiliar.
    
    Parámetros
    ----------
    -> hand:
        Lista de 21 landmarks de MediaPipe.

    Retorna
    -------
    -> dict
        Diccionario con gestos y métricas auxiliares.
    """

    # -------------------------------------
    # REFERENCIAS DE LANDMARKS PRINCIPALES
    # -------------------------------------
    thumb_tip = hand[4]    
    thumb_ip = hand[3]      
    thumb_mcp = hand[2]     

    index_tip = hand[8]
    middle_tip = hand[12]
    ring_tip = hand[16]
    pinky_tip = hand[20]


    # ------------------------------
    # DETECCIÓN DE DEDOS LEVANTADOS
    # ------------------------------
    # Para cada dedo comparamos su punta con una articulación intermedia
    index_up = finger_is_up(hand, 8, 6)
    middle_up = finger_is_up(hand, 12, 10)
    ring_up = finger_is_up(hand, 16, 14)
    pinky_up = finger_is_up(hand, 20, 18)


    # ---------------------------
    # DETECCIÓN DE DEDOS BAJADOS
    # ---------------------------
    # Para cada dedo comparamos su punta con una articulación intermedia
    index_down = finger_is_down(hand, 8, 6)
    middle_down = finger_is_down(hand, 12, 10)
    ring_down = finger_is_down(hand, 16, 14)
    pinky_down = finger_is_down(hand, 20, 18)


    # --------------------------
    # DISTANCIA PULGAR - ÍNDICE
    # --------------------------
    # Esta distancia sirve para detectar gestos de tipo OK o PERFECTO
    # Si la punta del pular y la punta del índice están cerca, se interpreta en contacto
    thumb_index_dist = landmark_distance_2d(thumb_tip, index_tip)

    # Umbral empírico
    # Si la distancia entre pulgar e índice es menor que 0.055 en coordenada
    # normalizada, se considera que se están tocando
    thumb_index_touch = thumb_index_dist < 0.055


    # ------------------------------
    # CENTRO APROXIMADO DE LA PALMA
    # ------------------------------
    # Se calcula promediando:
    #   - muñeca
    #   - base del índice
    #   - base del medio
    #   - base del anular
    #   - base del meñique
    # Esto permite tener un punto de referencia para saber si las puntas de los
    # de los dedos están cerca o lejos de la palma 
    palm_cx = (hand[0].x + hand[5].x + hand[9].x + hand[13].x + hand[17].x) / 5.0
    palm_cy = (hand[0].y + hand[5].y + hand[9].y + hand[13].y + hand[17].y) / 5.0

    # Creamos un objeto simple con atributos 'x' e 'y' para poder reutilizar
    # landmark_distance_2d(), que espera objetos con '.x' y '.y'
    class Point:
        pass

    palm_center = Point()
    palm_center.x = palm_cx
    palm_center.y = palm_cy


    # ----------------------------------------------
    # TAMAÑO DE LA PALMA PARA NORMALIZAR DISTANCIAS
    # ----------------------------------------------
    # La distancia entre la base del índice y la base del meñique se usa como medida
    # aproximada del tamaño de la mano
    # Esto es importante porque una misma distancia absoluta no significa lo mismo si 
    # la mano está cerca o lejos de la cámara
    palm_size = landmark_distance_2d(hand[5], hand[17])

    # Protección para evitar divisiones por cero o valores demasiado pequeños
    if palm_size < 0.001:
        palm_size = 0.001


    # -----------------------------------
    # DISTANCIA DE CADA PUNTA A LA PALMA
    # -----------------------------------
    # Calculamos cuánto de cerca está cada punta del dedo respecto al centro de la palma
    # Dividimos por palm_size para normalizar
    # Resultado:
    #   - Valor bajo -> punta cerca de la palma
    #   - Valor alto -> punta lejos de la palma
    # Esto se usa especialmente para detectar el puño   
    index_tip_to_palm = landmark_distance_2d(index_tip, palm_center) / palm_size
    middle_tip_to_palm = landmark_distance_2d(middle_tip, palm_center) / palm_size
    ring_tip_to_palm = landmark_distance_2d(ring_tip, palm_center) / palm_size
    pinky_tip_to_palm = landmark_distance_2d(pinky_tip, palm_center) / palm_size
    thumb_tip_to_palm = landmark_distance_2d(thumb_tip, palm_center) / palm_size


    # -------------------------
    # CONTEO DE DEDOS CERRADOS
    # -------------------------
    # Cada bool se convierte a entero
    #   True -> 1
    #   False -> 0
    # Así podemos contar cuántos dedos están bajados
    closed_count = (

        int(index_down)
        + int(middle_down)
        + int(ring_down)
        + int(pinky_down)
    )


    # --------------
    # GESTO DE PAZ
    # --------------
    peace_gesture = (

        index_up
        and middle_up
        and ring_down
        and pinky_down
    )
           

    # ------------------------------------------
    # VARIABLES RESERVADAS PARA GESTO DE PULGAR
    # ------------------------------------------
    # Actualemente desactivadas para que no interfieran con la detección de otros gestos
    thumb_up_gesture = False
    thumb_down_gesture = False
    thumb_vertical_gesture = False


    # --------------
    # GESTO DE PUÑO
    # --------------
    # La detección del puño no se basa solo en que los dedos estén bajados
    # También exige que las puntas entén cerca de la palma
    # Esto evita confundir ciertas posturas intermedias con un puño real
    fist_gesture = (

        closed_count >= 3
        and index_tip_to_palm < 1.05
        and middle_tip_to_palm < 1.05
        and ring_tip_to_palm < 1.05
        and pinky_tip_to_palm < 1.05
        and not peace_gesture
    )


    # ------------------
    # GESTO DE PERFECTO
    # ------------------
    # Empleado en el proyecto para activar o desactivar el modo 'o'
    perfect_gesture = (

        thumb_index_touch
        and middle_up
        and ring_up
        and pinky_up
        and not fist_gesture
        and not thumb_vertical_gesture
    )


    # ------------------
    # GESTO OK AUXILIAR
    # ------------------
    # Detecta contacto pulgar-índice siempre que no se clasifique como PERFECTO
    # Actualmente en desuso
    ok_calibration_gesture = (

        thumb_index_touch
        and not perfect_gesture
        and not fist_gesture
        and not thumb_vertical_gesture
    )


    # --------------
    # GESTO GROSERO
    # --------------
    # Se usa como gesto de parada manual con aviso visual
    rude_gesture = (

        middle_up
        and index_down
        and ring_down
        and pinky_down
        and not perfect_gesture
        and not fist_gesture
        and not thumb_vertical_gesture
    )


    # ---------------------
    # SALIDA DE LA FUNCIÓN
    # ---------------------
    # Devolvemos todas las variables útiles en un diccionario
    # main_hybrid_control.py leerá estas claves para decidir qué modo activar o qué comando enviar
    
    return {

        "ok_calibration_gesture": ok_calibration_gesture,
        "perfect_gesture": perfect_gesture,
        "fist_gesture": fist_gesture,
        "peace_gesture": peace_gesture,
        "rude_gesture": rude_gesture,
        "thumb_index_dist": thumb_index_dist,
        "index_up": index_up,
        "middle_up": middle_up,
        "ring_up": ring_up,
        "pinky_up": pinky_up,
        "index_tip_to_palm": index_tip_to_palm,
        "middle_tip_to_palm": middle_tip_to_palm,
        "ring_tip_to_palm": ring_tip_to_palm,
        "pinky_tip_to_palm": pinky_tip_to_palm,
        "closed_count": closed_count,
    }




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def draw_hand_connections(frame, hand_landmarks):
    """
    Dibuja sobre el frame el esqueleto de la mano. 
    
    Esta función no afecta a la lógica de control. 
    Solo sirve para visualizar qué está detectando MediaPipe.

    Parámetros
    ----------
    -> frame:
        Imagen de OpenCV sobre la que se dibuja
    
    -> hand_landmarks:
        Lista de landmarks detectados por MediaPipe
    """

    # Obtenemos dimensiones de la imagen
    h, w, _ = frame.shape

    # Lista de conexiones manual entre landmarks
    # Cada tupla (a, b) indica que se debe dibujar una línea entre el punto 'a' y 'b'
    connections = [

        (0, 1), (1, 2), (2, 3), (3, 4),
        (0, 5), (5, 6), (6, 7), (7, 8),
        (5, 9), (9, 10), (10, 11), (11, 12),
        (9, 13), (13, 14), (14, 15), (15, 16),
        (13, 17), (17, 18), (18, 19), (19, 20),
        (0, 17),
    ]

    # Convertimos coordenadas normalizadas a px
    # MediaPipe da 'x' e 'y' entre 0 y 1
    # OpenCV necesita coordenadas de px
    pts = []

    for lm in hand_landmarks:

        x = int(lm.x * w)
        y = int(lm.y * h)
        pts.append((x, y))

    # Dibujamos lineas entre landmarks conectados
    for a, b in connections:
        cv2.line(frame, pts[a], pts[b], (0, 255, 0), 2)

    # Dibujamos cada landmark como un punto rojo
    for p in pts:
        cv2.circle(frame, p, 4, (0, 0, 255), -1)




# //////////////////////////////////////////////////////////////////////////////////////////////
class HandTracker:
    """
    Clase principal de seguimiento de la mano.

    Esta clase encapsula:
        - apertura de la cámara
        - creación del modelo MediaPipe
        - lectura de frames
        - extracción de carcaterísticas
        - dibujo de información visual
        - liberación de cámara
    
    main_hybrid_control.py usa esta clase de forma sencilla:
        tracker = HandTracker(...)
        frame, hand_features = tracker.read()
    
        Si hay mano:
            hand_features contiene un diccionario con datos relevantes
        Si no hay mano
            hand_features vale None
    """

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def __init__(self, model_path: str, camera_id: int = 0):
        """
        Constructor de HandTracker.

        Parámetros
        ----------
        -> model_path : str
            Ruta al modelo .task de MediaPipe

        -> camara_id : int
            Índice de la cámara.
            Normalmente:
                0 -> webcam principal 
        """
        # Abrimos la cámara con OpenCV
        self.cap = cv2.VideoCapture(camera_id)

        # Si la cámara no se abre correctamente, paramos el programa
        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara.")

        # Configuración base de MediaPipe
        # Aquí se indica qué modelo se va a cargar
        base_options = python.BaseOptions(model_asset_path=model_path)

        # Opciones del detector de manos
        options = vision.HandLandmarkerOptions(

            base_options=base_options,

            # VIDEO indica que se procesan frames consecutivos de un vídeo/cámara
            running_mode=vision.RunningMode.VIDEO,

            # Solo necesitamos detectar una mano
            num_hands=1,

            # Confianza mñinima para aceptar una detección inicial de la mano
            min_hand_detection_confidence=0.6,

            # Confianza mínima para aceptar que la mano sigue presente
            min_hand_presence_confidence=0.6,

            # Confianza mínima para el seguimiento de frames
            min_tracking_confidence=0.6,
        )

        # Creamos el detector de manos usando las opciones anteriores
        self.landmarker = vision.HandLandmarker.create_from_options(options)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def read(self):
        """
        Lee un frame de cámara y extrae características de la mano

        Retorna
        -------
        -> frame: 
            Imagen proccesada, con dibujos de landmarks, ejes y paneles
        
        -> hand_features:
            Diccionario con información de la mano si hay una detección
            Si no hay una mano detectada hand_features = None
        """

        # ---------------------
        # 1. LECTURA DE CÁMARA
        # ---------------------
        ok, frame = self.cap.read()

        # Si no se pueden leer imágenes, se detienen el programa
        if not ok:
            raise RuntimeError("No se pudo leer frame de la cámara.")

        # Si mueves la mano a la derecha, en pantalla se mueve a la derecha
        frame = cv2.flip(frame, 1)

        # Dimensiones del frame
        h, w, _ = frame.shape


        # --------------------------------------
        # 2. CONVERSIÓN DE COLOR PARA MEDIAPIPE
        # --------------------------------------
        # OpenCV trabaja por defecto en BGR
        # MediaPipe espera RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Creamos un objeto mp.Imagen compatible con MediaPipe Tasks
        mp_image = mp.Image(

            image_format=mp.ImageFormat.SRGB,
            data=rgb,
        )


        # ----------------------------
        # 3. TIMESTAMP PARA MEDIAPIPE
        # ----------------------------
        # En modo VIDEO, MediaPipe necesita un timestamp en milisegundos
        # cv2.getTickCount() devuelve ticks del reloj interno
        # cv2.getTickFrecuency() devuelve ticks por segundo
        # Al dividirlos obtenemos segundos, y multiplicamos por 1000 para los milisegundos
        timestamp_ms = int(

            cv2.getTickCount() / cv2.getTickFrequency() * 1000
        )


        # ---------------------
        # 4. DETECCIÓN DE MANO
        # ---------------------
        # detect_for_video procesa el frame actual y devuelve:
        #   - landmarks normalizados
        #   - landmarks en coordenadas world si están disponibles
        #   - información adicional
        result = self.landmarker.detect_for_video(

            mp_image,
            timestamp_ms,
        )

        # Por defecto asumimos que no hay mano 
        hand_features = None


        # -------------------------
        # 5. SI HAY MANO DETECTADA
        # -------------------------
        if result.hand_landmarks:

            # Tomamos la primera mano detectada
            # Como num_hands = 1, solo debería haber una
            hand = result.hand_landmarks[0]

            # Dibujamos el esqueleto de la mano sobre el frame
            draw_hand_connections(frame, hand)

            # ------------------------------------
            # 5.1. CÁLCULO DEL CENTRO DE LA PALMA
            # ------------------------------------
            # Extraemos las coordenadas x, y, z de los puntos de la palma
            xs = [hand[i].x for i in PALM_IDS]
            ys = [hand[i].y for i in PALM_IDS]
            zs = [hand[i].z for i in PALM_IDS]

            # Convertimos la media normalizada a píxeles
            cx = float(np.mean(xs) * w)
            cy = float(np.mean(ys) * h)

            # Guardamos el centro como array: [cx, cy]
            # Este punto se usa como referencia para mover el robot
            center_px = np.array([cx, cy], dtype=float)

            # Profundidad normalizada de la palma
            # En la versión final no se usa para mover 'Y', pero queda disponible
            palm_z_norm = float(np.mean(zs))


            # -------------------------------
            # 5.2. ANCHO DE PALMA EN PÍXELES
            # -------------------------------
            # p5 -> base del índice
            p5 = np.array(

                [hand[5].x * w, hand[5].y * h],
                dtype=float,
            )

            # p17 -> base del meñique
            p17 = np.array(

                [hand[17].x * w, hand[17].y * h],
                dtype=float,
            )

            # La distancia entre ambos da una medida aproximada del tamaño de la mano
            palm_width_px = float(np.linalg.norm(p5 - p17))

            # Valor de profundidad en coordenadas world
            # Puede ser None si MediaPipe no lo proporciona
            palm_z_world_m = None


            # ----------------------------------
            # 5.3. GESTOS PARA ROTACIÓN DE BASE
            # ----------------------------------
            # En esta parte se detectan esoecíficamente:
            #   - índice bajado
            #   - meñique bajado
            # Estos gestos se usan para girar la base del robot
            index_down = hand[8].y > hand[5].y + cfg.FINGER_DOWN_MARGIN
            pinky_down = hand[20].y > hand[17].y + cfg.FINGER_DOWN_MARGIN

            # Por defecto no hay gesto de rotación
            rotation_gesture = "NONE"


            # ----------------------------------
            # 5.4. CÁLCULO DEL ROLL DE LA PALMA
            # ----------------------------------
            # Usamos la línea entre 
            #   - base del índice
            #   - base del meñique
            # El ángulo de esa línea indica cómo está girada la palma de la imagen
            # Este valor se usa para el modo de orientación de la herramienta
            index_mcp = hand[5]
            pinky_mcp = hand[17]

            dx = pinky_mcp.x - index_mcp.x
            dy = pinky_mcp.y - index_mcp.y

            palm_roll_deg = math.degrees(math.atan2(dy, dx))


            # ------------------------------------
            # 5.5. DETECCIÓN DE GESTOS ESPECIALES
            # ------------------------------------
            # Llamamamos a la función auxiliar que detecta:
            #   - paz
            #   - perfecto
            #   - puño
            #   - groseo
            #   - ok (auxiliar en desuso)
            extra_gestures = detect_extra_gestures(hand)


            # -------------------------
            # 5.6. PRIORIDAD DE GESTOS
            # --------------------------
            # No se desea que un gesto especial se confunda con un gesto de rotación de la base
            # Por eso si se detecta un gesto especial, se bloquea la rotación de la base
            # rotatios_gesture queda en None
            if extra_gestures["rude_gesture"]:
                rotation_gesture = "NONE"

            elif extra_gestures["peace_gesture"]:
                rotation_gesture = "NONE"

            elif extra_gestures["perfect_gesture"]:
                rotation_gesture = "NONE"

            elif extra_gestures["fist_gesture"]:
                rotation_gesture = "NONE"

            # Si no hay gestos espciales, entonces ya se mira el índice y el meñique
            elif index_down and not pinky_down:
                rotation_gesture = "BASE_RIGHT"

            elif pinky_down and not index_down:
                rotation_gesture = "BASE_LEFT"

            elif index_down and pinky_down:
                rotation_gesture = "STOP"

            else:
                rotation_gesture = "NONE"


            # ------------------------------------
            # 5.7. PROFUNDIDAD WORLD DE MEDIAPIPE
            # ------------------------------------
            # Si MediaPipe devuelve landmarks en coordenadas world, calculamos la Z media de la palma
            # En la versión final está en desuso y no es la base de control de rotación de la base
            if result.hand_world_landmarks:

                hand_world = result.hand_world_landmarks[0]
                palm_z_world_m = float(

                    np.mean([hand_world[i].z for i in PALM_IDS])
                )


            # -------------------------------------------
            # 5.8. DIBUJO DEL CENTRO Y ANCHO DE LA PALMA
            # -------------------------------------------
            # Círculo amarillo en el centro de la palma
            cv2.circle(

                frame,
                (int(cx), int(cy)),
                8,
                (255, 255, 0),
                -1,
            )

            # Línea magenta entre las bases del índice y el meñique
            cv2.line(

                frame,
                (int(p5[0]), int(p5[1])),
                (int(p17[0]), int(p17[1])),
                (255, 0, 255),
                2,
            )


            # ---------------------------------------------------------
            # 5.9. NOMBRE DEL GESTO DETECTADO PARA MOSTRAR EN PANTALLA
            # ---------------------------------------------------------
            detected_gesture_name = "NINGUNO"

            # Orden de prioridad visual.
            # Primero gestos críticos o especiales.
            if extra_gestures["rude_gesture"]:
                detected_gesture_name = "GROSERO"

            elif extra_gestures["peace_gesture"]:
                detected_gesture_name = "PAZ"

            elif extra_gestures["fist_gesture"]:
                detected_gesture_name = "PUNO"

            elif extra_gestures["perfect_gesture"]:
                detected_gesture_name = "PERFECTO"

            elif extra_gestures["ok_calibration_gesture"]:
                detected_gesture_name = "OK CAL"

            elif rotation_gesture != "NONE":
                detected_gesture_name = rotation_gesture

            # ----------------------------------
            # 5.10. PANEL VISUAL DE DIAGNÓSTICO
            # ----------------------------------
            # Este panel muestra en pantalla:
            #   - gesto detectado;
            #   - número de dedos cerrados;
            #   - distancia pulgar-índice;
            #   - flags de gestos.
            # Es muy útil para depurar porque permite ver por qué el programa
            # está interpretando una postura como un gesto u otro.

            panel_w = 170
            panel_h = 50

            x1 = int(w * 0.58)
            y1 = 88
            x2 = x1 + panel_w
            y2 = y1 + panel_h

            overlay = frame.copy()

            cv2.rectangle(overlay, (x1, y1), (x2, y2), (40, 40, 40), -1)
            cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 1)

            cv2.putText(

                frame,
                f"GESTO: {detected_gesture_name}",
                (x1 + 8, y1 + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 255, 255),
                1,
            )

            cv2.putText(

                frame,
                f"closed={extra_gestures['closed_count']} d={extra_gestures['thumb_index_dist']:.3f}",
                (x1 + 8, y1 + 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                (230, 230, 230),
                1,
            )

            cv2.putText(

                frame,
                (
                    f"O{int(extra_gestures['ok_calibration_gesture'])} "
                    f"P{int(extra_gestures['perfect_gesture'])} "
                    f"F{int(extra_gestures['fist_gesture'])} "
                    f"V{int(extra_gestures['peace_gesture'])} "
                    f"G{int(extra_gestures['rude_gesture'])}"
                ),
                (x1 + 8, y1 + 47),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.30,
                (210, 210, 210),
                1,
            )


            # ---------------------------------------------
            # 5.11. CREACIÓN DEL DICCIONARIO hand_features
            # ---------------------------------------------
            # Este diccionario es la salida principal del módulo.
            # main_hybrid_control.py no trabaja directamente con los landmarks,
            # sino con este resumen ya procesado.
            hand_features = {

                "center_px": center_px,
                "palm_z_norm": palm_z_norm,
                "palm_width_px": palm_width_px,
                "palm_z_world_m": palm_z_world_m,

                "index_down": index_down,
                "pinky_down": pinky_down,
                "rotation_gesture": rotation_gesture,

                "palm_roll_deg": palm_roll_deg,

                "ok_calibration_gesture": extra_gestures["ok_calibration_gesture"],
                "perfect_gesture": extra_gestures["perfect_gesture"],
                "fist_gesture": extra_gestures["fist_gesture"],
                "peace_gesture": extra_gestures["peace_gesture"],
                "rude_gesture": extra_gestures["rude_gesture"],

                "thumb_index_dist": extra_gestures["thumb_index_dist"],

            }


        # -------------------------------
        # 6. EJES VISUALES DE REFERENCIA
        # -------------------------------
        # Dibujamos una cruz azul en el centro de la imagen
        # Esto ayuda a ver la posición de la mano respecto al centro de la cámara
        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (255, 0, 0), 1)

        # Devolvemos
        #   - frame procesado
        #   - hand_features si hay mano, o None si no hay detección
        return frame, hand_features




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def release(self):
        """
        Librería de cámara
        Se llama al cerrar el programa para dejar libre el recurso de vídeo.
        """

        self.cap.release()
