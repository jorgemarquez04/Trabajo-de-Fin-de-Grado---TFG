"""
ARCHIVO: trajectory_mapper.py
-------------------------------------------------------------------------------------------

RESUMEN:
Este archivo se encarga de convertir el movimiento de la mano, detectado en píxeles por la
cámara, en un desplazamiento real del robot en metros.

Dicho de forma sencilla:

    posición de la mano en la imagen
        -> diferencia respecto al punto calibrado
        -> conversión de píxeles a metros
        -> offset X/Y/Z del robot
        -> pose objetivo del TCP

Este módulo NO lee la cámara y NO mueve el robot directamente.

Su función es actuar como traductor entre:

    hand_tracker.py          -> datos de la mano
    main_hybrid_control.py   -> lógica principal
    robot_controller.py      -> movimiento real del UR3e

La idea clave es que, cuando el usuario calibra, se guardan dos referencias:

    1. La posición inicial de la mano en la imagen.
    2. La pose inicial del robot.

Después, cuando la mano se mueve, este archivo calcula cuánto debe desplazarse el robot
respecto a esa pose inicial.

En la versión final del proyecto:
    - el eje X del robot se controla con el desplazamiento horizontal de la mano;
    - el eje Z del robot se controla con el desplazamiento vertical de la mano;
    - el eje Y/profundidad está desactivado por robustez.
"""


import numpy as np
import config as cfg


# //////////////////////////////////////////////////////////////////////////////////////////////
class TrajectoryMapper:
    """
    Clase encargada de mapear la posición de la mano a movimiento del robot.

    Esta clase guarda la calibración entre mano y robot.

    Para poder funcionar necesita saber:

        - dónde estaba la mano al calibrar;
        - dónde estaba el robot al calibrar;
        - cuánto equivale un píxel en metros;
        - cuál es el desplazamiento máximo permitido.

    Después, cada vez que llega una nueva posición de mano, calcula un offset:

        offset = [dx, dy, dz]

    Ese offset se suma después a la pose inicial del robot para obtener la pose objetivo.
    """



    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def __init__(
        self,
        pixel_to_meter_x: float,
        pixel_to_meter_z: float,
        gain: float,
        max_offset_x: float,
        max_offset_y: float,
        max_offset_z: float,
        enable_y_depth: bool,
        depth_mode: str,
        mediapipe_z_to_meter_y: float,
        depth_gain: float,
        depth_sign: float,
    ):
        """
        Constructor de la clase TrajectoryMapper.

        Aquí no se calcula todavía ningún movimiento, solo se guardan parámetros necesarios
        para poder hacer la conversión mano -> robot.

        Parámetros
        ----------
        -> pixel_to_meter_x : float
            Factor de conversión para el eje X

            Indica cuántos metros se mueve el robot por cada píxel de desplazamiento.
            horizontal de la mano.

        -> pixel_to_meter_z : float
            Factor de conversión para el eje Z

            Indica cuántos metros se mueve el robot por cada píxel de desplazamiento
            vertical de la mano

        -> gain : float
            Ganancia general del movimiento

            Permite aumentar o reducir la sensibilidad global del sistema

        -> max_offset_x : float
            Desplazamiento máximo permitido en X respecto al origen calibrado

        -> max_offset_y : float
            Desplazamiento máximo permitido en Y respecto al origen calibrado
        
        -> max_offset_z : float
            Desplazamiento máximo permitido en Z respecto al origen calibrado

        -> enable_y_depth : bool
            Activa o desactiva el control de profundidad Y

            En la versión final está desactivado

        -> depth_mode : str
            Modo de profundidad.

            En esta versión solo se acepta "mediapipe_z"

        -> mediapipe_z_to_meter_y : float
            Factor de conversión entre profundidad MediaPipe y movimiento 'Y'

            Se conserva como parámetro, aunque el control 'Y' esté desactivado 
        
        -> depth_gain : float
            Ganancia adicional para profundidad.
        
        -> depth_sign : float
            Signo del eje de profundidad

            Permitiría invertir el movimiento 'Y' si fuese necesario
        """

        # ------------------------------------------- 
        # FACTORES DE CONVERSIÓN DE PÍXELES A METROS
        # -------------------------------------------
        # La cámara trabaja en píxeles, pero el robot trabaja en metros
        # Por ejemplo, si pixel_to_meter_x = 0.001, entonces el desplazamiento
        # de 100 píxeles equivale a: 100 * 0.001 = 0.1 m
        # En este proyecto, estos valores se ajustan en config.py
        self.pixel_to_meter_x = float(pixel_to_meter_x)
        self.pixel_to_meter_z = float(pixel_to_meter_z)


        # -----------------
        # GANANCIA GENERAL
        # -----------------
        # La ganancia multiplica el movimiento final
        # Si gain = 1.0 -> se usa la conversión normal
        # Si gain = 0.5 -> el robot se mueve la mitad
        # Si gain = 2.0 -> el robot se mueve el doble
        self.gain = float(gain)


        # ------------------
        # LÍMITES DE OFFSET
        # ------------------
        # max_offset guarda cuánto se permite alejar el robot de la pose calibrada
        # Es un vector: [máximo X, máximo Y, máximo Z]
        # [0.14, 0.0, 0.15] significa que en Z puede moverse +/- 14 cm y etc.
        self.max_offset = np.array(

            [max_offset_x, max_offset_y, max_offset_z],
            dtype=float,
        )


        # --------------------------
        # PARÁMETROS DE PROFUNDIDAD
        # --------------------------
        # Estos parámetros permiten preparar un posible control del eje Y usando la profundidad
        # estimada por MediaPipe.
        # En la versión final, enable_y_depth está desactivado porque la profundidad resultaba
        # menos estable y podía introducir movimientos no deseados.
        self.enable_y_depth = bool(enable_y_depth)
        self.depth_mode = str(depth_mode)

        # Esta versión del código solo contempla profundidad mediante MediaPipe Z.
        # Si se pusiera otro modo en config.py, lanzamos un error para avisar de que
        # no está implementado.
        if self.depth_mode != "mediapipe_z":

            raise ValueError(
                "Esta versión limpia solo acepta DEPTH_MODE='mediapipe_z'."
            )

        # Factor de conversión de profundidad MediaPipe a metros en Y.
        self.mediapipe_z_to_meter_y = float(mediapipe_z_to_meter_y)

        # Ganancia extra para el eje de profundidad.
        self.depth_gain = float(depth_gain)

        # Signo del movimiento en profundidad.
        # Sirve para invertir el sentido si acercar la mano a la cámara mueve el robot
        # al revés de lo esperado.
        self.depth_sign = float(depth_sign)


        # -------------------------
        # VARIABLES DE CALIBRACIÓN
        # -------------------------
        # Estas variables empiezan en None porque al crear el objeto todavía no se ha calibrado
        # Se rellenan cuando el usuario pulsa 'C' o hace el gesto 'PAZ'
        self.hand_origin_px = None
        self.palm_origin_z_norm = None
        self.robot_origin_pose = None




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    @property
    def is_calibrated(self):
        """
        Indica si el mapper está calibrado.
        Un mapper está calibrado cuando ya tiene guardadas las tres referencias:

        1. hand_origin_px: posición inicial de la mano en píxeles
        2. palm_origin_z_norm: profundidad de la palma según MediaPipe
        3. robot_origin_pose: pose inicial del robot

        Retorna
        -------
        -> bool
            True si todas las referencias existen
            False si todavía falta alguna
        """

        # Esta propiedad devuelve True solo si las 3 variables de calibración tienen valor
        # Si alguna sigue siendo None, significa que aún se puede convertir
        # correctamente la mano en movimiento del robot
        return (

            self.hand_origin_px is not None
            and self.palm_origin_z_norm is not None
            and self.robot_origin_pose is not None
        )



    
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def calibrate_origin(self, hand_features, robot_pose):
        """
        Guarda el origen de calibración.
        
        Esta función se llama cuando el usuario calibra el sistema.

        En ese instante se toma una "foto" de dos cosas:
            -   posición inicial de la mano
            -   pose actual del robot
        
        A partir de ese momento, todos los movimientos se calculan respecto a esas referencias

        Parámetros
        ----------
        -> hand_features : dict
            Diccionario generado por hand_tracker.py

            Debe contener:
                - "center_px"
                - "palm_z_norm"
        
        -> robot_pose : list
            Pose actual del TCP del robot.
            
            Normalmente tiene esta forma: [x, y, z, rx, ry, rz]
        """

        # ------------------
        # ORIGEN DE LA MANO
        # ------------------
        # Guardamos el centro de la mano en píxeles
        # Por ejemplo: [320.0, 240.0]
        # Esto será el punto neutro, si la mano vuelve aquí, el offser será 0
        self.hand_origin_px = np.array(

            hand_features["center_px"],
            dtype=float,
        )


        # ----------------------
        # ORIGEN DE PROFUNDIDAD
        # ----------------------
        # Guardamos la profundidad normalizada de la palma en el instante de calibración
        # En la versión final no se usa para mover 'Y', pero se conserva como parte
        # de la estructura del mapper.
        self.palm_origin_z_norm = float(

            hand_features["palm_z_norm"]
        )


        # -----------------
        # ORIGEN DEL ROBOT
        # -----------------
        # Guardamos la pose actual del TCP
        # Está pose será la referencia a la que se sumarán los offsets de la mano
        self.robot_origin_pose = np.array(

            robot_pose,
            dtype=float,
        )

    


    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def _depth_offset_m(self, hand_features):
        """
        Calcula el desplazamiento en profundidad Y.

        En esta versión final devuelve siempre 0.0 porque el control de profundidad
        no se utiliza.

        Motivo:
            La profundidad estimada por MediaPipe puede ser más inestable que el movimiento
            X/Z en imagen. Para evitar movimientos no deseados, se dejó el eje Y desactivado.
        
        Parámetros
        ----------
        -> hand_features : dict
            Datos de la mano.

        Retorna
        -------
        -> float
            Desplazamiento en Y en metros.

            En esta versión:
                siempre 0.0
        """

        # Aunque la función existe, no se utiliza realmente.
        # Se conserva porque deja preparada la arquitectura para una futura versión
        # donde sí se controle la profundidad.
        return 0.0




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def hand_features_to_robot_offset_m(self, hand_features):
        """
        Convierte la posición actual de la mano en un offset del robot.

        Esta función es una de las más importantes del archivo.

        Hace la conversión:
            movimiento de mano en píxeles -> movimiento del robot en metros
        
        Parámetros
        ----------
        -> hand_features : dict
            Datos actuales de la mano generados por hand_tracker.py

        Retorna
        -------
        -> np.ndarray
            Offset del robot en metros: [dx, dy, dz]
        """

        # ---------------------------------------
        # 1. COMPROBAR QUE LA CALIBRACIÓN EXISTE
        # ---------------------------------------
        # Sin calibración no sabemos:
        #   - dónde estaba la mano al inicio;
        #   - dónde estaba el robot al inicio.
        # Por tanto, no podemos calcular un desplazamiento relativo.
        if not self.is_calibrated:
            raise RuntimeError("Mapper no calibrado. Pulsa 'c' primero.")


        # ---------------------------------
        # 2. LEER CENTRO ACTUAL DE LA MANO
        # ---------------------------------
        # hand_features["center_px"] viene de hand_tracker.py.
        # Es un vector: [cx, cy]
        # donde 'cx' es horizontal y 'cy' es vertical en píxeles.  
        hand_center_px = np.array(

            hand_features["center_px"],
            dtype=float,
        )


        # -------------------------------------------------
        # 3. CALCULAR DESPLAZAMIENTO DE LA MANO EN PÍXELES
        # -------------------------------------------------
        # Restamos: posición actual de la mano - posición de mano al calibrar
        # Si da: delta_px = [0, 0] -> la mano está en el origen calibrado
        # Si delta_px[0] es positivo: la mano se ha movido hacia la derecha
        # Si delta_px[1] es positivo: la mano se ha movido hacia abajo
        delta_px = hand_center_px - self.hand_origin_px


        # ------------------------
        # 4. CONVERSIÓN DEL EJE X
        # ------------------------
        # El desplazamiento horizontal de la mano se convierte en movimiento 'X' del robot
        # Fórmula: dx = delta_x_px * pixel_to_meter_x * gain * FREE_X_SIGN
        #   · delta_x_px[0]     ->  movimiento horizontal de la mano en píxeles
        #   · pixel_to_meter_x  ->  conversión píxel - metro      
        #   · gain  ->  sensibilidad general
        #   · FREE_X_SIGN   ->  permite invertir el sentido si es necesario
        dx = (

            delta_px[0]
            * self.pixel_to_meter_x
            * self.gain
            * cfg.FREE_X_SIGN
        )


        # ---------------------
        # 5. EJE Y DESACTIVADO
        # ---------------------
        # En la versión final no usamos profundidad para mover el robot en Y.
        # Por eso dy se deja siempre a cero.
        # Esto significa que el robot no se acerca ni se aleja en profundidad durante
        # el modo libre.
        dy = 0.0


        # ------------------------
        # 6. CONVERSIÓN DEL EJE Z
        # ------------------------
        # El desplazamiento vertical de la mano se convierte en movimiento Z del robot.
        # En imagen, si la mano sube, delta_px[1] es negativo, ya que el eje Y crece hacia abajo
        # Por eso aparece un signo menos al principio de la ecuación, de este modo hacemos
        # que al subir la mano el robot suba, y al bajarla el robot baje
        dz = (

            -delta_px[1]
            * self.pixel_to_meter_z
            * self.gain
            * cfg.FREE_Z_SIGN
        )


        # -----------------------
        # 7. CREAR VECTOR OFFSET
        # -----------------------
        # El offset representa cuánto debe moverse el robot respecto a la pose calibrada
        # Tiene 3 componentes: [dx, dy, dz]
        offset = np.array([dx, dy, dz], dtype=float)


        # ----------------------------
        # 8. LIMITAR EL OFFSET MÁXIMO
        # ----------------------------
        # np.clip limita cada componente dentro del rango
        # En este caso:
        #   offset mínimo = -self.max_offset
        #   offset máximo = self.max_offset
        #
        # Ejemplo:
        #   si dx = 0.30 m; max_offset_x = 0.14 m
        #   entonces dx se limita a 0.14 m
        offset = np.clip(

            offset,
            -self.max_offset,
            self.max_offset,
        )

        # Devolvemos el offset limitado
        return offset




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def target_pose_from_offset(self, filtered_offset_m):
        """
        Calcula la pose objetivo del robot a partir del offset filtrado.
        
        Esta función se llama después de aplicar los filtros.

        Recibe: 
            
            filtered_offset_m = [dx, dy, dz]
        
        y lo suma la pose de origen del robot:
        
            target_pose = robot_origin_pose + offset
        
        Parámetros
        ----------
        -> filtered_offset_m : list, tuple o np.ndaarray
            Offset final ya filtrado
        
        Retorna
        -------
        -> list
            Pose objetivo del robot: [x, y, z, rx, ry, rz]
        """

        # -------------------------
        # 1. COMPROBAR CALIBRACIÓN
        # -------------------------
        if not self.is_calibrated:
            raise RuntimeError("Mapper no calibrado.")

        # Convertimos el offset recibido a array de Numpy
        filtered_offset_m = np.array(

            filtered_offset_m,
            dtype=float,
        )


        # --------------------------------------
        # 2. COPIAR LA POSE DE ORIGEN DEL ROBOT
        # --------------------------------------
        # robot_origin_pose tiene esta forma: [x, y, z, rx, ry, rz]
        # Hacemos una copia para no modificar el origen real guardado
        target = self.robot_origin_pose.copy()


        # ---------------------------
        # 3. SUMAR OFFSET CARTESIANO
        # ---------------------------
        # La orientación se mantiene igual, solo modificamos las 3 primeras componentes: x, y, z
        target[0] = self.robot_origin_pose[0] + filtered_offset_m[0]
        target[1] = self.robot_origin_pose[1] + filtered_offset_m[1]
        target[2] = self.robot_origin_pose[2] + filtered_offset_m[2]


        # ---------------------------------
        # 4. MANTENER ORIENTACIÓN ORIGINAL
        # ---------------------------------
        # Las componentes: rx, ry, rz se copian desde la pose de origen
        # Esto significa que en el modo libre el robot solo cambia de posición
        # pero mantiene la orientación de la herramienta, que se controla en el modo 'O'
        target[3:6] = self.robot_origin_pose[3:6]


        # -----------------------
        # 5. DEVOLVER COMO LISTA
        # -----------------------
        # RTDE suele trabajar cómodamente con lista de Python
        # Por eso convertimos el arrat final a una lista
        return target.tolist()
