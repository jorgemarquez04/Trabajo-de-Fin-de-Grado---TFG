"""
ARCHIVO: virtual_iterface.py
-------------------------------------------------------------------------------------------

RESUMEN:
Este archivo contiene la interfaz virtual que se dibuja encima de la imagen de cámara.

Su función principal es convertir la posición del centro de la mano en comandos de interfaz.

Este archivo NO mueve el robot directamente.

Lo que hace es:
    1. Crear zonas virtuales en la imagen:
        - panel superior;
        - panel inferior;
        - panel izquierdo;
        - panel derecho;
        - triángulos en las esquinas.

    2. Comprobar si el centro de la mano entra en alguna de esas zonas.

    3. Aplicar un tiempo de permanencia, llamado dwell time.

    4. Devolver un estado y un comando:
        - FREE
        - WAITING_BUTTON
        - BUTTON_ACTIVE
        - NO_HAND

    5. Dibujar la interfaz sobre el frame.

Además, este archivo contiene funciones auxiliares para el modo O:
    - detección de flechas locales;
    - zona central donde se permite el roll;
    - dibujo de las flechas alrededor del origen de la mano.

La lógica general es:

    centro de mano
        -> comprobar si está dentro de un botón
        -> esperar dwell time
        -> activar comando
        -> main_hybrid_control.py decide qué movimiento enviar al robot
"""


import time
import cv2
import numpy as np
import config as cfg


# //////////////////////////////////////////////////////////////////////////////////////////////
class VirtualInterface:
    """
    Clase principal de la interfaz virtual.

    Esta clase crea una interfaz visual encima de la imagen de cámara.

    La interfaz tiene:
        - botones rectangulares arriba, abajo, izquierda y derecha;
        - triángulos en las esquinas;
        - estados internos para saber si el usuario está en zona libre,
          esperando activar un botón o manteniendo un botón activo.

    La clase no sabe nada del robot.
    Solo devuelve comandos como:
        - Z_UP
        - Z_DOWN
        - BASE_LEFT
        - BASE_RIGHT
        - MIX_LEFT_UP
        - etc.

    Luego main_hybrid_control.py interpreta esos comandos y llama al robot.
    """

    # -----------------------
    # ESTADOS DE LA INTERFAZ
    # -----------------------
    # FREE: La mano está en zona libre. El robot puede seguir la mano con servoL.
    # WAITING_BUTTON: La mano está dentro de un botón, pero aún no ha pasado el dwell time.
    # BUTTON_ACTIVE: La mano lleva suficiente tiempo dentro del botón y el comando está activo.
    # NO_HAND: No hay mano detectada.
    FREE = "FREE"
    WAITING_BUTTON = "WAITING_BUTTON"
    BUTTON_ACTIVE = "BUTTON_ACTIVE"
    NO_HAND = "NO_HAND"


    # ------------------------
    # COMANDOS DE LA INTERFAZ
    # ------------------------
    # STOP: No hay botón activo.
    # BASE_LEFT / BASE_RIGHT: Representan los paneles laterales. Mueven X, no la base real
    # Z_UP / Z_DOWN: Panel superior e inferior.
    # MIX_...: Triángulos de esquina para movimientos diagonales.
    STOP = "STOP"

    BASE_LEFT = "BASE_LEFT"
    BASE_RIGHT = "BASE_RIGHT"

    Z_UP = "Z_UP"
    Z_DOWN = "Z_DOWN"

    MIX_LEFT_UP = "MIX_LEFT_UP"
    MIX_RIGHT_UP = "MIX_RIGHT_UP"
    MIX_LEFT_DOWN = "MIX_LEFT_DOWN"
    MIX_RIGHT_DOWN = "MIX_RIGHT_DOWN"




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def __init__(
        self,
        frame_width,
        frame_height,
        button_width_px,
        button_height_px,
        side_button_width_px,
        side_button_height_px,
        corner_triangle_size_px,
        button_margin_px,
        dwell_time_s,
    ):
        """
        Constructor de la interfaz virtual.

        Aquí se guardan las dimensiones de la imagen y de los botones.

        También se inicializa el estado interno de la interfaz.

        Parámetros
        ----------
        -> frame_width : int
            Anchura de la imagen de cámara en píxeles.

        -> frame_height : int
            Altura de la imagen de cámara en píxeles.

        -> button_width_px : int
            Anchura de los botones superior e inferior.

        -> button_height_px : int
            Altura de los botones superior e inferior.

        -> side_button_width_px : int
            Anchura de los botones laterales.

        -> side_button_height_px : int
            Altura de los botones laterales.

        -> corner_triangle_size_px : int
            Tamaño de los triángulos de las esquinas.

        -> button_margin_px : int
            Margen respecto a los bordes de la imagen.

        -> dwell_time_s : float
            Tiempo que la mano debe permanecer dentro de una zona para activarla.
        """

        # Dimensiones del frame
        self.w = int(frame_width)
        self.h = int(frame_height)

        # Dimensiones de los botones superior/inferior
        self.button_width = int(button_width_px)
        self.button_height = int(button_height_px)

        # Dimensiones de los botones laterales
        self.side_button_width = int(side_button_width_px)
        self.side_button_height = int(side_button_height_px)

        # Tamaño de los triángulos de las esquinas
        self.corner_size = int(corner_triangle_size_px)

        # Margen de separación respecto de los bordes
        self.margin = int(button_margin_px)

        # Tiempo mínimo para activar un botón
        self.dwell_time_s = float(dwell_time_s)


        # -----------------------------
        # VARIABLES INTERNAS DE ESTADO
        # -----------------------------
        # current_candidate: Botón en el que está actualmente la mano.
        # candidate_since: Momento en el que la mano entró en ese botón.
        # active_command: Comando realmente activo.
        # state: Estado actual de la interfaz.
        self.current_candidate = self.STOP
        self.candidate_since = None
        self.active_command = self.STOP
        self.state = self.FREE

        # Creamos las zonas geométricas de la interfaz
        self.buttons = self._build_buttons()
        self.triangles = self._build_triangles()




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def _build_buttons(self):
        """
        Construye los botones rectangulares de la interfaz.

        Cada botón se define como un rectángulo: (x1, y1, x2, y2)

        Donde:
            x1, y1 -> esquina superior izquierda
            x2, y2 -> esquina inferior derecha

        Retorna
        -------
        -> dict
            Diccionario que asocia cada comando con su rectángulo.
        """

        # Guardamos variables locales para que el código sea más legible.
        w = self.w
        h = self.h

        # Centro de la pantalla.
        cx = w // 2
        cy = h // 2

        # Margen y tamaños de botones.
        m = self.margin
        bw = self.button_width
        bh = self.button_height
        sw = self.side_button_width
        sh = self.side_button_height

        # Diccionario de botones.
        # Z_UP: Botón superior centrado horizontalmente.
        # Z_DOWN: Botón inferior centrado horizontalmente.
        # BASE_LEFT: Botón lateral izquierdo.
        # BASE_RIGHT: Botón lateral derecho.
        return {

            self.Z_UP: (
                cx - bw // 2,
                m,
                cx + bw // 2,
                m + bh,
            ),

            self.Z_DOWN: (
                cx - bw // 2,
                h - m - bh,
                cx + bw // 2,
                h - m,
            ),

            self.BASE_LEFT: (
                m,
                cy - sh // 2,
                m + sw,
                cy + sh // 2,
            ),

            self.BASE_RIGHT: (
                w - m - sw,
                cy - sh // 2,
                w - m,
                cy + sh // 2,
            ),

        }




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def _build_triangles(self):
        """
        Construye los triángulos de las esquinas.

        Cada triángulo se define como un array de NumPy con tres puntos: [[x1, y1], [x2, y2], [x3, y3]]

        Estos triángulos se usan para movimientos diagonales.

        Retorna
        -------
        -> dict
            Diccionario que asocia cada comando diagonal con su polígono.
        """

       
        w = self.w
        h = self.h
        m = self.margin
        s = self.corner_size

        return {
            # Triángulo superior izquierdo.
            self.MIX_LEFT_UP: np.array(
                [
                    [m, m],
                    [m + s, m],
                    [m, m + s],
                ],
                dtype=np.int32,
            ),

            # Triángulo superior derecho.
            self.MIX_RIGHT_UP: np.array(
                [
                    [w - m, m],
                    [w - m - s, m],
                    [w - m, m + s],
                ],
                dtype=np.int32,
            ),

            # Triángulo inferior izquierdo.
            self.MIX_LEFT_DOWN: np.array(
                [
                    [m, h - m],
                    [m + s, h - m],
                    [m, h - m - s],
                ],
                dtype=np.int32,
            ),

            # Triángulo inferior derecho.
            self.MIX_RIGHT_DOWN: np.array(
                [
                    [w - m, h - m],
                    [w - m - s, h - m],
                    [w - m, h - m - s],
                ],
                dtype=np.int32,
            ),
        }




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    @staticmethod
    def _point_in_rect(x, y, rect):
        """
        Comprueba si un punto está dentro de un rectángulo.

        Parámetros
        ----------
        -> x, y : float
            Coordenadas del punto.

        -> rect : tuple
            Rectángulo en formato:
                (x1, y1, x2, y2)

        Retorna
        -------
        -> bool
            True si el punto está dentro del rectángulo.
        """

        x1, y1, x2, y2 = rect

        return x1 <= x <= x2 and y1 <= y <= y2




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    @staticmethod
    def _point_in_polygon(x, y, polygon):
        """
        Comprueba si un punto está dentro de un polígono.

        En este proyecto se usa para saber si el centro de la mano está dentro
        de un triángulo de esquina.

        OpenCV tiene una función muy útil: cv2.pointPolygonTest(...)

        Si devuelve un valor >= 0, significa que el punto está dentro o justo
        sobre el borde del polígono.

        Parámetros
        ----------
        -> x, y : float
            Coordenadas del punto.

        -> polygon : np.ndarray
            Array con los puntos del polígono.

        Retorna
        -------
        -> bool
            True si el punto está dentro del polígono.
        """

        return (
            cv2.pointPolygonTest(
                polygon.astype(np.float32),
                (float(x), float(y)),
                False,
            )
            >= 0
        )




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def get_candidate_from_point(self, center_px):
        """
        Determina en qué zona de la interfaz está el centro de la mano.

        Orden de comprobación:
            1. Primero triángulos.
            2. Después botones rectangulares.
            3. Si no está en nada, devuelve STOP.

        Parámetros
        ----------
        -> center_px : np.ndarray/list
            Centro de la mano en píxeles: [x, y]

        Retorna
        -------
        -> str
            Comando candidato.
        """

        # Se extraen coordenadas del centro de la mano.
        x = float(center_px[0])
        y = float(center_px[1])

        # Primero comprobar los triángulos. Esto evita que una esquina pueda confundirse con otra zona.
        for command, poly in self.triangles.items():
            if self._point_in_polygon(x, y, poly):
                return command
            
        # Después comprobar botones rectangulares.
        for command, rect in self.buttons.items():
            if self._point_in_rect(x, y, rect):
                return command
            
        # Si no está en ningún botón ni triángulo, no hay comando.
        return self.STOP



    
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def update(self, center_px):
        """
        Actualiza el estado de la interfaz según la posición actual de la mano.

        Esta es la función principal de la clase.

        Recibe el centro de la mano y devuelve: state, command, progress

        Donde:
            state: Estado de la interfaz.
            command: Comando activo.
            progress: Porcentaje de dwell time completado.

        Lógica:
            - Si no hay mano -> NO_HAND.
            - Si la mano no está en ningún botón -> FREE.
            - Si entra en un botón nuevo -> WAITING_BUTTON.
            - Si se mantiene suficiente tiempo -> BUTTON_ACTIVE.
        """

        # Tiempo actual.
        now = time.time()

        # --------------------
        # CASO 1: NO HAY MANO
        # --------------------
        if center_px is None:

            # Reseteamos estado interno.
            self.reset()

            # Marcamos estado de mano no detectada.
            self.state = self.NO_HAND

            # No hay comando activo.
            return self.state, self.active_command, 0.0


        # ------------------------------------------
        # CASO 2: HAY MANO, BUSCAMOS ZONA CANDIDATA
        # ------------------------------------------
        candidate = self.get_candidate_from_point(center_px)


        # -----------------------------------
        # CASO 3: LA MANO ESTÁ EN ZONA LIBRE
        # -----------------------------------
        if candidate == self.STOP:

            # Reseteamos porque no estamos manteniendo ningún botón.
            self.reset()

            # Estado libre.
            self.state = self.FREE

            return self.state, self.active_command, 0.0

        
        # ---------------------------------------
        # CASO 4: LA MANO HA ENTRADO EN UN BOTÓN
        # ---------------------------------------
        if candidate != self.current_candidate:

            # Guardamos el nuevo candidato.
            self.current_candidate = candidate

            # Guardamos el instante en el que entró en esa zona.
            self.candidate_since = now

            # Todavía no se activa el comando.
            self.active_command = self.STOP

            # Estado de espera.
            self.state = self.WAITING_BUTTON

            return self.state, self.active_command, 0.0


        # ----------------------------------------
        # CASO 5: LA MANO SIGUE EN EL MISMO BOTÓN
        # ----------------------------------------
        # Calculamos cuánto tiempo lleva dentro.
        elapsed = now - self.candidate_since

        # Progreso del dwell time. Se limita a 1.0 para no pasar del 100%.
        progress = min(elapsed / self.dwell_time_s, 1.0)

        # Si ya ha pasado el dwell time, activamos el comando.
        if elapsed >= self.dwell_time_s:
            self.active_command = candidate
            self.state = self.BUTTON_ACTIVE

        # Si todavía no ha pasado suficiente tiempo, seguimos esperando.
        else:
            self.active_command = self.STOP
            self.state = self.WAITING_BUTTON

        return self.state, self.active_command, progress




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def reset(self):
        """
        Reinicia el estado interno de la interfaz.

        Se usa cuando:
            - la mano sale de un botón;
            - se pierde la mano;
            - se cambia de modo;
            - se vuelve a HOME;
            - se calibra de nuevo.
        """

        self.current_candidate = self.STOP
        self.candidate_since = None
        self.active_command = self.STOP
        self.state = self.FREE




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def draw_locked(self, frame, center_px=None, home_done=False):
        """
        Dibuja un mensaje cuando el sistema todavía no está listo para controlar.

        Casos:
            - Antes de HOME: pide pulsar h.

            - Después de HOME pero antes de calibrar: pide colocar la mano y calibrar.

        Parámetros
        ----------
        -> frame:
            Imagen donde se dibuja.

        -> center_px:
            Centro de la mano, si existe.

        -> home_done:
            Indica si el robot ya ha ido a HOME.
        """

        # Si hay mano, dibujamos su centro aunque el sistema esté bloqueado.
        if center_px is not None:
            cv2.circle(
                frame,
                (int(center_px[0]), int(center_px[1])),
                8,
                (255, 0, 255),
                -1,
            )

        # Mensaje antes de HOME.
        if not home_done:
            line1 = "Pulsa 'h' para llevar el robot a HOME"
            line2 = "Antes de HOME no se manda movimiento por mano"

        # Mensaje después de HOME y antes de calibrar.
        else:
            line1 = "Robot en HOME. Coloca la mano neutra y pulsa 'c'"
            line2 = "Tras calibrar se activara la interfaz"

        # Dibujamos primera línea.
        cv2.putText(
            frame,
            line1,
            (30, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 255),
            2,
        )

        # Dibujamos segunda línea.
        cv2.putText(
            frame,
            line2,
            (30, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 0, 255),
            2,
        )




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def draw(self, frame, center_px=None, state=None, command=None, progress=0.0):
        """
        Dibuja la interfaz virtual completa.

        Incluye:
            - ejes centrales;
            - botones rectangulares;
            - triángulos;
            - centro de la mano;
            - estado actual;
            - comando actual;
            - progreso del dwell time.

        Esta función solo dibuja. No decide movimiento del robot.
        """

        # Dimensiones del frame.
        h, w, _ = frame.shape

        # Colores usados en la interfaz.
        axis_color = (0, 0, 0)

        button_color = (255, 180, 0)
        button_wait_color = (0, 180, 255)
        button_active_color = (0, 255, 0)

        corner_color = (0, 128, 255)
        corner_wait_color = (0, 200, 255)
        corner_active_color = (0, 255, 0)

        text_color = (0, 0, 255)

        # Ejes centrales de la pantalla.
        cv2.line(frame, (0, h // 2), (w, h // 2), axis_color, 3)
        cv2.line(frame, (w // 2, 0), (w // 2, h), axis_color, 3)


        # ------------------------------
        # DIBUJAR TRIÁNGULOS DE ESQUINA
        # ------------------------------
        for cmd, poly in self.triangles.items():

            # Color por defecto.
            color = corner_color
            thickness = 3

            # Si el triángulo está activo, se dibuja en verde.
            if command == cmd and state == self.BUTTON_ACTIVE:
                color = corner_active_color
                thickness = 5

            # Si la mano está esperando dentro de ese triángulo, se dibuja distinto.
            elif self.current_candidate == cmd and state == self.WAITING_BUTTON:
                color = corner_wait_color
                thickness = 5

            # Relleno del triángulo.
            cv2.fillPoly(frame, [poly], color)

            # Contorno del triángulo.
            cv2.polylines(frame, [poly], True, axis_color, thickness)

            # Si está esperando dwell, mostramos porcentaje.
            if self.current_candidate == cmd and state == self.WAITING_BUTTON:
                x, y = poly[0]

                cv2.putText(
                    frame,
                    f"{progress * 100:.0f}%",
                    (int(x), int(y) + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    text_color,
                    2,
                )


        # ------------------------------
        # DIBUJAR BOTONES RECTANGULARES
        # ------------------------------
        for cmd, rect in self.buttons.items():
            x1, y1, x2, y2 = rect

            # Color por defecto.
            color = button_color
            thickness = 3

            # Botón activo.
            if command == cmd and state == self.BUTTON_ACTIVE:
                color = button_active_color
                thickness = 5

            # Botón esperando dwell.
            elif self.current_candidate == cmd and state == self.WAITING_BUTTON:
                color = button_wait_color
                thickness = 5

            # Relleno del botón.
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, -1)

            # Contorno.
            cv2.rectangle(frame, (x1, y1), (x2, y2), axis_color, thickness)

            # Texto del botón.
            label = cmd.replace("_", " ")

            cv2.putText(
                frame,
                label,
                (x1 + 7, y1 + 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                axis_color,
                2,
            )

            # Barra de progreso si está esperando dwell.
            if self.current_candidate == cmd and state == self.WAITING_BUTTON:
                bar_width = int((x2 - x1) * progress)

                cv2.rectangle(
                    frame,
                    (x1, y2 + 5),
                    (x1 + bar_width, y2 + 12),
                    button_wait_color,
                    -1,
                )

        # Dibujamos el centro de la mano.
        if center_px is not None:
            cv2.circle(
                frame,
                (int(center_px[0]), int(center_px[1])),
                8,
                (255, 0, 255),
                -1,
            )

        # Texto inferior con estado y comando.
        text = f"STATE: {state} | CMD: {command}"

        cv2.putText(
            frame,
            text,
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            text_color,
            2,
        )

# //////////////////////////////////////////////////////////////////////////////////////////////




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def _draw_arrow_outline(frame, pts, color=(255, 0, 0), thickness=3):
    """
    Dibuja el contorno de una flecha.

    Esta función recibe una lista de puntos y dibuja un polígono cerrado.

    En la versión actual del archivo puede estar como función auxiliar.
    """

    pts_np = np.array(pts, dtype=np.int32)
    cv2.polylines(frame, [pts_np], isClosed=True, color=color, thickness=thickness)



    
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def _fill_arrow_alpha(frame, pts, fill_ratio, color=(255, 0, 0), alpha=0.28):
    """
    Rellena parcialmente una flecha con transparencia.

    Parámetros
    ----------
    -> frame:
        Imagen sobre la que se dibuja.

    -> pts:
        Puntos que forman la flecha.

    -> fill_ratio:
        Porcentaje de relleno:
            0.0 -> sin relleno
            1.0 -> relleno completo

    -> color:
        Color del relleno.

    -> alpha:
        Transparencia del relleno.

    -> Nota:
        Esta función puede usarse para representar visualmente una intensidad
        o progreso.
    """

    # Limitamos fill_ratio entre 0 y 1.
    fill_ratio = max(0.0, min(1.0, float(fill_ratio)))

    # Si el relleno es casi cero, no dibujamos nada.
    if fill_ratio <= 0.01:
        return
    
    # Convertimos los puntos de la flecha a array de NumPy.
    pts_np = np.array(pts, dtype=np.int32)

    # Obtenemos el rectángulo que envuelve la flecha.
    x, y, w, h = cv2.boundingRect(pts_np)

    # Copia del frame para dibujar encima.
    overlay = frame.copy()

    # Rellenamos la flecha en el overlay.
    cv2.fillPoly(overlay, [pts_np], color)

    # Creamos una máscara para que el relleno sea parcial.
    if w >= h:
        # Flecha horizontal.
        visible_w = int(w * fill_ratio)
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        mask[y:y+h, x:x+visible_w] = 255

    else:
        # Flecha vertical.
        visible_h = int(h * fill_ratio)
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        mask[y + (h - visible_h):y+h, x:x+w] = 255

    # Mezclamos overlay y frame original con transparencia.
    blended = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

    # Aplicamos la parte mezclada solo donde la máscara vale 255.
    frame[mask == 255] = blended[mask == 255]




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def _point_in_rect(point_px, rect):
    """
    Comprueba si un punto está dentro de un rectángulo.

    Esta función es parecida al método estático de la clase VirtualInterface,
    pero está fuera de la clase para usarse en el modo O.

    Parámetros
    ----------
    -> point_px:
        Punto en píxeles: [x, y]

    ->rect:
        Rectángulo: (x1, y1, x2, y2)

    Retorna
    -------
    -> bool
        True si el punto está dentro.
    """

    x, y = point_px
    x1, y1, x2, y2 = rect

    return x1 <= x <= x2 and y1 <= y <= y2




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def is_tool_roll_center_active(center_px, anchor_px):
    """
    Comprueba si la mano está dentro de la zona central del modo 'O'.

    En el modo 'O':
        - las flechas controlan pitch/side;
        - el círculo central permite el roll de herramienta.

    Esta función devuelve True solo si el centro actual de la mano está dentro
    del radio configurado alrededor del origen del modo 'O'.

    Parámetros
    ----------
    -> center_px:
        Centro actual de la mano.

    -> anchor_px:
        Punto donde estaba la mano al activar el modo 'O'.

    Retorna
    -------
    -> bool
        True si la mano está dentro del círculo central.
    """

    if center_px is None or anchor_px is None:
        return False
    
    # Diferencia entre la posición actual y el origen del modo 'O'.
    dx = float(center_px[0]) - float(anchor_px[0])
    dy = float(center_px[1]) - float(anchor_px[1])

    # Distancia euclídea al centro.
    dist = (dx * dx + dy * dy) ** 0.5

    # Si la distancia es menor que el radio configurado, se permite el roll.
    return dist <= float(cfg.TOOL_ROLL_CENTER_RADIUS_PX)




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def get_tool_arrow_candidate(center_px, anchor_px):
    """
    Detecta si la mano está dentro de alguna flecha del modo O.

    Al activar el modo 'O', se guarda un punto anchor_px.
    Alrededor de ese punto se generan cuatro zonas:

        - TOOL_UP
        - TOOL_DOWN
        - TOOL_LEFT
        - TOOL_RIGHT

    Esta función comprueba si el centro de la mano está dentro de alguna zona.

    Parámetros
    ----------
    -> center_px:
        Centro actual de la mano.

    -> anchor_px:
        Centro de referencia guardado al entrar en modo O.

    Retorna
    -------
    -> str
        Comando candidato:
            "TOOL_UP"
            "TOOL_DOWN"
            "TOOL_LEFT"
            "TOOL_RIGHT"
            "NONE"
    """

    if center_px is None or anchor_px is None:
        return "NONE"
    
    # Origen local del modo O.
    cx = int(anchor_px[0])
    cy = int(anchor_px[1])

    # Posición actual de la mano.
    px = int(center_px[0])
    py = int(center_px[1])

    # Parámetros de tamaño de flechas desde config.py.
    offset = int(cfg.TOOL_ARROW_OFFSET_PX)
    bw = int(cfg.TOOL_ARROW_WIDTH_PX)
    bh = int(cfg.TOOL_ARROW_HEIGHT_PX)

    # Rectángulo de activación de la flecha superior.
    up_rect = (
        cx - bw // 2,
        cy - offset - bh // 2,
        cx + bw // 2,
        cy - offset + bh // 2,
    )

    # Rectángulo de activación de la flecha inferior.
    down_rect = (
        cx - bw // 2,
        cy + offset - bh // 2,
        cx + bw // 2,
        cy + offset + bh // 2,
    )

    # Rectángulo de activación de la flecha izquierda.
    # Aquí se intercambian bw/bh para adaptar mejor la zona al dibujo horizontal.
    left_rect = (
        cx - offset - bh // 2,
        cy - bw // 2,
        cx - offset + bh // 2,
        cy + bw // 2,
    )

    # Rectángulo de activación de la flecha derecha.
    right_rect = (
        cx + offset - bh // 2,
        cy - bw // 2,
        cx + offset + bh // 2,
        cy + bw // 2,
    )

    point = (px, py)

    # Comprobamos cada rectángulo.
    if _point_in_rect(point, up_rect):
        return "TOOL_UP"

    if _point_in_rect(point, down_rect):
        return "TOOL_DOWN"

    if _point_in_rect(point, left_rect):
        return "TOOL_LEFT"

    if _point_in_rect(point, right_rect):
        return "TOOL_RIGHT"

    return "NONE"




# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def draw_tool_orient_overlay(
    frame,
    anchor_px,
    candidate_command="NONE",
    active_command="NONE",
    progress=0.0,
):
    """
    Dibuja la interfaz local del modo 'O'.

    Esta interfaz aparece alrededor del punto donde estaba la mano al activar
    el modo orientación.

    Elementos dibujados:
        - círculo central para permitir roll;
        - cruz central;
        - flecha arriba;
        - flecha abajo;
        - flecha izquierda;
        - flecha derecha.

    Parámetros
    ----------
    -> frame:
        Imagen sobre la que se dibuja.

    -> anchor_px:
        Punto central del modo O.

    -> candidate_command:
        Flecha sobre la que está actualmente la mano, antes de completar dwell.

    -> active_command:
        Flecha ya activada tras completar dwell.

    -> progress:
        Progreso del dwell time entre 0.0 y 1.0.
    """

    # Si no hay origen del modo O, no se puede dibujar.
    if anchor_px is None:
        return
    
    # Dimensiones del frame.
    h, w = frame.shape[:2]

    # Centro local del modo 'O'.
    cx = int(anchor_px[0])
    cy = int(anchor_px[1])

    # Distancia desde el centro hasta las flechas.
    offset = int(cfg.TOOL_ARROW_OFFSET_PX)

    # Colores.
    color = (180, 60, 210)
    active_color = (0, 255, 255)
    line_color = (0, 180, 255)

    # Limitamos progress entre 0 y 1.
    progress = max(0.0, min(1.0, float(progress)))


    # -------------------------
    # CÍRCULO CENTRAL DEL ROLL
    # -------------------------
    # Solo cuando la mano está dentro de este círculo, main_hybrid_control.py
    # permite el roll de herramienta.
    center_radius = int(cfg.TOOL_ROLL_CENTER_RADIUS_PX)

    cv2.circle(
        frame,
        (cx, cy),
        center_radius,
        (180, 60, 210),
        2,
    )

    # Cruz central para marcar visualmente el origen.
    cv2.line(frame, (cx - 35, cy), (cx + 35, cy), line_color, 2)
    cv2.line(frame, (cx, cy - 35), (cx, cy + 35), line_color, 2)
    cv2.circle(frame, (cx, cy), 5, line_color, -1)


    # -----------------------------------------------
    # FUNCIÓN INTERNA: DEFINIR PUNTOS DE CADA FLECHA
    # -----------------------------------------------
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def arrow_points(direction):
        """
        Devuelve la lista de puntos que forman cada flecha.

        Cada flecha se dibuja como un polígono.
        """

        if direction == "TOOL_UP":
            return [
                (cx, cy - offset - 45),
                (cx - 28, cy - offset),
                (cx - 12, cy - offset),
                (cx - 12, cy - offset + 42),
                (cx + 12, cy - offset + 42),
                (cx + 12, cy - offset),
                (cx + 28, cy - offset),
            ]

        if direction == "TOOL_DOWN":
            return [
                (cx, cy + offset + 45),
                (cx - 28, cy + offset),
                (cx - 12, cy + offset),
                (cx - 12, cy + offset - 42),
                (cx + 12, cy + offset - 42),
                (cx + 12, cy + offset),
                (cx + 28, cy + offset),
            ]

        if direction == "TOOL_LEFT":
            return [
                (cx - offset - 45, cy),
                (cx - offset, cy - 28),
                (cx - offset, cy - 12),
                (cx - offset + 42, cy - 12),
                (cx - offset + 42, cy + 12),
                (cx - offset, cy + 12),
                (cx - offset, cy + 28),
            ]

        if direction == "TOOL_RIGHT":
            return [
                (cx + offset + 45, cy),
                (cx + offset, cy - 28),
                (cx + offset, cy - 12),
                (cx + offset - 42, cy - 12),
                (cx + offset - 42, cy + 12),
                (cx + offset, cy + 12),
                (cx + offset, cy + 28),
            ]

        return []


    # -------------------------------------
    # FUNCIÓN INTERNA: DIBUJAR UNA FLECHA
    # -------------------------------------
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def draw_arrow(command):
        """
        Dibuja una flecha concreta.

        Si la flecha es candidata, se rellena parcialmente.
        Si la flecha está activa, se resalta más.
        """

        pts = arrow_points(command)

        if not pts:
            return

        pts_np = np.array(pts, dtype=np.int32)

        # Si esta flecha está activa, usa color activo.
        draw_color = active_color if command == active_command else color

        # Si esta flecha está activa, el borde es más grueso.
        thickness = 4 if command == active_command else 3


        # -----------------
        # FLECHA CANDIDATA
        #------------------
        # Si la mano está sobre esta flecha pero aún no se completó el dwell,
        # se rellena con transparencia proporcional al progreso.
        if command == candidate_command and progress > 0.0:

            overlay = frame.copy()

            cv2.fillPoly(overlay, [pts_np], draw_color)

            alpha = 0.15 + 0.30 * progress

            cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)


        # --------------
        # FLECHA ACTIVA
        #----------------
        # Si ya ha pasado el dwell time, se rellena más fuerte.
        if command == active_command:
            overlay = frame.copy()

            cv2.fillPoly(overlay, [pts_np], draw_color)

            cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)

        # Dibujamos el contorno final de la flecha.
        cv2.polylines(
            frame,
            [pts_np],
            isClosed=True,
            color=draw_color,
            thickness=thickness,
        )

    # Dibujamos las cuatro flechas.
    draw_arrow("TOOL_UP")
    draw_arrow("TOOL_DOWN")
    draw_arrow("TOOL_LEFT")
    draw_arrow("TOOL_RIGHT")
