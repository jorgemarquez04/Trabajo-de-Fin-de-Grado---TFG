"""
ARCHIVO: filters.py
-------------------------------------------------------------------------------------------

RESUMEN: Módulo de filtrado para suavizar la señal de movimiento de la mano antes de enviarlo
al robot.
En este proyecto, MediaPipe detecta la mano frame a frame, aunque la mano esté quieta, la 
posición de los landmarks varía ligeramente como consecuencia del ruido de cámara, iluminación,
pequeñas oscilaciones o imprecisiones del modelo.
Si esa señal se enviase directamente al robot, el UR3e intentaría copiar también esos
micromovimientos. Por eso este archivo introduce dos filtros:

1. EmaFilter: Suaviza los cambios entre un frame y el siguiente

2. DeadbandFreezeFilter: Congela la referencia cuando el movimiento es muy pequeño y
   probablemente corresponda a ruido, no a una intención real.

La cadena de uso en main_hybrid_control.py es:

    raw_offset = mapper.hand_features_to_robot_offset_m(hand_features)
    smooth_offset = ema.update(raw_offset)
    filtered_offset = freeze.update(smooth_offset)

Es decir:

    señal bruta de la mano
        -> suavizado EMA
        -> eliminación de microtemblores
        -> referencia final para el robot


GUÍA DE LECTURA:
Este archivo es corto pero importante: evita que el robot se mueva por ruido de visión. `EmaFilter`
suaviza cambios; `DeadbandFreezeFilter` mantiene la salida estable cuando el movimiento real es
pequeño.
"""

import time

import numpy as np

# //////////////////////////////////////////////////////////////////////////////////////////////
class EmaFilter:

    """
    Filtro de media exponencial.
        - alpha bajo -> más suave, más retardo
        - alph alto -> más rápido, más ruido

    Este filtro no sustituye directamente el valor anterior por el nuevo,
    sino que mezcla ambos:

        valor_filtrado =
            alpha * valor_nuevo
            +
            (1 - alpha) * valor_filtrado_anterior

    De esta manera, si el valor nuevo cambia de golpe, el filtro no pega
    un salto inmediato, sino que se acerca progresivamente.

    En el proyecto, esto sirve para que el robot no copie de forma brusca
    todos los cambios pequeños de la detección de la mano.
    """


    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def __init__(self, alpha: float):
        """
        Constructor de filtro.
        
        Parámetros
        ----------
        -> alpha : float
            Factor de suavizado, si alpha es alto el filtro hace mucho caso al dato nuevo.
            El robot responde más rápido, pero puede copiar más ruido.
            Si alpha es bajo, el filtro hace más caso al valor anterior.
            El robot se mueve más suave, pero con más retardo.

            Ejemplo:
                alpha = 0.30
                nuevo filtrado = 30% del valor nuevo y 70% del valor filtrado anterior
        """

        # Guardamos alpha como número decimal y lo convertimos a float por si llega int o similar
        self.alpha = float(alpha)

        # Esta variable guarda el último valor filtrado, al principio None porque todavía
        # no ha llegado ningún dato
        self.value = None


    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def reset(self, value=None):
        """
        Reinicia el estado interno del filtro
        
        Esto se usa cuando recalibramos el sistema, volvemos a HOME, cambiamos de modo o queremos
        evitar que el filtro arrastre memoria de un movimiento anterior.

        Parámetros
        ----------
        -> value : list, tuple, np.ndarray o None.
            Valor inicial opcional.

            Si value es None: El filtro queda vacío y esperará al primer dato real
            Si value tiene valor: el filtro arranca directamente desde ese punto 
        """

        if value is None:
            # Dejamos el filtro sin memoria
            # El próximo update() tomará el dato recibido como primer valor
            self.value = None

        else:
            # Convertimos el valor inicial a array de Numpy [0.0, 0.0, 0.0]
            self.value = np.array(value, dtype=float)


    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def update(self, new_value):
        """
        Actualiza el filtro con un nuevo valor.

        Este método se llama en cada ciclo de control

        Parámetros
        ----------
        -> new_value : list, tuple o np.ndarray
            Nuevo valor medido
        
            En este proyecto suele ser el offset de la mano convertido a metros:
            [offset_x, offset_y, offset_z]
        
        Retorna
        -------
        np.ndarray
            Valor filtrado
        """

        # Convertimos el dato recibido a array Numpy para poder operar componente a componente
        # Ejemplo: new_value = [0.01, 0.00, 0.03]
        new_value = np.array(new_value, dtype=float)

        # Caso especial: primera llamada
        # Si todavía no existe un valor filtrado anterior, no podemos aplicar la fórmula EMA
        if self.value is None:

            self.value = new_value.copy()

        # Caso normal: ya existe un valor anterior 
        # Aquí aplicamos la fórmula EMA
        else:

            self.value = (
                self.alpha * new_value
                + (1.0 - self.alpha) * self.value
            )

        return self.value.copy()
# //////////////////////////////////////////////////////////////////////////////////////////////




# //////////////////////////////////////////////////////////////////////////////////////////////
class DeadbandFreezeFilter:
    """
    Filtro de zona muerta de congelación.

    Este filtro sirve para evitar que el robot se mueva por variaciones muy pequeñas de la señal
    
    La idea principal es:
        Si el cambio respecto al último valor aceptado es muy pequeño probablemente sea ruido.
        Si además ese movimiento pequeño se mantiene durante un cierto tiempo, se congela la
        referencia y se sigue devolviendo el último valor estable
    """



    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def __init__(self, deadband_m: float, freeze_speed_m_s: float, freeze_time_s: float,):
        """
        Constructor del filtro.

        Parámetros
        ----------
        -> deadband_m : float
            Umbral de desplazamiento mínimo
            
            Si el cambio entre el valor nuevo y el último valor estable es menor que este umbral,
            se considera movimiento pequeño
            Unidad: metros
        
        -> freeze_speed_s : float
            Umbral de velocidad mínima.
            
            Sirve para detectar si el movimiento es realmente lento
            Si la velocidad estimada es menor que este valor, el sistema entiende que la mano
            está prácticamente quieta
            Unidad: m/s

        -> freeze_time_s : float
            Tiempo que debe mantenerse la condición de bajo movimiento previo a congelar la referencia.
            
            Unidad: segundos.
        """

        # Guardamos los parámetros de configuración
        self.deadband_m = float(deadband_m)
        self.freeze_speed_m_s = float(freeze_speed_m_s)
        self.freeze_time_s = float(freeze_time_s)

        # Último valor aceptado como referencia válida
        # Este valor se devuelve cuando decidimos congelar
        self.last_value = None

        # Instente de tiempo de laúltima llamada a update()
        # Usado para calcular dt, es decir, tiempo entreuna actualización y la siguiente
        self.last_time = None

        # Momento desde el quue la señal lleva estando casi quieta
        # Si esta situación dura más de freeze_time_s, se congela la referencia 
        self.still_since = None




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def reset(self, value=None):
        """
        Reinicia el filtro.

        Se usa al recalibrar, volver a HOME o cambiar de modo

        Parámetros
        ----------
        -> value: list, tuple, np.ndarray o None
            Valor inicial opcional
        """

        if value is None:
            # Si no se pasa valor inicial, borramos la última referencia
            # En la próxima llamada update(), el filtro tomará el primer valor recibido como pto partida
            self.last_value = None

        else:
            # Si se pasa un valor inicial, ese valor se toma como última referenvia 
            self.last_value = np.array(value, dtype=float)

        # Guardamos el tiempo actual como referencia inicial
        self.last_time = time.time()

        # Reiniciamos el contador de "mano quieta"
        self.still_since = None




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def update(self, value):
        """
        Actualiza el filtro.

        Este método recibe el valor suavizado por el EMA y decide si debe
        - Dejarlo pasar
        - Aplicar zona muerta
        - Congelar referencia

        Parámetros
        ----------
        -> value: list, tuple, np.ndarray
            Valor de entrada del filtro, normalmente [offset_x, offset_y, offset_z]

        Retorna
        -------
        -> np.ndarray
            Valor final filtrado
        """
        # Convertimos el valor recibido a array de Numpy
        value = np.array(value, dtype=float)

        # Guardamos el instante actual
        now = time.time()

        #----------------
        # 1. ZONA MUERTA
        #----------------
        # Mira cada componente del vector, si una componente es menor que deadband_m
        # en valor absolsuto, entonces se fuerza a 0
        # Ejemplo: deadband_m = 0.001 y value = [0.0004, 0.0, 0.0020] entonces 
        # value = [0.0, 0.0, 0.0020]
        value[np.abs(value) < self.deadband_m] = 0.0


        # -------------------
        # 2. PRIMERA LLAMADA
        # -------------------
        # Si todavía no tenemos el valor anterior, no podemos calcular la velocidad
        # Por lo que guardamos el valor y tiempo actual como último valor  y devolvemos
        # directamente el valor
        if self.last_value is None:

            self.last_value = value.copy()

            self.last_time = now

            return value.copy()


        # --------------------------------
        # 3. CÁLCULO DEL TIEMPO ENTRE FRAMES
        # --------------------------------
        # dt es el tiempo pasado desde la última actualización
        # Se emplea max(..., 1e-6) para evitar que dt sea 0,
        # si dt fuera 0, al calcular la velocidad habría una división por 0
        dt = max(now - self.last_time, 1e-6)


        # ---------------------------
        # 4. ESTIMACIÓN DE VELOCIDAD
        # ---------------------------
        # Calculamos cuánto ha cambiado el valor actual respecto al último valor
        # value - self.last_value da un vector diferencia
        # np.linalg.norm() calcula la longitud de ese vector
        # Finalemnete velocidad = distancia / tiempo
        speed = np.linalg.norm(value - self.last_value) / dt


        # ----------------------------
        # 5. DETECCIÓN DE MANO QUIETA
        # ----------------------------
        # Si la velocidad calculada es menor que freeze_speed_m_s, consideramos
        # que la señal está prácticamente quieta
        if speed < self.freeze_speed_m_s:

            # Si acabamos de entrar en esta situación de baja velocidad, guardamos 
            # instante donde empezó
            if self.still_since is None:
                self.still_since = now

            # Si la señal lleva quieta más tiempo que freeze_time_s congelamos valor
            # es decir, no aceptar el nuevo value, usar de nuevo self.last_value
            if now - self.still_since >= self.freeze_time_s:
                value = self.last_value.copy()

        # Si la velocidad vuelve a ser suficientemente alta, el usuario sí mueve la mano
        # Cancelamos condición de congelación
        else:
            self.still_since = None


        # ----------------------
        # 6. ACTUALIZAR MEMORIA
        # ----------------------
        # Guardamos el valor final como último valor
        self.last_value = value.copy()

        # Guardamos el tiempo actual para la siguiente iteración
        self.last_time = now

        # Devolvemos una copia del valor final
        return value.copy()
