"""
ARCHIVO: robot_controller.py
-------------------------------------------------------------------------------------------

RESUMEN:
Este archivo contiene la clase RobotController, que actúa como capa de comunicación entre
el programa Python y el robot UR3e.

Su función es encapsular todas las llamadas RTDE necesarias para:

    - conectar con el robot
    - leer la pose TCP actual
    - leer las posiciones articulares
    - mover el robot a HOME
    - mover la base
    - mover articulaciones individuales
    - mover el TCP mediante servoL
    - mover por velocidades mediante speedJ/speedL
    - parar movimientos
    - activar/desactivar salidas digitales
    - desconectar de forma segura

Este archivo NO detecta la mano y NO decide cuándo debe moverse el robot.
Eso lo hace main_hybrid_control.py.

La idea de este módulo es que el programa principal pueda escribir cosas como:

    robot.servo_to_pose(target_pose)
    robot.stop_motion()
    robot.set_digital_output(0, True)

sin tener que preocuparse directamente de cómo funciona RTDE por debajo.
"""

import time

# //////////////////////////////////////////////////////////////////////////////////////////////
class RobotController:
    """
    Clase encargada de controlar el UR3e mediante RTDE.

    Esta clase agrupa tres interfaces principales:

        · rtde_control -> Envía órdenes de movimiento al robot.

        · rtde_receive -> Lee información actual del robot, como pose TCP o articulaciones.

        · rtde_io -> Controla entradas/salidas digitales del robot.

    Además, permite trabajar en modo DRY_RUN.

    DRY_RUN significa:
        - no se conecta al robot real;
        - no se envían movimientos reales;
        - se imprimen por consola las órdenes que se habrían mandado.

    Esto es muy útil para probar la lógica del programa sin riesgo.
    """




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def __init__(
        self,
        host: str,
        dry_run: bool,
        servo_acc: float,
        servo_vel: float,
        servo_period_s: float,
        lookahead_time: float,
        gain: int,
    ):  
        """
        Constructor del controlador del robot.

        Parámetros
        ----------
        -> host : str
            IP del robot UR3e.

        -> dry_run : bool
            Si es True, no se conecta al robot real.

        -> servo_acc : float
            Aceleración usada en servoL.

        -> servo_vel : float
            Velocidad usada en servoL.

        -> servo_period_s : float
            Periodo de actualización de servoL.

        -> lookahead_time : float
            Tiempo de anticipación/suavizado de servoL.

        -> gain : int
            Ganancia interna de servoL.

        Lógica:
            1. Guarda parámetros.
            2. Comprueba que SERVO_GAIN esté en rango válido.
            3. Si DRY_RUN está activo, no conecta.
            4. Si DRY_RUN está desactivado, crea las interfaces RTDE.
        """

        # IP del robot
        self.host = host

        # Modo simulación
        self.dry_run = bool(dry_run)

        # Prámetros de servoL
        self.servo_acc = float(servo_acc)
        self.servo_vel = float(servo_vel)
        self.servo_period_s = float(servo_period_s)
        self.lookahead_time = float(lookahead_time)
        self.gain = int(gain)

        # Interfaces RTDE
        # Se inicializan a None y se crean solo si no estamos en DRY_RUN
        self.rtde_io = None
        self.rtde_c = None
        self.rtde_r = None

        # Protección importante:
        # UR RTDE exige que la ganancia de servoL esté de [100, 2000]
        # So sale de ese rango el robot puede rechazar esa orden
        if self.gain < 100 or self.gain > 2000:
            raise ValueError("SERVO_GAIN debe estar dentro del rango [100, 2000].")

        # Si estamos en DRY_RUN, no se importa ni se conecta nada del robot real
        if self.dry_run:
            print("[DRY_RUN] RobotController iniciado sin conexión real.")
            return

        # Importamos las librerías RTDE solo cuando realmente vamos a conectar
        # Esto permite que el código pueda abrir en un PC sin robot si DRY_RUN = True
        import rtde_control
        import rtde_receive
        import rtde_io

        print(f"Conectando a robot UR en {self.host}...")

        # Interfaz de control:
        # Sirve para enviar movimientos moveJ, servoL, speedJ, speedL, stop, etc.
        self.rtde_c = rtde_control.RTDEControlInterface(self.host)

        # Interfaz de lectura:
        # Sirve para leer la pose TCP, articulaciones, estado, etc
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.host)

        # Interfaz de E/S:
        # Sirve para activar/desactivar salidas digitales
        self.rtde_io = rtde_io.RTDEIOInterface(self.host)

        print("Conexión RTDE establecida.")




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def get_tcp_pose(self):
        """
        Lee la pose actual del TCP del robot.

        El TCP es el punto de herramienta del robot.

        Retorna
        -------
       -> list
            Pose actual: [x, y, z, rx, ry, rz]

            donde:
                x, y, z    -> posición en metros;
                rx, ry, rz -> orientación en vector de rotación.
        """

        # En DRY_RUN devolvemos una pose ficticia para que el resto del programa
        # pueda seguir funcionando sin robot
        if self.dry_run:
            return [0.300, -0.200, 0.250, 0.0, 3.14, 0.0]

        # En modo real, se consulta al robot
        return self.rtde_r.getActualTCPPose()




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def get_joint_positions(self):
        """
        Lee las posiciones articulares actuales del robot.

        Retorna
        -------
        -> list
            Lista de 6 valores articulares en radianes: [q0, q1, q2, q3, q4, q5]
        """
        if self.dry_run:
            return [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

        return self.rtde_r.getActualQ()




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def is_soft_joint_limit_reached(
        self,
        joint_index,
        limit_rad,
        limit_type,
        margin_rad,
    ):
        """
        Comprueba si una articulación se está acercando a un límite blando.

        Este método se usa como medida de seguridad.

        Parámetros
        ----------
        -> joint_index:
            Índice de la articulación que se quiere vigilar.

            Ejemplo:
                2 -> tercera articulación del robot.

        -> limit_rad:
            Valor límite de la articulación en radianes.

        ->limit_type:
            Tipo de límite:

                "min": se considera peligro si la articulación baja demasiado.
                "max": se considera peligro si la articulación sube demasiado.

        -> margin_rad:
            Margen de seguridad antes del límite.

        Retorna
        -------
        -> bool
            True si el límite está alcanzado o próximo.
            False si no hay problema.
        """

        # En DRY_RUN no hay articulaciones reales, así que nunca se alcanza el límite.
        if self.dry_run:
            return False

        try:
            # Leemos las posiciones articulares actuales.
            q = self.rtde_r.getActualQ()

            # Extraemos la articulación que queremos vigilar.
            q_value = float(q[int(joint_index)])

            # Convertimos límite y margen a float.
            limit = float(limit_rad)
            margin = abs(float(margin_rad))

            # Caso límite mínimo:
            # Si q_value es menor o igual que limit + margin,
            # significa que estamos cerca de bajar demasiado.
            if limit_type == "min":
                return q_value <= limit + margin

            # Caso límite máximo:
            # Si q_value es mayor o igual que limit - margin,
            # significa que estamos cerca de subir demasiado.
            if limit_type == "max":
                return q_value >= limit - margin

            # Si el tipo de límite no es válido, avisamos.
            print("[WARN safety] SAFETY_JOINT_LIMIT_TYPE no válido:", limit_type)
            return False

        except Exception as e:
            # Si hay error leyendo la articulación, no rompemos el programa.
            # Avisamos y devolvemos False.
            print("[WARN safety] No se pudo leer la articulación de seguridad:")
            print(" error =", e)
            return False





    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def move_home(self, home_joints, speed_rad_s, accel_rad_s2):
        """
        Mueve el robot a la posición HOME.

        HOME está definida en config.py como una lista de 6 articulaciones.

        Se usa moveJ, porque HOME es una posición articular.

        Parámetros
        ----------
        -> home_joints:
            Lista de 6 posiciones articulares en radianes.

        -> speed_rad_s:
            Velocidad articular.

        -> accel_rad_s2:
            Aceleración articular.
        """

        # Convertimos todas las articulaciones a float
        home_joints = [float(q) for q in home_joints]

        if self.dry_run:
            print("[DRY_RUN] move_home joints =", [round(q, 4) for q in home_joints])
            return

        try:
            # moveJ mueve el robot en espacio articular
            self.rtde_c.moveJ(
                home_joints,
                float(speed_rad_s),
                float(accel_rad_s2),
            )

        except Exception as e:
            print("[ERROR move_home] No se pudo mover a HOME:")
            print(" error =", e)
            raise




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def move_base_increment(
        self,
        delta_q0_rad,
        speed_rad_s,
        accel_rad_s2,
        asynchronous=False,
    ):
        """
        Gira la base del robot sumando un incremento a la articulación q0.

        Este método se usa para el giro de base mediante gestos de índice/meñique.

        Lógica:
            1. Lee las articulaciones actuales.
            2. Copia la lista.
            3. Suma delta_q0_rad a q0.
            4. Lanza un moveJ hacia la nueva configuración.

        Parámetros
        ----------
        -> delta_q0_rad:
            Incremento de base en radianes.

        -> speed_rad_s:
            Velocidad articular.

        -> accel_rad_s2:
            Aceleración articular.

        -> asynchronous:
            Si True, el moveJ se lanza en modo asíncrono.
            Esto permite que el programa siga ejecutándose mientras el robot gira.
        """

        if self.dry_run:
            print(
                "[DRY_RUN] move_base_increment =",
                round(float(delta_q0_rad), 4),
                "rad | async =",
                bool(asynchronous),
            )
            return

        try:
            # Leemos articulaciones actuales
            q_actual = self.rtde_r.getActualQ()

            # Creamos una copia modificable
            q_target = list(q_actual)

            # Sumamos el incremento de la base q0
            q_target[0] += float(delta_q0_rad)

            # Lanzamos el movimiento articular
            self.rtde_c.moveJ(
                q_target,
                float(speed_rad_s),
                float(accel_rad_s2),
                bool(asynchronous),
            )

        except Exception as e:
            print("[ERROR move_base_increment] No se pudo girar base:")
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def move_joint_increment(
        self,
        joint_index,
        delta_rad,
        speed_rad_s,
        accel_rad_s2,
        asynchronous=False,
    ):
        """
        Mueve cualquier articulación sumándole un incremento.

        Es una versión genérica de move_base_increment.

        Parámetros
        ----------
        -> joint_index:
            Índice de la articulación que se quiere modificar.

        -> delta_rad:
            Incremento en radianes.

        -> speed_rad_s:
            Velocidad del movimiento.

        -> accel_rad_s2:
            Aceleración del movimiento.

        -> asynchronous:
            Si True, no espera a que termine el movimiento.
        """

        if self.dry_run:
            print(
                "[DRY_RUN] move_joint_increment joint =",
                int(joint_index),
                "| delta =",
                round(float(delta_rad), 4),
                "rad | async =",
                bool(asynchronous),
            )
            return

        try:
            # Leemos articulaciones actuales.
            q_actual = self.rtde_r.getActualQ()

            # Copiamos la configuración articular.
            q_target = list(q_actual)

            # Modificamos solo la articulación elegida.
            q_target[int(joint_index)] += float(delta_rad)

            # Enviamos moveJ hacia la nueva configuración.
            self.rtde_c.moveJ(
                q_target,
                float(speed_rad_s),
                float(accel_rad_s2),
                bool(asynchronous),
            )

        except Exception as e:
            print("[ERROR move_joint_increment] No se pudo mover articulación:")
            print(" joint_index =", joint_index)
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def stop_joint_motion(self, accel_rad_s2=1.5):
        """
        Detiene un movimiento articular.

        Usa stopJ.

        Se utiliza especialmente para parar movimientos lanzados con moveJ
        o velocidades articulares.
        """

        if self.dry_run:
            print("[DRY_RUN] stop_joint_motion")
            return

        try:
            self.rtde_c.stopJ(float(accel_rad_s2))

        except Exception as e:
            print("[WARN stop_joint_motion]", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def servo_to_pose(self, target_pose):
        """
        Envía una pose objetivo al robot usando servoL.

        servoL se usa para seguimiento cartesiano continuo.

        En el proyecto, esta función es la base del modo FREE:
            mano -> offset -> pose objetivo -> servoL

        Parámetros
        ----------
        -> target_pose:
            Pose objetivo: [x, y, z, rx, ry, rz]
        """

        if self.dry_run:
            print("[DRY_RUN] servoL target_pose =", [round(float(v), 4) for v in target_pose])
            return

        try:
            # servoL no es un movimiento punto a punto normal
            # Está pensado para recibir muchas referencia en tiempo real
            self.rtde_c.servoL(
                target_pose,
                self.servo_vel,
                self.servo_acc,
                self.servo_period_s,
                self.lookahead_time,
                self.gain,
            )

        except Exception as e:
            print("[ERROR servoL] Pose rechazada por el robot:")
            print(" target_pose =", [round(float(v), 4) for v in target_pose])
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def speed_base(self, q0_speed_rad_s):
        """
        Mueve la base mediante speedJ.

        Este método pertenece a una versión anterior.
        En la versión final, la base se controla principalmente con moveJ asíncrono
        mediante move_base_increment().

        Se puede conservar como función de respaldo.
        """
        if self.dry_run:
            print("[DRY_RUN] speed_base =", round(float(q0_speed_rad_s), 4), "rad/s")
            return

        try:
            # Vector de velocidades articulares.
            #
            # Tiene 6 componentes, una por articulación.
            qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            # Solo movemos q0, la base.
            qd[0] = float(q0_speed_rad_s)

            # speedJ impone velocidades articulares.
            self.rtde_c.speedJ(
                qd,
                0.35,
                0.05,
            )

        except Exception as e:
            print("[ERROR speedJ] No se pudo mover la base:")
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def speed_joint(self, joint_index, joint_speed_rad_s, accel_rad_s2=0.6, time_s=0.08):
        """
        Mueve una articulación concreta mediante velocidad.

        Usa speedJ.

        Parámetros
        ----------
        -> joint_index:
            Índice de la articulación.

        -> joint_speed_rad_s:
            Velocidad de esa articulación en rad/s.

        -> accel_rad_s2:
            Aceleración de speedJ.

        -> time_s:
            Tiempo durante el que se aplica la orden.
        """

        if self.dry_run:
            print(
                "[DRY_RUN] speed_joint q[{}] = {:.4f} rad/s".format(
                    int(joint_index),
                    float(joint_speed_rad_s),
                )
            )
            return

        try:
            # Creamos un vector de 6 velocidades a cero
            qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            # Activamos solo la articulación deseada
            qd[int(joint_index)] = float(joint_speed_rad_s)

            # Enviamos velocidad articular
            self.rtde_c.speedJ(
                qd,
                float(accel_rad_s2),
                float(time_s),
            )

        except Exception as e:
            print("[ERROR speed_joint] No se pudo mover articulación:")
            print(" joint_index =", joint_index)
            print(" speed =", joint_speed_rad_s)
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def speed_joints(self, joint_speeds_rad_s, accel_rad_s2=0.6, time_s=0.08):
        """
        Mueve varias articulaciones mediante speedJ.

        Esta función se usa en el modo O, donde se pueden mandar velocidades
        a varias articulaciones de orientación.

        Parámetros
        ----------
        -> joint_speeds_rad_s:
            Lista de 6 velocidades articulares.

        -> accel_rad_s2:
            Aceleración.

        -> time_s:
            Duración de la orden.
        """


        if self.dry_run:
            print(
                "[DRY_RUN] speed_joints =",
                [round(float(v), 4) for v in joint_speeds_rad_s],
            )
            return

        try:
            # Convertimos todas las velocidades a floar
            qd = [float(v) for v in joint_speeds_rad_s]

            # Enviamos speedJ
            self.rtde_c.speedJ(
                qd,
                float(accel_rad_s2),
                float(time_s),
            )

        except Exception as e:
            print("[ERROR speed_joints] No se pudieron mover articulaciones:")
            print(" speeds =", joint_speeds_rad_s)
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def speed_tcp_z(self, vz_m_s):
        """
        Mueve el TCP en el eje Z mediante speedL.

        Se usa para el panel superior/inferior.

        Parámetros
        ----------
        -> vz_m_s:
            Velocidad cartesiana en Z, en m/s.
        """

        if self.dry_run:
            print("[DRY_RUN] speed_tcp_z =", round(float(vz_m_s), 4), "m/s")
            return

        try:
            # speedL usa un vector de velocidad cartesiana: [vx, vy, vz, wx, wy, wz]
            # Las tres primeras son velocidades lineales.
            # Las tres últimas son velocidades angulares.
            speed = [0.0, 0.0, float(vz_m_s), 0.0, 0.0, 0.0]

            self.rtde_c.speedL(
                speed,
                0.15,
                0.05,
            )

        except Exception as e:
            print("[ERROR speedL Z] No se pudo mover TCP en Z:")
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def speed_tcp_xz(self, vx_m_s=0.0, vz_m_s=0.0):
        """
        Mueve el TCP en X/Z mediante speedL.

        Se usa para paneles laterales y diagonales.

        Parámetros
        ----------
        -> vx_m_s:
            Velocidad en X.

        -> vz_m_s:
            Velocidad en Z.
        """

        if self.dry_run:
            print(
                "[DRY_RUN] speed_tcp_xz =",
                [round(float(vx_m_s), 4), 0.0, round(float(vz_m_s), 4)],
                "m/s",
            )
            return

        try:
            # Vector de velocidad cartesiana: [vx, vy, vz, wx, wy, wz]
            # En este proyecto, para paneles:
            #   - vx puede ser positivo/negativo;
            #   - vy se mantiene a 0;
            #   - vz puede ser positivo/negativo;
            #   - las velocidades angulares son 0.
            speed = [
                float(vx_m_s),
                0.0,
                float(vz_m_s),
                0.0,
                0.0,
                0.0,
            ]

            self.rtde_c.speedL(
                speed,
                0.15,
                0.05,
            )

        except Exception as e:
            print("[ERROR speedL XZ] No se pudo mover TCP en X/Z:")
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def set_digital_output(self, output_id, value):
        """
        Activa o desactiva una salida digital estándar del robot.

        En el proyecto se usa para controlar la ventosa/garra.

        Parámetros
        ----------
        -> output_id:
            Número de salida digital.

        -> value:
            True o False.
        """

        if self.dry_run:
            print(f"[DRY_RUN] digital output {int(output_id)} = {bool(value)}")
            return

        try:
            # Si por algún motivo rtde_io no existe, lo creamos.
            if self.rtde_io is None:
                import rtde_io
                self.rtde_io = rtde_io.RTDEIOInterface(self.host)

            # Cambiamos la salida digital estándar.           
            self.rtde_io.setStandardDigitalOut(
                int(output_id),
                bool(value),
            )

        except Exception as e:
            print("[ERROR digital output] No se pudo cambiar la salida:")
            print(" output_id =", output_id)
            print(" value =", value)
            print(" error =", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def stop_speed(self):
        """
        Detiene movimientos de velocidad.

        Usa speedStop.

        Se emplea para parar movimientos enviados con speedJ o speedL.
        """

        if self.dry_run:
            print("[DRY_RUN] stop_speed")
            return

        try:
            self.rtde_c.speedStop()

        except Exception:
            pass




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def stop_servo(self):
        """
        Detiene el modo servoL.

        Usa servoStop.

        Se emplea al salir del modo FREE o cuando se necesita cortar seguimiento
        cartesiano continuo.
        """

        if self.dry_run:
            print("[DRY_RUN] stop_servo")
            return

        try:
            self.rtde_c.servoStop()

        except Exception as e:
            print("[WARN stop_servo]", e)




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def stop_motion(self):
        """
        Parada general de movimiento.

        Intenta detener:
            1. servoL mediante servoStop;
            2. speedJ/speedL mediante speedStop.

        Esta función es útil cuando no sabemos exactamente qué tipo de movimiento
        estaba activo.
        """

        if self.dry_run:
            return
        
        # Intentamos parar servoL.
        try:
            self.rtde_c.servoStop()
        except Exception:
            pass
        
        # Pequeña pausa para que el controlador procese la parada.
        time.sleep(0.05)

        # Intentamos parar movimientos por velocidad.
        try:
            self.rtde_c.speedStop()
        except Exception:
            pass




    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def disconnect(self):
        """
        Desconecta de forma segura las interfaces RTDE.

        Lógica:
            1. Para el movimiento.
            2. Detiene el script RTDE si procede.
            3. Desconecta rtde_control.
            4. Desconecta rtde_receive.
            5. Desconecta rtde_io.
        """

        if self.dry_run:
            return
        
        # Primero intentamos parar cualquier movimiento.
        try:
            self.stop_motion()
            time.sleep(0.05)
        except Exception:
            pass
        
        # Detenemos el script RTDE.
        try:
            self.rtde_c.stopScript()
        except Exception:
            pass
        
        # Desconectamos interfaz de control.
        try:
            self.rtde_c.disconnect()
        except Exception:
            pass

        # Desconectamos interfaz de lectura.
        try:
            self.rtde_r.disconnect()
        except Exception:
            pass
        
        # Desconectamos interfaz de salidas digitales.
        try:
            if self.rtde_io is not None:
                self.rtde_io.disconnect()
        except Exception:
            pass

        print("Robot desconectado.")