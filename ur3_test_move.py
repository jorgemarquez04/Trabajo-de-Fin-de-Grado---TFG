import time

import rtde_control

import rtde_receive

ROBOT_HOST = "158.42.215.182"

DELTA_Z_M = 0.02

VEL = 0.03

ACC = 0.05

def main():

    rtde_c = None

    rtde_r = None

    try:

        print(f"Conectando a UR3 en {ROBOT_HOST}...")

        rtde_c = rtde_control.RTDEControlInterface(ROBOT_HOST)

        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_HOST)

        print("Conectado correctamente.")

        q = rtde_r.getActualQ()

        tcp_pose = rtde_r.getActualTCPPose()

        print("")

        print("Estado inicial:")

        print("  q (rad):", [f"{qi:.3f}" for qi in q])

        print("  TCP pose [x y z rx ry rz]:", [f"{p:.3f}" for p in tcp_pose])

        target_up = tcp_pose.copy()

        target_up[2] += DELTA_Z_M

        print("")

        print(f"Moviendo TCP arriba {DELTA_Z_M * 100:.1f} cm...")

        print("  Target:", [f"{p:.3f}" for p in target_up])

        rtde_c.moveL(target_up, VEL, ACC)

        time.sleep(1.0)

        print("")

        print("Volviendo a pose inicial...")

        rtde_c.moveL(tcp_pose, VEL, ACC)

        time.sleep(1.0)

        print("")

        print("Prueba completada correctamente.")

    except Exception as e:

        print("")

        print("ERROR durante la prueba:")

        print(repr(e))

    finally:

        print("")

        print("Desconectando de forma segura...")

        if rtde_c is not None:

            try:

                rtde_c.stopScript()

            except Exception:

                pass

            try:

                rtde_c.disconnect()

            except Exception:

                pass

        if rtde_r is not None:

            try:

                rtde_r.disconnect()

            except Exception:

                pass

        print("Conexión cerrada.")

if __name__ == "__main__":

    main()
