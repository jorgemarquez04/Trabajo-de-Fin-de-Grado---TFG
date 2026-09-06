import rtde_receive

ROBOT_HOST = "158.42.215.182"

def main():

    rtde_r = None

    try:

        print(f"Conectando a UR3 en {ROBOT_HOST}...")

        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_HOST)

        q = rtde_r.getActualQ()

        pose = rtde_r.getActualTCPPose()

        print("")

        print("Articulaciones actuales:")

        for i, value in enumerate(q):

            print(f"q[{i}] = {value:.6f} rad")

        print("")

        print("Valor de seguridad recomendado para articulación 3:")

        print(f"SAFETY_JOINT_INDEX = 2")

        print(f"SAFETY_JOINT_LIMIT_RAD = {q[2]:.6f}")

        print("")

        print("TCP pose actual:")

        print([round(float(v), 6) for v in pose])

        print("")

    finally:

        if rtde_r is not None:

            try:

                rtde_r.disconnect()

            except Exception:

                pass

        print("Conexión cerrada.")

if __name__ == "__main__":

    main()
