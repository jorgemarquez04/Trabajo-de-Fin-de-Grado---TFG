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

        print("HOME_JOINTS = [")

        for value in q:

            print(f"    {value:.6f},")

        print("]")

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
