import logging

import pybullet
import time
import spot_sim_util


def main():
    loop_period_s = 19.65E-3
    spot_sim = spot_sim_util.SpotSim()
    spot_sim.reset()
    try:
        while True:
            spot_sim.step()
            pybullet.stepSimulation()
            while time.perf_counter() < spot_sim.next_loop_s:
                time.sleep(0.0001)
            spot_sim.next_loop_s = time.perf_counter() + loop_period_s
            if spot_sim.rc_thread.window.was_closed():
                raise KeyboardInterrupt
    except (KeyboardInterrupt, pybullet.error) as e:
        logging.info("Caught exception... Killing threads then terminating...")
        spot_sim.serial_thread.kill()
        spot_sim.serial_thread.join()
        spot_sim.rc_thread.kill()
        spot_sim.rc_thread.join()
        if pybullet.isConnected():
            pybullet.disconnect()
        # Eat the exceptions associated with killing the program or closing the pybullet window
        if (
                (not isinstance(e, KeyboardInterrupt))
                and
                ('Not connected to physics server' not in repr(e))
        ):
            raise
        else:
            logging.info("Exception was expected. Exception was eaten.")


if __name__ == "__main__":
    main()
