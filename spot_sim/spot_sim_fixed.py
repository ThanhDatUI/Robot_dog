import logging
import pybullet
import time
import threading
import os
import sys
import math
import enum

import pybullet_data
import PySimpleGUI as sg
import serial.tools.list_ports

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "micropython"))

import state
from profiler import Profiler
import spot_sim_util
import rc_sim

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(name)s %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d,%H:%M:%S'
)


class SpotSimFixed:
    """Modified SpotSim that doesn't auto-start RC thread"""
    def __init__(self):
        self.log = logging.getLogger(__name__)
        
        # Create RC controller object but DON'T start it as thread
        self.rc_controller = rc_sim.RCSimThread()
        self.rc_controller.running = True  # Mark as running but don't call start()
        
        # startup pybullet
        self.physicsClient = pybullet.connect(pybullet.GUI)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        
        # for software sim
        self.profiler = Profiler()
        self.state_machine = state.StateMachine()
        
        # for hardware and software sims
        self.serial_thread = spot_sim_util.ReadSerialFileThread()
        self.serial_thread.start()
        self.next_loop_s = 0
        
        self.bot_id = None
        
    def reset(self):
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setTimeStep(0.01)
        pybullet.loadURDF("plane.urdf")
        
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=0.75,
            cameraYaw=60,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )
        
        urdf_xml_path = os.path.join(
            os.path.dirname(__file__), "..", "nvidia-jetson-nano", "urdf", "spotmicroai_gen.urdf.xml"
        )
        
        self.bot_id = pybullet.loadURDF(
            urdf_xml_path,
            [0, 0, 0.255],
            pybullet.getQuaternionFromEuler([0, 0, math.pi]),
            flags=pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_INITIALIZE_SAT_FEATURES,
        )
        
        num_joints = pybullet.getNumJoints(self.bot_id)
        joint_info = [pybullet.getJointInfo(self.bot_id, i) for i in range(num_joints)]
        for joint in joint_info:
            if joint[1][-3:] == b"toe":
                pybullet.changeDynamics(self.bot_id, joint[0], lateralFriction=0.2)
        
        # Start robot in prone position
        start_shoulder = 51
        start_leg = -54
        start_foot = 140
        self.goto_deg(
            start_shoulder, start_shoulder,
            start_leg, start_leg,
            start_foot, start_foot,
            start_shoulder, start_shoulder,
            start_leg, start_leg,
            start_foot, start_foot,
        )
        
    def step_simulation(self):
        """One step of physics simulation"""
        if self.rc_controller.simulation_mode in (rc_sim.SimulationMode.SW_GUI, rc_sim.SimulationMode.SW_TX):
            self._software_sim()
        elif self.rc_controller.simulation_mode is rc_sim.SimulationMode.HW:
            self._hardware_sim()
        
        pybullet.stepSimulation()
        
    def _hardware_sim(self):
        targets = self.serial_thread.get_targets()
        if targets:
            self.goto_deg(*targets)
            
    def _software_sim(self):
        self.profiler.tick()
        pos_cmds = self.profiler.get_position_commands()
        rc_command = self.serial_thread.get_rcs()
        self.state_machine.update(self.profiler, rc_command, self.next_loop_s * 1e6)
        self.goto_deg(*pos_cmds)
        
    def goto_deg(self, front_right_shoulder, front_left_shoulder, front_right_leg, front_left_leg,
                 front_right_foot, front_left_foot, rear_right_shoulder, rear_left_shoulder,
                 rear_right_leg, rear_left_leg, rear_right_foot, rear_left_foot):
        if self.bot_id is None:
            return
            
        num_joints = pybullet.getNumJoints(self.bot_id)
        joint_info = [pybullet.getJointInfo(self.bot_id, i) for i in range(num_joints)]
        param_dict_with_b_keys = {key.encode(): val for key, val in locals().items()}
        
        for joint in joint_info:
            if joint[1] in param_dict_with_b_keys.keys():
                if param_dict_with_b_keys[joint[1]] is not None:
                    target_position_degrees = param_dict_with_b_keys[joint[1]]
                    target_position = math.radians(target_position_degrees)
                    if b"left_shoulder" in joint[1]:
                        target_position = -target_position
                    pybullet.setJointMotorControl2(
                        bodyUniqueId=self.bot_id,
                        jointIndex=joint[0],
                        controlMode=pybullet.POSITION_CONTROL,
                        targetPosition=target_position,
                        maxVelocity=math.radians(1 / (0.11 / 60)),
                        force=35,
                    )


def main():
    loop_period_s = 19.65E-3
    spot_sim = SpotSimFixed()
    spot_sim.reset()
    
    # Create GUI in main thread
    spot_sim.rc_controller.make_gui()
    
    # Truncate data file
    with open(spot_sim.rc_controller.file_path, 'w') as rc_input_fp:
        rc_input_fp.write('')
    
    previous_simulation_mode = spot_sim.rc_controller.simulation_mode
    
    try:
        while spot_sim.rc_controller.running:
            # GUI event handling (main thread)
            event, values = spot_sim.rc_controller.window.read(timeout=10)  # Short timeout for responsive sim
            
            if event == sg.WIN_CLOSED:
                spot_sim.rc_controller.write_to_log_and_gui_log('RC Simulator Window Closed')
                spot_sim.rc_controller.running = False
                break
            
            # Handle simulation mode changes
            if isinstance(values, dict):
                if values.get(rc_sim.SimulationMode.SW_GUI):
                    spot_sim.rc_controller.simulation_mode = rc_sim.SimulationMode.SW_GUI
                elif values.get(rc_sim.SimulationMode.SW_TX):
                    spot_sim.rc_controller.simulation_mode = rc_sim.SimulationMode.SW_TX
                elif values.get(rc_sim.SimulationMode.HW):
                    spot_sim.rc_controller.simulation_mode = rc_sim.SimulationMode.HW
                
                # Handle mode transitions
                if spot_sim.rc_controller.simulation_mode != previous_simulation_mode:
                    spot_sim.rc_controller.write_to_log_and_gui_log('Changed Simulation Mode: {}'.format(spot_sim.rc_controller.simulation_mode))
                    previous_simulation_mode = spot_sim.rc_controller.simulation_mode
                    
                    # Handle COM port
                    if spot_sim.rc_controller.com_port.is_open:
                        spot_sim.rc_controller.write_to_log_and_gui_log('Closed COM port: {}'.format(spot_sim.rc_controller.com_port.name))
                        spot_sim.rc_controller.com_port.close()
                    
                    if spot_sim.rc_controller.simulation_mode in (rc_sim.SimulationMode.SW_TX, rc_sim.SimulationMode.HW):
                        spot_sim.rc_controller.disable_rc_inputs(False)
                        spot_sim.rc_controller.com_port.port = values["com_port"]
                        try:
                            spot_sim.rc_controller.com_port.open()
                        except Exception as e:
                            spot_sim.rc_controller.write_to_log_and_gui_log('Failed to open COM port: {}\n{}'.format(values["com_port"], e))
                        else:
                            spot_sim.rc_controller.write_to_log_and_gui_log('Opened COM port: {}'.format(values["com_port"]))
                    else:
                        spot_sim.rc_controller.disable_rc_inputs(True)
                
                # Perform simulation mode actions
                if spot_sim.rc_controller.simulation_mode == rc_sim.SimulationMode.SW_GUI:
                    spot_sim.rc_controller.write_simulated_rc_values_to_file(values)
                elif spot_sim.rc_controller.simulation_mode in (rc_sim.SimulationMode.SW_TX, rc_sim.SimulationMode.HW):
                    spot_sim.rc_controller.write_com_port_to_file()
                
                # Update debug flags
                if values.get('log_gui_values'):
                    spot_sim.rc_controller.write_to_log_and_gui_log('GUI Values: {}'.format(values))
                spot_sim.rc_controller.log_simulated_rc_commands = values.get('log_simulated_rc_commands', False)
                spot_sim.rc_controller.log_servo_angles = values.get('log_servo_angles', False)
                spot_sim.rc_controller.log_joint_euler_angles = values.get('log_joint_euler_angles', False)
            
            # Run physics simulation step
            if time.perf_counter() >= spot_sim.next_loop_s:
                spot_sim.step_simulation()
                spot_sim.next_loop_s = time.perf_counter() + loop_period_s
                
    except (KeyboardInterrupt, pybullet.error) as e:
        logging.info("Caught exception... Terminating...")
    finally:
        # Clean up
        spot_sim.serial_thread.kill()
        spot_sim.serial_thread.join()
        
        if spot_sim.rc_controller.com_port.is_open:
            spot_sim.rc_controller.com_port.close()
        
        if pybullet.isConnected():
            pybullet.disconnect()
        
        spot_sim.rc_controller.write_to_log_and_gui_log('RC Simulator Terminated')
        logging.info("Simulation terminated cleanly")


if __name__ == "__main__":
    main()

