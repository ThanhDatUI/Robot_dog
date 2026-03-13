import datetime
import enum
import logging
import os
import threading
import time

import serial.tools.list_ports
# noinspection PyPep8Naming
import PySimpleGUI as sg


class SimulationMode(enum.Enum):
    SW_GUI = enum.auto()
    SW_TX = enum.auto()
    HW = enum.auto()


class RCSimThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.running = True
        self.log = logging.getLogger(__name__)
        self.window = None
        self.simulation_mode = SimulationMode.SW_GUI
        self.com_ports = [port.name for port in serial.tools.list_ports.comports()]
        self.com_port = serial.Serial(baudrate=115200)
        # Expand serial rx buffer to try not to lose data. Alternatively, may service port faster.
        self.com_port.set_buffer_size(rx_size=12800)
        self.file_path = os.path.join(os.path.dirname(__file__), 'spot_sim_data.txt')

        # Debug flags
        self.log_simulated_rc_commands = False
        self.log_servo_angles = False
        self.log_joint_euler_angles = False

    def run(self) -> None:
        self.make_gui()

        # Truncate the file that the simulator uses for control and motor data
        with open(self.file_path, 'w') as rc_input_fp:
            rc_input_fp.write('')

        # Event loop to process 'events' and get the 'values' of the inputs
        previous_simulation_mode = self.simulation_mode
        while self.running:
            event, values = self.window.read(timeout=200)

            if event is sg.WIN_CLOSED:  # if user closes window or clicks cancel
                self.write_to_log_and_gui_log('RC Simulator Window Closed')
                self.running = False
                break

            if isinstance(values, dict):
                # Read simulation mode from GUI
                if values[SimulationMode.SW_GUI]:
                    self.simulation_mode = SimulationMode.SW_GUI
                elif values[SimulationMode.SW_TX]:
                    self.simulation_mode = SimulationMode.SW_TX
                elif values[SimulationMode.HW]:
                    self.simulation_mode = SimulationMode.HW
                else:
                    raise KeyError('No simulation mode was set')

            # Detect transition in simulation mode and perform actions that are transition specific
            if self.simulation_mode != previous_simulation_mode:
                self.write_to_log_and_gui_log('Changed Simulation Mode: {}'.format(self.simulation_mode))
                previous_simulation_mode = self.simulation_mode

                # If mode uses the com port, close it and re-open it in case user changed com port.
                # If mode doesn't use com port, it will be closed here too.
                # Disable irrelevant controls in GUI based on simulation mode.
                if self.com_port.is_open:
                    self.write_to_log_and_gui_log('Closed COM port: {}'.format(self.com_port.name))
                    self.com_port.close()
                if self.simulation_mode in (SimulationMode.SW_TX, SimulationMode.HW):
                    self.disable_rc_inputs(False)
                    self.com_port.port = values["com_port"]
                    try:
                        self.com_port.open()
                    except serial.serialutil.SerialException as e:
                        self.write_to_log_and_gui_log('Failed to open COM port: {}\n{}'.format(
                            values["com_port"], e))
                        self.log.exception('Failed to open COM port')
                    else:
                        self.write_to_log_and_gui_log('Opened COM port: {}'.format(values["com_port"]))
                else:
                    self.disable_rc_inputs(True)

            # Perform the simulation mode specific actions on every event loop
            if self.simulation_mode == SimulationMode.SW_GUI:
                self.write_simulated_rc_values_to_file(values)
            elif self.simulation_mode in (SimulationMode.SW_TX, SimulationMode.HW):
                self.write_com_port_to_file()
            else:
                raise KeyError('Unknown Simulation mode {}'.format(self.simulation_mode))

            # update debug flags
            if values['log_gui_values']:
                self.write_to_log_and_gui_log('GUI Values: {}'.format(values))
            self.log_simulated_rc_commands = values['log_simulated_rc_commands']
            self.log_servo_angles = values['log_servo_angles']
            self.log_joint_euler_angles = values['log_joint_euler_angles']

        self.com_port.close()
        self.write_to_log_and_gui_log('RC Simulator Terminated')

    def make_gui(self):
        # Define the GUI layout
        sg.theme('DarkAmber')  # Add a touch of color
        text_width = 5
        center_pwm = 980
        full_pwm_range = 1600  # Note that this is not centered about zero
        half_pwm_range = full_pwm_range // 2
        layout = [
            [sg.Text('Simulation Mode')],
            [
                sg.Radio(
                    'Software with RC from GUI',
                    default=True,
                    group_id='simulation_mode',
                    key=SimulationMode.SW_GUI
                )
            ],
            [
                sg.Text('Arm', size=(text_width, 0)),
                sg.Slider(
                    range=(center_pwm - half_pwm_range, center_pwm + half_pwm_range),
                    default_value=center_pwm - half_pwm_range,
                    resolution=full_pwm_range,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key='arm'
                ),
            ],
            [
                sg.Text('Mode', size=(text_width, 0)),
                sg.Slider(
                    range=(center_pwm - half_pwm_range, center_pwm + half_pwm_range),
                    default_value=center_pwm - half_pwm_range,
                    resolution=half_pwm_range,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key='mode'
                )

            ],
            [
                sg.Text('Pitch', size=(text_width, 0)),
                sg.Slider(
                    range=(center_pwm - half_pwm_range, center_pwm + half_pwm_range),
                    default_value=center_pwm,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key='pitch'
                )

            ],
            [
                sg.Text('Roll', size=(text_width, 0)),
                sg.Slider(
                    range=(center_pwm - half_pwm_range, center_pwm + half_pwm_range),
                    default_value=center_pwm,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key='roll'
                )

            ],
            [
                sg.Text('Throttle', size=(text_width, 0)),
                sg.Slider(
                    range=(center_pwm - half_pwm_range, center_pwm + half_pwm_range),
                    default_value=center_pwm,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key='throttle'
                )
            ],
            [sg.HorizontalSeparator()],
            [sg.Text('Robot COM Port (Required for other Simulation Modes)')],
            [
                sg.DropDown(
                    values=self.com_ports,
                    default_value=self.com_ports[0],
                    size=(2*text_width, 0),
                    key='com_port'
                )
            ],
            [
                sg.Radio(
                    'Software with RC from Transmitter',
                    group_id='simulation_mode',
                    key=SimulationMode.SW_TX
                )
            ],
            [
                sg.Radio(
                    'Hardware',
                    group_id='simulation_mode',
                    key=SimulationMode.HW
                )
            ],
            [sg.HorizontalSeparator()],
            [sg.Text('Debug Flags')],
            [sg.Checkbox("Log GUI Values", key='log_gui_values')],
            [sg.Checkbox("Log Simulated RC Commands", key='log_simulated_rc_commands')],
            [sg.Checkbox("Log Servo Angles (robot must be armed)", key='log_servo_angles')],
            [sg.Checkbox("Log Joint Euler Angles (robot must be armed)", key='log_joint_euler_angles')],
            [sg.HorizontalSeparator()],
            [
                sg.Multiline(
                    autoscroll=True,
                    write_only=True,
                    size=(60, 10),
                    font='Courier 8',
                    disabled=True,
                    key='log'
                )
            ]
        ]

        # Create the GUI window
        self.window = sg.Window(
            'RC Controller Simulator',
            layout,
            location=(1530, 172),
        )

    def write_simulated_rc_values_to_file(self, values):
        with open(self.file_path, 'a') as sim_data_fp:
            rc_command_packet = '\x02{}\x03\r\n'.format('\t'.join([
                '2',  # Packet type 2 is RC input
                str(int(values['pitch'])),  # 0th RC channel Y command
                str(int(values['roll'])),  # X command
                str(int(values['throttle'])),  # Z command
                '3',
                '4',
                '5',
                str(int(values['arm'])),  # Arm spot
                str(int(values['mode'])),  # Mode Control
                '8',
                '9',
                '10',
                '11',
                '12',
                '13',
                '14',
                '15',
                '16',
                '17',
                '18',
            ]))
            sim_data_fp.write(rc_command_packet)
            if self.log_simulated_rc_commands:
                self.write_to_log_and_gui_log('RC Simulator Wrote: {}'.format(rc_command_packet.strip()))

    def write_com_port_to_file(self):
        if self.com_port.is_open:  # Make sure port is open before trying to interface with it
            if self.com_port.in_waiting >= 64:
                bytes_from_serial_port = self.com_port.read(64)
                with open(self.file_path, 'a', buffering=64) as sim_data_fp:
                    sim_data_fp.write(bytes_from_serial_port.decode())
                self.write_to_log_and_gui_log("RC Simulator Wrote: ".format(bytes_from_serial_port))
            else:
                self.write_to_log_and_gui_log("{} had no new bytes".format(self.com_port.name))
        else:
            self.write_to_log_and_gui_log(
                "{} is not open. Select Simulation Mode again to re-open.".format(self.com_port.name))

    def write_to_log_and_gui_log(self, message):
        if not self.window.was_closed():
            self.window['log'].print('{} - {}'.format(datetime.datetime.now().isoformat(), message))
        self.log.debug(message)

    def disable_rc_inputs(self, disabled):
        rc_control_elements = [
            self.window['arm'],
            self.window['mode'],
            self.window['pitch'],
            self.window['roll'],
            self.window['throttle'],
        ]
        for rc_control_element in rc_control_elements:
            rc_control_element.update(disabled=disabled)

    def kill(self):
        self.running = False


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s.%(msecs)03d %(name)s %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d,%H:%M:%S'
    )
    rc_sim_thread = RCSimThread()
    rc_sim_thread.start()

    try:
        while rc_sim_thread.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        rc_sim_thread.kill()
        rc_sim_thread.join()
