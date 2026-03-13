import math


class Profiler:
    def __init__(self):
        self.start_pose_x = 60
        self.start_pose_y = 12
        self.start_pose_z = 58
        self.front_right = Leg(self.start_pose_x, self.start_pose_y, self.start_pose_z)
        self.front_left = Leg(-self.start_pose_x, self.start_pose_y, self.start_pose_z, True)
        self.rear_right = Leg(self.start_pose_x, self.start_pose_y, self.start_pose_z)
        self.rear_left = Leg(-self.start_pose_x, self.start_pose_y, self.start_pose_z, True)

        self.servos_init = False

        self.legs = [
            self.front_right,
            self.front_left,
            self.rear_right,
            self.rear_left,
        ]
        self.position_target_queue = []

    def update_position_targets(
            self,
            front_right_x, front_right_y, front_right_z,
            front_left_x, front_left_y, front_left_z,
            rear_right_x, rear_right_y, rear_right_z,
            rear_left_x, rear_left_y, rear_left_z,
    ):
        self.front_right.x_target = front_right_x
        self.front_right.y_target = front_right_y
        self.front_right.z_target = front_right_z
        self.front_left.x_target = front_left_x
        self.front_left.y_target = front_left_y
        self.front_left.z_target = front_left_z
        self.rear_right.x_target = rear_right_x
        self.rear_right.y_target = rear_right_y
        self.rear_right.z_target = rear_right_z
        self.rear_left.x_target = rear_left_x
        self.rear_left.y_target = rear_left_y
        self.rear_left.z_target = rear_left_z

    def add_position_target(
        self,
        front_right_x, front_right_y, front_right_z,
        front_left_x, front_left_y, front_left_z,
        rear_right_x, rear_right_y, rear_right_z,
        rear_left_x, rear_left_y, rear_left_z,
    ):
        self.position_target_queue.append((
            front_right_x, front_right_y, front_right_z,
            front_left_x, front_left_y, front_left_z,
            rear_right_x, rear_right_y, rear_right_z,
            rear_left_x, rear_left_y, rear_left_z,
        ))

    def get_position_commands(self):
        position_commands_by_leg = []
        if self.servos_init:
            for leg in self.legs:
                position_commands_by_leg.append(leg.get_commands_deg())
            position_commands = [
                position_commands_by_leg[0][0],
                position_commands_by_leg[1][0],
                position_commands_by_leg[0][1],
                position_commands_by_leg[1][1],
                position_commands_by_leg[0][2],
                position_commands_by_leg[1][2],
                position_commands_by_leg[2][0],
                position_commands_by_leg[3][0],
                position_commands_by_leg[2][1],
                position_commands_by_leg[3][1],
                position_commands_by_leg[2][2],
                position_commands_by_leg[3][2],
            ]
        else:
            position_commands = 12 * [None]
        return position_commands

    def tick(self):
        for leg in self.legs:
            leg.tick()

        if self.get_all_are_in_position() and self.position_target_queue:
            self.update_position_targets(*self.position_target_queue.pop(0))

    def get_all_are_in_position(self):
        all_in_position = True
        for leg in self.legs:
            if not leg.get_is_in_position():
                all_in_position = False
                break
        return all_in_position

    def get_motion_complete(self):
        return self.get_all_are_in_position() and (len(self.position_target_queue) == 0)

    def set_velocity_and_acceleration(self, velocity_max=None, acceleration=None):
        return
        # todo: Changing this temporarily causes it to sick
        # if velocity_max is None:
        #     velocity_max = self.front_right.velocity_max_default
        # if acceleration is None:
        #     acceleration = self.front_right.acceleration_default

        # for leg in [self.front_left, self.front_right, self.rear_left, self.rear_right]:
        #     leg.velocity_max = velocity_max
        #     leg.acceleration = acceleration


class Leg:
    velocity_max_default = 10.0
    acceleration_default = 0.1

    def __init__(self, x_target, y_target, z_target, invert_x=False):
        self.x_target = x_target
        self.y_target = y_target
        self.z_target = z_target

        self.x_command = x_target
        self.y_command = y_target
        self.z_command = z_target

        self.invert_x = invert_x
        self.in_position = True

        self._velocity = 0.0
        self.velocity_max = self.velocity_max_default
        self.acceleration = self.acceleration_default

        self._move_dist_total = 0
        self._move_acceleration_dist = 0
        self._move_start_x = 0
        self._move_start_y = 0
        self._move_start_z = 0

    def tick(self):
        if self.in_position:
            move_dist = math.sqrt(
                (self.x_target - self.x_command)**2 +
                (self.y_target - self.y_command)**2 +
                (self.z_target - self.z_command)**2
            )
            if move_dist != 0:
                self._move_dist_total = move_dist
                self._move_acceleration_dist = (self.velocity_max ** 2) / (2 * self.acceleration)
                if move_dist < (self._move_acceleration_dist * 2):
                    # move will not reach slew so set accell / decell point to half way through move
                    self._move_acceleration_dist = self._move_dist_total / 2
                self._move_start_x = self.x_command
                self._move_start_y = self.y_command
                self._move_start_z = self.z_command
                self.in_position = False
        else:
            dist_traveled = math.sqrt(
                (self.x_command - self._move_start_x)**2 +
                (self.y_command - self._move_start_y)**2 +
                (self.z_command - self._move_start_z)**2
            )
            dist_remaining = self._move_dist_total - dist_traveled

            if dist_remaining < self._move_acceleration_dist:
                if self._velocity > (self.acceleration * 2):
                    self._velocity -= self.acceleration  # slow down
            elif dist_traveled < self._move_acceleration_dist:
                if self._velocity < self.velocity_max:
                    self._velocity += self.acceleration  # speed up

            dist_percent_to_complete = ((dist_traveled + self._velocity) / self._move_dist_total)
            self.x_command = self._move_start_x + ((self.x_target - self._move_start_x) * dist_percent_to_complete)
            self.y_command = self._move_start_y + ((self.y_target - self._move_start_y) * dist_percent_to_complete)
            self.z_command = self._move_start_z + ((self.z_target - self._move_start_z) * dist_percent_to_complete)

            if dist_remaining < 1.0:
                self.x_command = self.x_target
                self.y_command = self.y_target
                self.z_command = self.z_target
                self.in_position = True

    def get_commands_deg(self):
        foot_len = 115.0000058
        leg_len = 120.4159355
        shoulder_len = 5.2

        if self.invert_x:
            x_command_with_invert = -self.x_command
        else:
            x_command_with_invert = self.x_command

        diag_hyp = math.sqrt((self.z_command**2) + (shoulder_len - x_command_with_invert) ** 2)
        leg_length = math.sqrt((diag_hyp**2) - (shoulder_len**2))
        shoulder_angle = math.asin((shoulder_len + x_command_with_invert) / leg_length) + math.asin(leg_length / diag_hyp) - (math.pi / 2)
        leg_length = math.sqrt((leg_length**2) + (self.y_command**2))
        leg_angle_due_to_y_component = math.sin(-self.y_command / self.z_command)
        leg_angle = -(math.acos((leg_len**2 + leg_length**2 - foot_len**2) / (2 * leg_len * leg_length)) + leg_angle_due_to_y_component)
        foot_angle = math.pi - math.acos((foot_len**2 + leg_len**2 - leg_length**2) / (2 * foot_len * leg_len))
        shoulder_angle_deg = math.degrees(shoulder_angle)
        leg_angle_deg = math.degrees(leg_angle)
        foot_angle_deg = math.degrees(foot_angle)
        return [int(x) for x in (shoulder_angle_deg, leg_angle_deg, foot_angle_deg)]

    def get_is_in_position(self):
        at_targets = (
                (self.x_target == self.x_command) and
                (self.y_target == self.y_command) and
                (self.z_target == self.z_command)
        )
        return self.in_position and at_targets
