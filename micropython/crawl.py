""" Implements a basic statically stable crawl gate where one leg is moved at a time. """


def crawl(profiler, crawl_velocity: float, sub_state: int) -> int:
    """
    While dog is crawling or use is commanding crawl, update profiler with new movement commands.

    :param profiler: Profiler to add position targets to.
    :param crawl_velocity: User commanded velocity to determine direction.
    :param sub_state: Current crawl phase to be used and updated by this function.
    :return: Next sub_state.
    """
    if profiler.get_motion_complete():
        leg_z = 145
        if sub_state == 0:  # if not crawling
            if crawl_velocity < 0:  # and commanded to craw forward
                sub_state += 1
            elif crawl_velocity > 0:  # and command to crawl backwards
                sub_state -= 1
            else:
                profiler.set_velocity_and_acceleration()
                profiler.add_position_target(
                    front_right_x=0, front_right_y=-30, front_right_z=leg_z,
                    front_left_x=0, front_left_y=-30, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=-30, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=-30, rear_left_z=leg_z,
                )
        else:  # if crawling
            leg_y_step = 110
            body_y_step =  57
            leg_z_step = 55          
            left_leg_z_step = 55       

            step_velocity_max = 30.0
            step_acceleration = 5.0

            if sub_state == -1:
                sub_state = 16

            if sub_state == 0:  # shift body weight backwards while moving front leg
                # todo: fix bug with not going to state zero. Handle new states for not moving
                profiler.set_velocity_and_acceleration()
                leg_y = 0 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 1:  # front left lift
                profiler.set_velocity_and_acceleration(step_velocity_max, step_acceleration)
                leg_y = 0 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y, front_left_z=leg_z - left_leg_z_step,
                    rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 2:  # front left forward
                leg_y = 0 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z - left_leg_z_step,
                    rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 3:  # front left lower
                profiler.set_velocity_and_acceleration()
                leg_y = 0 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )

            elif sub_state == 4:  # shift body weight forward while moving rear leg
                leg_y = -1 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 5:  # rear right lift
                profiler.set_velocity_and_acceleration(step_velocity_max, step_acceleration)
                leg_y = -1 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z - leg_z_step,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 6:  # rear right forward
                leg_y = -1 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z - leg_z_step,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 7:  # rear right lower
                profiler.set_velocity_and_acceleration()
                leg_y = -1 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )

            elif sub_state == 8:  # shift body weight backward while moving front leg
                leg_y = -2 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 9:  # front right lift
                profiler.set_velocity_and_acceleration(step_velocity_max, step_acceleration)
                leg_y = -2 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y, front_right_z=leg_z - leg_z_step,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 10:  # front right forward
                leg_y = -2 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z - leg_z_step,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 11:  # front right lower
                profiler.set_velocity_and_acceleration()
                leg_y = -2 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )

            elif sub_state == 12:  # shift body weight forward while moving rear leg
                leg_y = -3 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                )
            elif sub_state == 13:  # rear left lift
                profiler.set_velocity_and_acceleration(step_velocity_max, step_acceleration)
                leg_y = -3 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z - left_leg_z_step,
                )
            elif sub_state == 14:  # rear left forward
                leg_y = -3 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y + leg_y_step, rear_left_z=leg_z - left_leg_z_step,
                )
            elif sub_state == 15:  # rear left lower
                profiler.set_velocity_and_acceleration()
                leg_y = -3 * body_y_step
                profiler.add_position_target(
                    front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                    front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                    rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                    rear_left_x=0, rear_left_y=leg_y + leg_y_step, rear_left_z=leg_z,
                )
            elif sub_state == 16:
                if crawl_velocity < 0:  # keep crawling
                    sub_state = 1
                elif crawl_velocity > 0:  # keep crawling
                    sub_state = 15
                else:  # stop stop crawling and go back to starting pose to
                    sub_state = 0

            if sub_state != 0:
                if crawl_velocity < 0:
                    sub_state += 1
                elif crawl_velocity > 0:
                    sub_state -= 1
                else:
                    # no command so find quickest way out of crawl
                    if sub_state > 7:
                        sub_state += 1
                    else:
                        sub_state -= 1

    return sub_state
