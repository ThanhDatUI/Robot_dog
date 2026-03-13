def get_rc_command(sbus_uart):
    sbus_buffer = bytearray(50)
    sbus_uart.readinto(sbus_buffer)

    start_byte_index = None
    for sbus_index in range(len(sbus_buffer) - 24):
        if sbus_buffer[sbus_index] == 0x0F and sbus_buffer[sbus_index + 24] == 0x00:
            start_byte_index = sbus_index
            break

    if start_byte_index is None:
        channels = None
    else:
        channels = []
        sbus_frame = sbus_buffer[start_byte_index:start_byte_index + 25]
        sbus_as_int = int.from_bytes(sbus_frame, "little")

        for channel_index in range(16):
            shift = (8 + (channel_index * 11))
            channel_value = (sbus_as_int & (0x7FF << shift)) >> shift
            channels.append(channel_value)
        for bit_index in range(4):  # digital channels 17, 18, frame lost flag, failsafe flag
            channels.append(bool((sbus_frame[-2] & (0x1 << bit_index)) >> bit_index))

        if (channels[-2] is True) or (channels[-1] is True):  # if error bits are set throw it out
            channels = None

    return channels
