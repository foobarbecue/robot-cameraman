from collections import namedtuple

import serial
import struct

from simplebgc.command_ids import *
from simplebgc.command_names import get_incoming_command_name
from simplebgc.command_parser import parse_cmd
from simplebgc.commands import ControlOutCmd, BoardInfoInCmd, RawCmd

MessageHeader = namedtuple(
    'MessageHeader',
    'start_character command_id payload_size header_checksum')

MessagePayload = namedtuple(
    'MessagePayload',
    'payload payload_checksum')

Message = namedtuple(
    'Message',
    'start_character command_id payload_size header_checksum payload payload_checksum')


def pack_control_cmd(cmd: ControlOutCmd) -> bytes:
    return struct.pack('<BBBhhhhhh', *cmd)


def pack_board_info_cmd(cmd: BoardInfoInCmd) -> bytes:
    return struct.pack('<BHBHBI7s', *cmd)


def create_message(command_id: int, payload: bytes = b'') -> Message:
    payload_size = len(payload)
    return Message(start_character=ord('>'),
                   command_id=command_id,
                   payload_size=payload_size,
                   header_checksum=(command_id + payload_size) % 256,
                   payload=payload,
                   payload_checksum=sum(payload) % 256)


def pack_message(message: Message) -> bytes:
    message_format = '<BBBB{}sB'.format(message.payload_size)
    return struct.pack(message_format, *message)


def unpack_message(data: bytes, payload_size: int) -> Message:
    message_format = '<BBBB{}sB'.format(payload_size)
    return Message._make(struct.unpack(message_format, data))


def read_message(connection: serial.Serial, payload_size: int) -> Message:
    # 5 is the length of the header + payload checksum byte
    # 1 is the payload size
    response_data = connection.read(5 + payload_size)
    # print('received response', response_data)
    return unpack_message(response_data, payload_size)


def read_message_header(connection: serial.Serial) -> MessageHeader:
    header_data = connection.read(4)
    print('received message header data', header_data)
    return MessageHeader._make(struct.unpack('<BBBB', header_data))


def read_message_payload(connection: serial.Serial,
                         payload_size: int) -> MessagePayload:
    # +1 because of payload checksum
    payload_data = connection.read(payload_size + 1)
    print('received message payload data', payload_data)
    payload_format = '<{}sB'.format(payload_size)
    return MessagePayload._make(struct.unpack(payload_format, payload_data))


def read_cmd(connection: serial.Serial) -> RawCmd:
    header = read_message_header(connection)
    print('parsed message header', header)
    assert header.start_character == 62
    assert (
                   header.command_id + header.payload_size) % 256 == header.header_checksum
    payload = read_message_payload(connection, header.payload_size)
    print('parsed message payload', payload)
    assert sum(payload.payload) % 256 == payload.payload_checksum
    return RawCmd(header.command_id, payload.payload)


def rotate_gimbal(yaw_speed: int = 0) -> None:
    control_data = ControlOutCmd(roll_mode=1, roll_speed=0, roll_angle=0,
                                 pitch_mode=1, pitch_speed=0, pitch_angle=0,
                                 yaw_mode=1, yaw_speed=yaw_speed, yaw_angle=0)
    # print('command to send:', control_data)
    packed_control_data = pack_control_cmd(control_data)
    # print('packed command as payload:', packed_control_data)
    message = create_message(CMD_CONTROL, packed_control_data)
    # message = create_message(CMD_BOARD_INFO)
    # message = create_message(CMD_BOARD_INFO_3)
    # message = create_message(CMD_READ_PARAMS_3)
    # message = create_message(CMD_READ_PARAMS_EXT)
    # message = create_message(CMD_READ_PARAMS_EXT2)
    # message = create_message(CMD_REALTIME_DATA_4)
    # print('created message:', message)
    packed_message = pack_message(message)
    # print('packed message:', packed_message)

    connection = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=10)
    # print('send packed message:', packed_message)
    connection.write(packed_message)
    message = read_message(connection, 1)
    # print('received confirmation:', message)
    # print('confirmed command with ID:', ord(message.payload))
    # cmd = read_cmd(connection)
    # print('incoming command:', get_incoming_command_name(cmd.id))
    # print('incoming command payload length:', len(cmd.payload))
    # print(parse_cmd(cmd))


if __name__ == '__main__':
    # rotate_gimbal(yaw_speed=-100)
    # rotate_gimbal(yaw_speed=100)
    rotate_gimbal(yaw_speed=0)

# 1u – 1 byte unsigned
# 1s – 1 byte signed
# 2u – 2 byte unsigned (little-endian order)
# 2s – 2 byte signed (little-endian order)
# 4f – float (IEEE-754 standard)
# 4s – 4 bytes signed (little-endian order)
# string – ASCII character array, first byte is array size
# Nb – byte array size N

# CMD_BOARD_INFO
# >   id  size  check  payload  check
# 62  86    1    87       1       1
# 62  86    0    86              0