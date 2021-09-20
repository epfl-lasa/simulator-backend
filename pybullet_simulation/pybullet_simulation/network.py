from dataclasses import dataclass

import clproto
import state_representation as sr
import zmq


@dataclass
class StateMessage:
    """
    A collection of state variables in Cartesian and joint space that the robot publishes.
    """
    ee_state: sr.CartesianState
    joint_state: sr.JointState


@dataclass
class CommandMessage:
    """
    A collection of state variables in Cartesian and joint space that the
    robot follows according to the specified control type.
    """
    ee_state: sr.CartesianState
    joint_state: sr.JointState


def encode_state(state):
    """
    Encode a state message into the serialized binary format.
    :param state: The StateMessage to encode
    :type state: StateMessage
    :return: An ordered vector of encoded strings representing the state message fields
    :rtype: list of str
    """
    encoded_state = list()
    encoded_state.append(clproto.encode(state.ee_state, clproto.MessageType.CARTESIAN_STATE_MESSAGE))
    encoded_state.append(clproto.encode(state.joint_state, clproto.MessageType.JOINT_STATE_MESSAGE))
    return encoded_state


def encode_command(command):
    """
    Encode a command message into the serialized binary format.
    :param command: The CommandMessage to encode
    :type command: CommandMessage
    :return: An ordered vector of encoded strings representing the command message fields
    :rtype: list of str
    """
    encoded_command = list()
    encoded_command.append(clproto.encode(command.ee_state, clproto.MessageType.CARTESIAN_STATE_MESSAGE))
    encoded_command.append(clproto.encode(command.joint_state, clproto.MessageType.JOINT_STATE_MESSAGE))
    return encoded_command


def decode_state(message):
    """
    Decode a state message from the serialized binary format.
    :param message: An ordered vector of encoded strings representing the state message fields
    :type message: list of str
    :return: The equivalent StateMessage
    :rtype: StateMessage
    """
    state = StateMessage(clproto.decode(message[0]), clproto.decode(message[1]))
    return state


def decode_command(message):
    """
    Decode a command message from the serialized binary format.
    :param message: An ordered vector of encoded strings representing the state message fields
    :type message: list of str
    :return: The equivalent CommandMessage
    :rtype: CommandMessage
    """
    command = CommandMessage(clproto.decode(message[0]), clproto.decode(message[1]))
    return command


def configure_subscriber(context, subscriber_uri):
    """
    Configure the zmq subscriber socket to receive messages.
    :param context: The ZMQ context object
    :param subscriber_uri: The URI (IP:Port) of the network socket
    :type context: zmq.Context
    :type subscriber_uri: str
    :return: The subscription socket that is used to receive messages
    :rtype: zmq.Socket
    """
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    subscriber.connect("tcp://" + subscriber_uri)
    return subscriber


def configure_publisher(context, publisher_uri):
    """
    Configure the zmq publisher socket to publish messages.
    :param context: The ZMQ context object
    :param publisher_uri: The URI (IP:Port) of the network socket
    :type context: zmq.Context
    :type publisher_uri: str
    :return: The publication socket that is used to send messages
    :rtype: zmq.Socket
    """
    publisher = context.socket(zmq.PUB)
    publisher.bind("tcp://" + publisher_uri)
    return publisher


def configure_sockets(context, subscriber_uri, publisher_uri):
    """
    Configure the zmq sockets to publish and receive messages.
    :param context: The ZMQ context object
    :param subscriber_uri: The URI (IP:Port) of the network socket
    :param publisher_uri: The URI (IP:Port) of the network socket
    :type context: zmq.Context
    :type subscriber_uri: str
    :type publisher_uri: str
    :return: The subscription and publication socket that is used to receive and send messages
    :rtype: tuple[zmq.Socket]
    """
    return configure_subscriber(context, subscriber_uri), configure_publisher(context, publisher_uri)


def send_encoded_fields(encoded_fields, publisher):
    """
    Send a sequence of encoded field messages.
    :param encoded_fields: An ordered vector of encoded fields
    :param publisher: The configured ZMQ publisher socket
    :type encoded_fields: list of str
    :type publisher: zmq.Socket
    """
    packet = clproto.pack_fields(encoded_fields)
    publisher.send(packet)


def send_state(state, publisher):
    """
    Send a state message.
    :param state: The StateMessage to publish
    :param publisher: The configured ZMQ publisher socket
    :type state: StateMessage
    :type publisher: zmq.Socket
    """
    send_encoded_fields(encode_state(state), publisher)


def send_command(command, publisher):
    """
    Send a command message.
    :param command: The CommandMessage to publish
    :param publisher: The configured ZMQ publisher socket
    :type command: CommandMessage
    :type publisher: zmq.Socket
    """
    send_encoded_fields(encode_command(command), publisher)


def poll_encoded_fields(subscriber):
    """
    Poll the socket for a sequence of encoded field messages.
    :param subscriber: The configured ZMQ subscriber socket
    :type subscriber: zmq.Socket
    :return: A vector of encoded fields if a message is available, empty otherwise
    :rtype: list of str
    """
    try:
        message = subscriber.recv(zmq.DONTWAIT)
    except zmq.error.Again:
        return []

    fields = []
    if message:
        fields = clproto.unpack_fields(message)
    return fields


def poll_state(subscriber):
    """
    Poll the socket for a state message.
    :param subscriber: The configured ZMQ subscriber socket
    :type subscriber: zmq.Socket
    :return: The StateMessage object if a message is available
    :rtype: StateMessage
    """
    fields = poll_encoded_fields(subscriber)
    if fields:
        return decode_state(fields)


def poll_command(subscriber):
    """
    Poll the socket for a command message.
    :param subscriber: The configured ZMQ subscriber socket
    :type subscriber: zmq.Socket
    :return: The CommandMessage object if a message is available
    :rtype: CommandMessage
    """
    fields = poll_encoded_fields(subscriber)
    if fields:
        return decode_command(fields)
