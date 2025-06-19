import pytest
from unittest.mock import MagicMock, patch
from car_controls.keyboard_control_node import KeyboardControlNode
from std_msgs.msg import String

class DummyKey:
    def __init__(self, char):
        self.char = char

@pytest.fixture
def node():
    with patch("rclpy.node.Node.__init__", lambda self, name: None), \
         patch.object(KeyboardControlNode, "create_publisher"), \
         patch.object(KeyboardControlNode, "create_timer"), \
         patch("car_controls.keyboard_control_node.keyboard.Listener"), \
         patch.object(KeyboardControlNode, "get_logger", return_value=MagicMock()):
        n = KeyboardControlNode()
    # Patch publisher to store last message
    n.last_msg = None
    def fake_publish(msg):
        n.last_msg = msg
    publisher_mock = MagicMock()
    publisher_mock.publish.side_effect = fake_publish
    n.publisher = publisher_mock
    return n

def test_update_and_publish_left(node):
    node.last_msg = None
    node.pressed_keys = {'a'}
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    accel, steer = map(int, msg.data.split(";"))
    assert steer > 0
    assert accel == 0

def test_update_and_publish_no_keys(node):
    node.last_msg = None
    node.pressed_keys = set()
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    accel, steer = map(int, msg.data.split(";"))
    assert accel == 0
    assert steer == 0

def test_update_and_publish_forward_right(node):
    node.last_msg = None
    node.pressed_keys = {'w', 'd'}
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    accel, steer = map(int, msg.data.split(";"))
    assert accel > 0
    assert steer < 0

def test_update_and_publish_backward_left(node):
    node.last_msg = None
    node.pressed_keys = {'s', 'a'}
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    accel, steer = map(int, msg.data.split(";"))
    assert accel < 0
    assert steer > 0

def test_update_and_publish_all_keys(node):
    node.last_msg = None
    node.pressed_keys = {'w', 'a', 's', 'd'}
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    # Depending on your logic, this may cancel out to 0,0 or prioritize some keys
    accel, steer = map(int, msg.data.split(";"))
    # Accept any value, just check it's an int
    assert isinstance(accel, int)
    assert isinstance(steer, int)
    accel, steer = map(int, msg.data.split(";"))
    assert accel < 0
    assert steer == 0

def test_update_and_publish_left(node):
    node.last_msg = None
    node.pressed_keys = {'a'}
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    accel, steer = map(int, msg.data.split(";"))
    assert steer > 0
    assert accel == 0

def test_update_and_publish_right(node):
    node.last_msg = None
    node.pressed_keys = {'d'}
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    accel, steer = map(int, msg.data.split(";"))
    assert steer < 0
    assert accel == 0

def test_update_and_publish_forward_left(node):
    node.last_msg = None
    node.pressed_keys = {'w', 'a'}
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    accel, steer = map(int, msg.data.split(";"))
    assert accel > 0
    assert steer > 0

def test_update_and_publish_backward_right(node):
    node.last_msg = None
    node.pressed_keys = {'s', 'd'}
    node.update_and_publish()
    assert node.last_msg is not None, "No message was published"
    msg = node.last_msg
    accel, steer = map(int, msg.data.split(";"))
    assert accel < 0
    assert steer < 0