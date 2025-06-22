import pytest
from unittest.mock import MagicMock, patch
from car_controls.simple_driver import SimpleDriver

class MockMsg:
    def __init__(self, data):
        self.data = data

@pytest.fixture
def driver():
    with patch("rclpy.node.Node.__init__", lambda self, node_name: None), \
         patch.object(SimpleDriver, "create_subscription"), \
         patch.object(SimpleDriver, "create_publisher"), \
         patch.object(SimpleDriver, "get_logger", return_value=MagicMock()):
        driver = SimpleDriver()
    driver.get_logger = MagicMock()
    driver.create_subscription = MagicMock()
    driver.create_publisher = MagicMock(return_value=MagicMock())
    driver.publisher = MagicMock()
    return driver

def test_no_cones(driver):
    msg = MockMsg("")
    driver.listener_callback(msg)
    driver.publisher.publish.assert_called_once()
    published = driver.publisher.publish.call_args[0][0].data
    assert published == "0;0"
    driver.get_logger().warn.assert_called_once()

def test_only_blue_cones(driver):
    msg = MockMsg("B;10;0;20")
    driver.listener_callback(msg)
    published = driver.publisher.publish.call_args[0][0].data
    assert published == "8;-100"

def test_only_yellow_cones(driver):
    msg = MockMsg("Y;10;0;20")
    driver.listener_callback(msg)
    published = driver.publisher.publish.call_args[0][0].data
    assert published == "8;100"

def test_both_cones(driver):
    msg = MockMsg("B;100;0;200|Y;-100;0;200")
    driver.listener_callback(msg)
    published = driver.publisher.publish.call_args[0][0].data
    assert published == "8;0"
    
def test_both_cones_steer_left(driver):
    msg = MockMsg("B;150;0;200|Y;-50;0;200")
    driver.listener_callback(msg)
    published = driver.publisher.publish.call_args[0][0].data
    assert published.startswith("8;") and int(published.split(";")[1]) < 0

def test_both_cones_steer_right(driver):
    msg = MockMsg("B;50;0;200|Y;-150;0;200")
    driver.listener_callback(msg)
    published = driver.publisher.publish.call_args[0][0].data
    assert published.startswith("8;") and int(published.split(";")[1]) > 0
