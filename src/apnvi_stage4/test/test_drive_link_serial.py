"""The drive link against a fake USB port and a fake master: nothing real is opened."""

import json
import os
import queue
import threading
import time

from apnvi_stage4.drive_link_node import DriveLinkNode, find_port, SerialLink
from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import UInt8


class FakeMaster:
    """A fake serial port that answers like the master ESP32."""

    def __init__(self, path=''):
        self.path = path
        self.incoming = queue.Queue()
        self.written = []
        self.closed = False
        self.lock = threading.Lock()
        self.broken = False

    def say(self, *lines):
        for line in lines:
            self.incoming.put((line + '\n').encode())

    def readline(self):
        if self.broken:
            raise OSError('device disconnected')
        try:
            return self.incoming.get(timeout=0.05)
        except queue.Empty:
            return b''

    def write(self, data):
        if self.broken:
            raise OSError('device disconnected')
        with self.lock:
            self.written.extend(data.decode().splitlines())
        if b'who\n' in data:
            self.say('who master')

    def lines(self):
        with self.lock:
            return list(self.written)

    def close(self):
        self.closed = True


def wait_for(check, seconds=2.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.01)
    return False


def test_find_port_by_id(tmp_path):
    for name in ('usb-Silicon_Labs_CP2102N_lidar-if00-port0',
                 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'):
        (tmp_path / name).touch()
    found = find_port('Controller_0001', str(tmp_path))
    assert found.endswith('Controller_0001-if00-port0')
    assert find_port('nothing_like_it', str(tmp_path)) == ''


def test_find_port_refuses_two_matches(tmp_path):
    (tmp_path / 'usb-CP2102_Controller_0001-a').touch()
    (tmp_path / 'usb-CP2102_Controller_0001-b').touch()
    assert find_port('Controller_0001', str(tmp_path)) == ''   # which one? never guess


def test_serial_link_reads_and_writes():
    fake = FakeMaster()
    got = []
    link = SerialLink(lambda path: fake, got.append, log=lambda text: None)
    assert link.open('/fake')
    fake.say('btn red down')
    assert wait_for(lambda: got == ['btn red down'])
    assert link.write(['d 10 20', 'vibe 0'])
    assert fake.lines() == ['d 10 20', 'vibe 0']
    link.close()
    assert fake.closed and not link.is_open()


def test_serial_link_closes_when_the_port_breaks():
    fake = FakeMaster()
    link = SerialLink(lambda path: fake, lambda line: None, log=lambda text: None)
    link.open('/fake')
    fake.broken = True
    assert wait_for(lambda: not link.is_open())
    assert not link.write(['d 0 0'])


def test_serial_link_open_failure():
    def refuse(path):
        raise OSError('no such port')
    link = SerialLink(refuse, lambda line: None, log=lambda text: None)
    assert not link.open('/fake') and not link.is_open()


@pytest.fixture
def node():
    rclpy.init()
    fakes = []

    def opener(path):
        fakes.append(FakeMaster(path))
        return fakes[-1]
    made = DriveLinkNode(opener=opener, parameter_overrides=[
        Parameter('port', value='/fake/master'), Parameter('speed_at_cap_mps', value=0.5)])
    made.fakes = fakes
    yield made
    made.destroy_node()
    rclpy.shutdown()


def test_node_checks_the_master_then_drives_and_stops(node):
    node.tick()                                     # opens the (fake) port
    assert len(node.fakes) == 1 and node.fakes[0].path == '/fake/master'
    fake = node.fakes[0]
    node.tick()
    assert fake.lines() == []                       # the master is restarting: wait 2 s
    node.opened_at -= 3.0
    node.tick()                                     # asks "who"
    assert wait_for(lambda: node.link.master.is_master)
    assert 'who' in fake.lines() and wait_for(lambda: 'rear off' in fake.lines())

    fake.say('clear handle', 'clear jetson', 'ready')
    assert wait_for(lambda: node.link.master.ready)
    move = Twist()
    move.linear.x = 0.25
    node.on_cmd_vel(move)
    node.on_vibration(UInt8(data=40))
    node.tick()
    assert fake.lines()[-2:] == ['d 72 72', 'vibe 40']           # 0.25 m/s on the floor table

    fake.say('btn red down', 'stop red')            # red pressed: the master stops itself
    assert wait_for(lambda: 'red' in node.link.master.stops)
    node.on_cmd_vel(move)
    node.tick()
    assert fake.lines()[-2] == 'd 0 0'
    status = json.loads(node.link.status(True))
    assert status['stops'] == ['red'] and status['red']


def test_node_never_drives_a_board_that_is_not_the_master(node):
    node.tick()
    fake = node.fakes[0]
    fake.write = lambda data: fake.written.extend(data.decode().splitlines())  # never answers
    node.opened_at -= 3.0
    move = Twist()
    move.linear.x = 0.25
    node.on_cmd_vel(move)
    for _ in range(3):
        node.tick()
    assert all(not line.startswith('d ') for line in fake.lines())


def test_node_reconnects_after_the_usb_drops(node):
    node.tick()
    node.opened_at -= 3.0
    node.tick()
    assert wait_for(lambda: node.link.master.is_master)
    node.fakes[0].broken = True                     # USB drops under motor load
    assert wait_for(lambda: not node.serial.is_open())
    node.tick()                                     # notices, and tries again at once
    assert not node.link.master.is_master
    node.next_try = 0.0
    node.tick()
    assert len(node.fakes) == 2


def test_dry_run_opens_nothing():
    rclpy.init()
    opened = []
    try:
        made = DriveLinkNode(opener=opened.append,
                             parameter_overrides=[Parameter('dry_run', value=True)])
        made.tick()
        made.destroy_node()
    finally:
        rclpy.shutdown()
    assert opened == []


def test_default_master_name_is_a_by_id_name():
    from apnvi_stage4 import drive_link_node
    assert drive_link_node.BY_ID == '/dev/serial/by-id'
    assert 'ttyUSB' not in drive_link_node.MASTER_MATCH
    assert os.path.basename(drive_link_node.MASTER_MATCH) == drive_link_node.MASTER_MATCH
