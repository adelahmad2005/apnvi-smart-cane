"""Log Stage 1's rate in 5 s windows, plus free memory, while a load test runs.

  python3 loadtest_logger.py OUT_PREFIX MINUTES
Writes OUT_PREFIX_stage1.csv (time, levels in the window, levels a second) and
OUT_PREFIX_free.txt. Run tegrastats next to it: tegrastats --interval 1000 > file.
"""

import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WINDOW_S = 5.0


class Counter(Node):
    """Count the messages on /hazard/level."""

    def __init__(self):
        super().__init__('loadtest_counter')
        self.count = 0
        self.create_subscription(String, '/hazard/level', self.on_level, 50)

    def on_level(self, _msg):
        self.count += 1


def main():
    prefix, minutes = sys.argv[1], float(sys.argv[2])
    rclpy.init()
    node = Counter()
    end = time.time() + minutes * 60
    with open(prefix + '_stage1.csv', 'w') as csv, open(prefix + '_free.txt', 'w') as free:
        csv.write('time,levels,per_second\n')
        while time.time() < end:
            node.count = 0
            window_end = time.time() + WINDOW_S
            while time.time() < window_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            csv.write('%s,%d,%.2f\n' % (time.strftime('%H:%M:%S'), node.count,
                                        node.count / WINDOW_S))
            csv.flush()
            free.write(time.strftime('%H:%M:%S') + '\n' + subprocess.run(
                ['free', '-m'], capture_output=True, text=True).stdout)
            free.flush()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
