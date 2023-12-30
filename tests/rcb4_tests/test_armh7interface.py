import time
import unittest

import numpy as np

from rcb4.armh7interface import ARMH7Interface


class TestRobotModel(unittest.TestCase):

    interface = None

    @classmethod
    def setUpClass(cls):
        cls.interface = ARMH7Interface()
        cls.interface.auto_open()
        cls.interface.search_servo_ids()

    def test_servo_angle_vector(self):
        self.interface.hold()
        self.interface.neutral()
        self.interface.servo_angle_vector(
            [32, 34],
            [8000, 8000],
            velocity=0)
        time.sleep(4.0)
        if np.any(np.abs(self.interface.angle_vector() - 16) > 2.0):
            self.assertRaises()
        self.interface.neutral()
        time.sleep(4.0)
        self.interface.free()
