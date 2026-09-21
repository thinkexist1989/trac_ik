import math
import pathlib
import sys
import unittest
from pin_ik_python.pin_ik import IK

XML = pathlib.Path(sys.argv.pop()).read_text()
class StandaloneTest(unittest.TestCase):
    def test_modes_and_limits(self):
        for mode in ('Speed', 'Distance', 'Manipulation1', 'Manipulation2', 'Manipulation3'):
            ik = IK('base', 'tip', urdf_string=XML, timeout=0.03, solve_type=mode)
            self.assertEqual(ik.joint_names, ('x', 'y', 'z', 'roll', 'pitch', 'yaw'))
            self.assertEqual(len(ik.link_names), 8)
            # RPY(0.1, 0.2, 0.3) fixed tool orientation, zero moving rotations.
            r, p, y = 0.05, 0.1, 0.15
            quat = (math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y),
                    math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y),
                    math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y),
                    math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y))
            result = ik.get_ik([0]*6, 0.4, 0, 0.7, *quat, brx=0, bry=0, brz=0)
            self.assertIsNotNone(result)
            for actual, expected in zip(result, [0.3,-0.2,0.4,0,0,0]):
                self.assertAlmostEqual(actual, expected, delta=1e-4)
            lo, hi = map(list, ik.get_joint_limits())
            self.assertEqual(lo[0], -1.0)
            lo[0], hi[0] = -0.5, 0.5
            ik.set_joint_limits(lo, hi)
            self.assertEqual(ik.get_joint_limits()[0][0], -0.5)
            self.assertIsNone(ik.get_ik([0]*6, 10,10,10,0,0,0,1))
    def test_continuous_winding(self):
        xml = """<robot name="spin"><link name="base"/><link name="tip"/>
          <joint name="spin" type="continuous"><parent link="base"/>
          <child link="tip"/><axis xyz="0 0 1"/></joint></robot>"""
        ik = IK('base', 'tip', urdf_string=xml, timeout=0.1)
        self.assertEqual(ik.number_of_joints, 1)
        self.assertEqual(ik.get_joint_limits(), ((-math.inf,), (math.inf,)))
        angle = -math.pi + 0.04
        seed = 11 * math.pi - 0.04
        result = ik.get_ik([seed], 0, 0, 0, 0, 0,
                           math.sin(angle / 2), math.cos(angle / 2),
                           brx=0, bry=0, brz=0)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[0], seed + 0.08, delta=1e-4)

    def test_bad_inputs(self):
        for kwargs in ({}, {'urdf_string':'bad'}, {'urdf_string':XML, 'solve_type':'bad'}):
            with self.assertRaises(ValueError): IK('base','tip',**kwargs)
        ik = IK('base','tip',urdf_string=XML)
        with self.assertRaises(ValueError): ik._ik_solver.CartToJnt([0],0,0,0,0,0,0,1)
        with self.assertRaises(ValueError): ik.get_ik([0]*6,0,0,0,0,0,0,0)

unittest.main()
