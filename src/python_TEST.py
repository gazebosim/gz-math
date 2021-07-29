import unittest
import ignition.math 
class TestVector3(unittest.TestCase):

    def test_construction(self):
        angle1 = ignition.math.Angle()
        self.assertEqual(angle1.Radian(), 0.0)
        v1 = ignition.math.Vector3d(0, 0, 0)
        self.assertEqual(v1, ignition.math.Vector3d.Zero)
        v2 = ignition.math.Vector2d(1, 2)
        self.assertEqual(v2.X(), 1)
        self.assertEqual(v2.Y(), 2)

if __name__ == '__main__':
    unittest.main()
