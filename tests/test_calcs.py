import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R

from simulation.models.airdata import airdata

class TestCalcAeroOutputs(unittest.TestCase):
    def test_zero_velocity(self):
        V_body = np.array([0, 0, 0])
        V_wind_NED = np.array([0, 0, 0])
        body_orientation = np.array([1, 0, 0, 0])  # No rotation
        airspeed, alpha, beta, Va_body = airdata(V_body, V_wind_NED, body_orientation)
        self.assertAlmostEqual(airspeed, 0)
        self.assertAlmostEqual(alpha, 0)
        self.assertAlmostEqual(beta, 0)

    def test_non_zero_velocity(self):
        V_body = np.array([10, 0, 0])
        V_wind_NED = np.array([5, 0, 0])
        body_orientation = np.array([1, 0, 0, 0])  # No rotation
        airspeed, alpha, beta, Va_body = airdata(V_body, V_wind_NED, body_orientation)
        self.assertAlmostEqual(airspeed, 5)
        self.assertAlmostEqual(alpha, 0)
        self.assertAlmostEqual(beta, 0)
        self.assertTrue(np.allclose(Va_body, [5, 0, 0], atol=1e-6))

    def test_with_wind_and_rotation(self):
        V_body = np.array([10, 0, 0])
        V_wind_NED = np.array([0, 5, 0])
        body_orientation = np.array([np.cos(np.pi/4), 0,0, np.sin(np.pi/4)])  # 90 degrees rotation around z-axis
        airspeed, alpha, beta, Va_body = airdata(V_body, V_wind_NED, body_orientation)
        self.assertAlmostEqual(airspeed, 5.0, places=5)
        self.assertAlmostEqual(alpha, 0, places=5)
        self.assertAlmostEqual(beta, 0, places=5)
        self.assertTrue(np.allclose(Va_body, [5, 0, 0], atol=1e-6))

    def test_arbitrary_orientation(self):
        V_body = np.array([10, 2, -4])
        V_wind_NED = np.array([-2, 5, 10])

        angle_z = np.deg2rad(30)  # 30 degrees
        angle_y = np.deg2rad(45)  # 45 degrees
        angle_x = np.deg2rad(60)  # 60 degrees
        euler_angles = [angle_z, angle_y, angle_x]

        body_orientation = R.from_euler('zyx', euler_angles, degrees=False)
        quaternion = body_orientation.as_quat()
        quaternion = np.roll(quaternion, 1) # Rotation package puts real part last
        airspeed, alpha, beta, Va_body = airdata(V_body, V_wind_NED, quaternion)
        self.assertAlmostEqual(airspeed, 10.960667531, places=5)
        self.assertAlmostEqual(alpha, -0.46787239, places=5)
        self.assertAlmostEqual(beta, -0.9034154, places=5)

if __name__ == '__main__':
    unittest.main(verbosity=2)
