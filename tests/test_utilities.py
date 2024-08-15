import pandas as pd
import unittest

from utilities.interpolation import Interpolator

class TestInterpolator(unittest.TestCase):
    def setUp(self):
        # Sample data similar to what's used in the main module
        data = {
            'x1': [1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3],
            'x2': [1, 1, 2, 2, 1, 1, 2, 2, 1, 1, 2, 2],
            'x3': [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2],
            'y1': [10, 15, 12, 18, 14, 20, 16, 22, 18, 24, 20, 26],
            'y2': [20, 25, 22, 28, 24, 30, 26, 32, 28, 34, 30, 36]
        }
        df = pd.DataFrame(data)
        self.interpolator = Interpolator(n_independent_vars=3, dataframe=df)

    def test_interpolation_correctness(self):
        # Test interpolation at a known point
        input_value = (2, 2, 2)  # Known point
        expected_output = [22, 32]  # Corresponding outputs for y1 and y2
        result = self.interpolator.interpolate(input_value)
        self.assertEqual(result, expected_output)

    def test_intermediate_point(self):
        # Test interpolation at an intermediate point
        input_value = (1.5, 1.5, 1.5)  # Intermediate point
        expected_output = [15.875, 25.875]  # Expected interpolated outputs for y1 and y2
        result = self.interpolator.interpolate(input_value)
        self.assertEqual(result, expected_output)

    def test_interpolation_edge_case(self):
        # Test interpolation at the boundary of the data
        input_value = (1, 1, 1)  # Boundary point
        expected_output = [10, 20]  # Known outputs for y1 and y2
        result = self.interpolator.interpolate(input_value)
        self.assertEqual(result, expected_output)

    def test_interpolation_out_of_bounds(self):
        # Test interpolation outside the grid range
        input_value = (4, 4, 4)  # Out of bounds
        with self.assertRaises(ValueError):
            self.interpolator.interpolate(input_value)

    # TODO: write test over a plotted table data to verify visually

if __name__ == '__main__':
    unittest.main()