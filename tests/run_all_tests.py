import unittest

# Import test modules
from test_calcs import TestCalcAeroOutputs
from test_utilities import TestInterpolator

# Initialize a test suite
loader = unittest.TestLoader()
suite = unittest.TestSuite()

# Add tests to the suite
suite.addTests(loader.loadTestsFromTestCase(TestCalcAeroOutputs))
suite.addTests(loader.loadTestsFromTestCase(TestInterpolator))

# Run the test suite
runner = unittest.TextTestRunner(verbosity=2)
runner.run(suite)
