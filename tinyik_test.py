import tinyik
import numpy as np
import matplotlib.pyplot as plt
import pyquaternion
import time
import pdb
from tinyik.optimizer import NewtonOptimizer, ScipyOptimizer
from scipy.interpolate import interp1d

def random_ee():
	return np.hstack( (pyquaternion.Quaternion.random().elements, np.random.rand(3)*2 + 1 ) )

def test_speed(arm, cycle_count):
	t = np.ones(cycle_count)
	for i in range(cycle_count):
		now = time.time()
		arm.ee = random_ee()
		t[i] = time.time() - now

	print("Average Time for {} Random Solutions: {}".format(cycle_count, np.mean(t)))


np.random.seed(seed = 123)
arm = tinyik.Actuator([
				'z', [0., 0., 1.], 
				'x', [1., 0., 0.],
				'y', [0., 1., 0.],
				'z', [0., 0., 1.], 
				'x', [1., 0., 0.],
				'y', [0., 1., 0.],
				'z', [0., 0., 1.],
				],
				optimizer = ScipyOptimizer()
				)

print("Angles: {} -- EE: {}".format(arm.angles, arm.ee))
arm.angles = [np.pi/2, 0, 0, 0, 0, 0, 0]
print("Angles: {} -- EE: {}".format(arm.angles, arm.ee))
arm.angles = [0, 0, 0, 0, 0, np.pi/2, 0]
print("Angles: {} -- EE: {}".format(arm.angles, arm.ee))
des_ee = np.hstack( (pyquaternion.Quaternion.random().elements, [2, 0, 0] ) )
arm.ee = des_ee
print("Desired EE: {}".format(np.round(des_ee,2)))
print("Angles: {} -- EE: {}".format(np.round(arm.angles,2), np.round(arm.ee,2)))


# test_speed(arm, 100)

# plan a path
start = random_ee()
end = random_ee()

pts_in_path = 10
q_gen = pyquaternion.Quaternion.intermediates(start[:4], end[:4], pts_in_path, include_endpoints=True)
p_gen = interp1d([0,1], np.vstack([start, end]), axis=0)
for t in range(pts_in_path):
	q = next(q_gen)
	p = p_gen(float(t)/pts_in_path)
	



