"""Solvers."""

from functools import reduce

import autograd.numpy as np

from .component import Joint

import pyquaternion

import pdb

class FKSolver(object):
    """A forward kinematics solver."""

    def __init__(self, components):
        """Generate a FK solver from link and joint instances."""
        joint_indexes = [
            i for i, c in enumerate(components) if isinstance(c, Joint)
        ]

        def matrices(angles):
            joints = dict(zip(joint_indexes, angles))
            a = [joints.get(i, None) for i in range(len(components))]
            return [c.matrix(a[i]) for i, c in enumerate(components)]

        self._matrices = matrices

    def solve(self, angles):
        """Calculate a position of the end-effector and return it."""
        pt = np.array([
            [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]
            ])
        for m in reversed(self._matrices(angles)):
            pt = np.dot(m, pt)
        q = pyquaternion.Quaternion(matrix=pt).elements
        end_pose = np.hstack((q, pt[:3,3]))
        return end_pose


class IKSolver(object):
    """An inverse kinematics solver."""

    def __init__(self, fk_solver, optimizer):
        """Generate an IK solver from a FK solver instance."""
        print "Initializing IKSolver"
        def distance_squared(angles, target):
            x = target - fk_solver.solve(angles)
            return np.sum(np.power(x, 2))

        def distance_squared_sub(angles, target):
            # only care about some items - like first three values in quaternion
            x = target[:3] - fk_solver.solve(angles)[:3]
            return np.sum(np.power(x, 2))

        optimizer.prepare(distance_squared_sub)
        self.optimizer = optimizer

    def solve(self, angles0, target):
        """Calculate joint angles and returns it."""
        return self.optimizer.optimize(np.array(angles0), target)
