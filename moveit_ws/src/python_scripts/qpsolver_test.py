import numpy as np
import qpsolvers
from qpsolvers import solve_qp
from scipy.sparse import csc_matrix


print("The available solvers are %s " % qpsolvers.available_solvers)

# P = np.array([[2, 0], [0, 2]])
# q = np.array([1, 1])
# G = np.array([[2, 0], [0, 2]])
# h = np.array([1, 0])
# A = np.array([0, 1])
# b = np.array([0])


# M = np.array([[1.0, 2.0, 0.0], [-8.0, 3.0, 2.0], [0.0, 1.0, 1.0]])
# P = M.T @ M  # this is a positive definite matrix
# q = np.array([3.0, 2.0, 3.0]) @ M
# G = np.array([[1.0, 2.0, 1.0], [2.0, 0.0, 1.0], [-1.0, 2.0, -1.0]])
# h = np.array([3.0, 2.0, -2.0])
# A = np.array([1.0, 1.0, 1.0])
# b = np.array([1.0])


row = np.array([0, 1])
col = np.array([0, 1])
data = np.array([1, 1])
P = csc_matrix((data, (row, col)), shape=(2, 2))

q = np.array([1, 1])
lb = np.array([-5, -5])
ub = np.array([-1, -1])

# initialize matrix using scipy sparse matrix CSC format
P = csc_matrix((data, (row, col)), shape=(2, 2))
# G = csc_matrix((data, (row, col)), shape=(2, 2))

x = solve_qp(P, q=None, G=None, h=None, A=None, b=None, lb=lb, ub=ub, solver='osqp')

print("QP solution: x = %s" % x)
