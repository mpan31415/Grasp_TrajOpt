import osqp
import numpy as np
from scipy.sparse import csc_matrix

###########################################################################################
# initialize vectors
q = np.array([1, 1])
l = np.array([-1, -2])
u = np.array([1, 0])
# q_new = q
# l_new = l
# u_new = u

# the indices of all the entries in the matrices, and the entries themselves
row = np.array([0, 1])
col = np.array([0, 1])
data = np.array([1, 1])

# initialize matrix using scipy sparse matrix CSC format
P = csc_matrix((data, (row, col)), shape=(2, 2))
A = csc_matrix((data, (row, col)), shape=(2, 2))

print(P.toarray())
print(A.toarray())


###########################################################################################
# initialize solver by creating OSQP object
m = osqp.OSQP()

# specify problem in the setup phase
m.setup(P, q, A, l, u)

# solve the problem
results = m.solve()

# access results
primal_sol = results.x
print("The primal solution is %s" % primal_sol)

dual_sol = results.y
print("The dual solution is %s" % dual_sol)

info = results.info

iterations = info.iter
print("The number of iterations is %d" % iterations)

objective_value = info.obj_val
solve_time = info.solve_time
print("Problem solved in %.2f seconds" % solve_time)


###########################################################################################
# can update problem vectors without new setup
# m.update(q=q_new, l=l_new, u=u_new)


###########################################################################################
# can auto-generate C code (no malloc / external libraries / division in ADMM algo)
# m.codegen('code')
