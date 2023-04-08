import casadi as cs
import numpy as np
import matplotlib.pyplot as plt

# Define the system dynamics
A = np.array([[1, 1], [0, 1]])
B = np.array([[0], [1]])
C = np.array([[1, 0], [0, 1]])
D = np.array([[0], [0]])

# Define the MPC parameters
N = 5   
dt = 0.1
Q = np.diag([1, 1])
R = np.array([[1]])

# Define the optimization problem
opti = cs.Opti()

# Define the state variables
x = opti.variable(2, N+1)
x0 = opti.parameter(2, 1)

# Define the control variables
u = opti.variable(1, N)

# Define the reference trajectory
x_ref = opti.parameter(2, N+1)
u_ref = opti.parameter(1, N)

# Define the initial state constraint
opti.subject_to(x[:,0] == x0)

# Define the dynamic constraints
for k in range(N):
    x_next = cs.mtimes(A, x[:,k]) + cs.mtimes(B, u[:,k])
    opti.subject_to(x[:,k+1] == x_next)

# Define the cost function
J = 0
for k in range(N):
    J += cs.mtimes([(x[:,k] - x_ref[:,k]).T, Q, (x[:,k] - x_ref[:,k])])
    J += cs.mtimes([(u[:,k] - u_ref[:,k]).T, R, (u[:,k] - u_ref[:,k])])
opti.minimize(J)

# Define the control constraints
opti.subject_to(u <= 1)
opti.subject_to(u >= -1)

# Set the initial state parameter
x0_val = np.array([[0], [0]])
opti.set_value(x0, x0_val)

# Define the reference trajectory and control inputs
x_ref_val = np.zeros((2, N+1))
x_ref_val[0,:] = np.linspace(0, 1, N+1)
u_ref_val = np.zeros((1, N))
opti.set_value(x_ref, x_ref_val)
opti.set_value(u_ref, u_ref_val)

# Simulate the system and plot the results
x_val = np.zeros((2, N+1))
u_val = np.zeros((1, N))

for i in range(N):
    # Update the optimization problem with the current state
    opti.set_initial(u, u_val)
    opti.set_initial(x, x_val)

    # Solve the optimization problem
    opti.solver('ipopt')
    sol = opti.solve()

    # Extract the control input
    u_val = opti.value(u[:,0])

    # Update the system state
    x_val[:,i+1] = np.squeeze(cs.mtimes(A, x_val[:,i]) + cs.mtimes(B, u_val))

# Plot the results
plt.plot(x_ref_val[0,:], x_ref_val[1,:], 'r--', label='Reference')
plt.plot(x_val[0,:], x_val[1,:], 'b', label='MPC')
plt.legend()
plt.xlabel('x1')
plt.ylabel('x2')
plt.show()
