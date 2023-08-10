using JLD2, FileIO
Ns  = 10  # Number of subsystems
nx  = 2   # Number of states
nu  = 2   # Number of control inputs
nr  = 2   # Number of shared resources
Np  = 10  # Prediction horizon
Run = 1   # Problem instance 

# Read data
File = string("DMPC_Ns_",Ns,"_nx_",nx,"_nu_",nu,"_nr_",nr,"_Np_",Np,"_Run_",Run,".jld2")
Data = FileIO.load(File)
##############################################################################################
## Data entries
# Data["r_max]:     Maximum availability of resources
# Data["System i"]: Data of subsystem i
##############################################################################################
## Subproblem Format
# A     Dynamics matrix
# B     Input matrix
# Hx    State objective function weight
# Hu    Input obctive function weight
# Gx    State constraint matrix
# p_x   State constraint right-hand side
# Gu    Input constraint matrix
# p_u   Input constraint right-hand side
# x_ref State reference trajectory
# x0    State initial condition
# R     Resource utilization matrix

# nx = size(A,1)        Number of states
# nu = size(B,2)        Number of inputs
# nc = length(Gx)       Number of constraints
# nr = size(R,1)        Number of resources
# Np = size(x_ref,2)    Prediction horizon
###############################################
# # Define variables
# @variable(m, x[1:nx,1:Np])
# @variable(m, u[1:nu,1:Np-1])
# @variable(m, Resources[1:nr*(Np-1)])
# # Define Symbolic Objective Value
# @objective(m, Min,
#   # Terminal cost
#   (x[:,Np]-x_ref[:,Np])'Hx*(x[:,Np]-x_ref[:,Np]) +
#   # Reference Tracking
#   ∑((x[:,k]-x_ref[:,k])'Hx*(x[:,k]-x_ref[:,k]) + u[:,k]'*Hu*u[:,k] for k = 1:Np-1))
# # Dynamics
# for k = 1:Np-1
#     @constraint(m,
#     x[:,k+1] .== A*x[:,k+1] + B*u[:,k]
#     )
# end
# # Convex Constraints
# for c = 1:nc
#     # States
#     for k = 1:Np
#         @constraint(m,
#         x[:,k]'*Gx[c]*x[:,k] ≤ p_x[c]^2
#         )
#     end
#     # Inputs
#     for k = 1:Np-1
#         @constraint(m,
#         u[:,k]'*Gu[c]*u[:,k] ≤ p_u[c]^2
#         )
#     end
# end
# # Resources
# global counter = 1
# for j = 1:nr, k = 1:Np-1
#     @constraint(m,
#     Resources[counter] == R[j,:]'*u[:,k]
#     )
#     global counter += 1
# end