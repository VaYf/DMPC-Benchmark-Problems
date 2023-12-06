##############################################################################################
# This script demonstrates how the data for a DMPC benchmark instance is read in Julia and how
# the corresponding JuMP model is built
##############################################################################################
## Load module
include("DMPCBenchmarks.jl")
## Specify benchmark parametes
Ns  = 10  # Number of subsystems
nx  = 2   # Number of states
nu  = 2   # Number of control inputs
nr  = 2   # Number of shared resources
Np  = 10  # Prediction horizon
Run = 1   # Problem instance 

## Read data
Data = DMPCBenchmarks.read(Ns,nx,nu,nr,Np,Run)

## Build model of system 1 using JuMP
# Specify data
A     = Data["System 1"]["A"]     # Dynamics matrix
B     = Data["System 1"]["B"]     # Input matrix
Hx    = Data["System 1"]["Hx"]    # State objective function weight
Hu    = Data["System 1"]["Hu"]    # Input obctive function weight
Gx    = Data["System 1"]["Gx"]    # State constraint matrix
p_x   = Data["System 1"]["p_x"]   # State constraint right-hand side
Gu    = Data["System 1"]["Gu"]    # Input constraint matrix
p_u   = Data["System 1"]["p_u"]   # Input constraint right-hand side
x_ref = Data["System 1"]["x_ref"] # State reference trajectory
x0    = Data["System 1"]["x0"]    # State initial condition
R     = Data["System 1"]["R"]     # Resource utilization matrix
# Create empty model (optimizer needs to be attached)
using JuMP
m = JuMP.Model()
# Define variables
@variable(m, x[1:nx,1:Np])
@variable(m, u[1:nu,1:Np-1])
@variable(m, Resources[1:nr*(Np-1)])
# Define Objective Function
@objective(m, Min,
   # Terminal cost
   (x[:,Np]-x_ref[:,Np])'Hx*(x[:,Np]-x_ref[:,Np]) +
   # Reference Tracking
   ∑((x[:,k]-x_ref[:,k])'Hx*(x[:,k]-x_ref[:,k]) + u[:,k]'*Hu*u[:,k] for k = 1:Np-1))
 # Dynamics
 for k = 1:Np-1
     @constraint(m,
     x[:,k+1] .== A*x[:,k+1] + B*u[:,k]
     )
 end
# Convex Constraints (number of constraints = number of states)
for c = 1:nx
    # States
    for k = 1:Np
        @constraint(m,
        x[:,k]'*Gx[c]*x[:,k] ≤ p_x[c]^2
        )
    end
     # Inputs
    for k = 1:Np-1
        @constraint(m,
        u[:,k]'*Gu[c]*u[:,k] ≤ p_u[c]^2
        )
    end
end
# Resources
global counter = 1
for j = 1:nr, k = 1:Np-1
    @constraint(m,
    Resources[counter] == R[j,:]'*u[:,k]
    )
    global counter += 1
end
