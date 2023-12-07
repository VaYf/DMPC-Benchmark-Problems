# DMPC-Benchmark-Problems
This repository containts a collection of benchmark problems for constraint-coupled distributed model predictive control. The benchmark problems have the following structure:
```math
\begin{align}
\underset{\mathbf{x}^{0:N_p},\mathbf{u}^{0:N_p-1}}{\min}\;&\sum_{i =1}^{N_s}\left[J_i^f(\mathbf{x}_i^{N_p}) + \sum_{k=0}^{N_p-1}J_i(\mathbf{x}_i^k,\mathbf{u}_i^k)\right],\\

\text{s.\,t. } &\mathbf{x}_i^{k+1} = \mathbf{A}_{i}\mathbf{x}_i^k + \mathbf{B}_{i}\mathbf{u}_i^k,\;\forall i \in \mathcal{I},  k=0,\dots,N_p -1,\\

&\mathbf{x}_i^0 = \tilde{\mathbf{x}}(t_0),\;\forall i \in \mathcal{I},\\

&\mathbf{x}_i^{k}\in\mathcal{X}_i\subset \mathbb{R}^{n_{x_i}},\;\forall i \in \mathcal{I}, k=0,\dots,N_p,\\

&\mathbf{u}_i^{k}\in\mathcal{U}_i\subset \mathbb{R}^{n_{u_i}},\;\forall i \in \mathcal{I}, k=0,\dots,N_p-1,\\

&\sum_{i \in \mathcal{I}} \mathbf{R}_i \mathbf{u}_i^k \le \mathbf{r}_{\max}^{k},\;k=0,\dots,N_p-1,
\end{align}
```
with
```math
\mathbf{x}_i\in\mathbb{R}^{n_{\mathbf{x}}}, \mathbf{u}_i \in\mathbb{R}^{n_{\mathbf{u}}}, \mathbf{A}_i \in\mathbb{R}^{n_{\mathbf{x}} \times n_{\mathbf{x}}}, \mathbf{B}_i\in\mathbb{R}^{n_{\mathbf{x}} \times n_{\mathbf{u}}}, \mathbf{R}_i\in\mathbb{R}^{n_{\mathbf{u}} \times n_{\mathbf{r}}},
```
and convex individual constraints
```math
\begin{align}
&\mathcal{X}_i = \{\mathbf{x}_i\in\mathbb{R}^{n_{\mathbf{x}}}\vert\;\mathbf{x}_i^T\mathbf{G}_{\mathbf{x}_i,l}\mathbf{x}_i \le p_{\mathbf{x}_i,l}^2,\;l=1,\dots,n_{\mathbf{x}}	\},\\
&\mathcal{U}_i = \{\mathbf{u}_i\in\mathbb{R}^{n_{\mathbf{u}}}\vert\;\mathbf{u}_i^T\mathbf{G}_{\mathbf{u}_i,l}\mathbf{u}_i \le p_{\mathbf{u}_i,l}^2,\;l=1,\dots,n_{\mathbf{x}}	\}.
\end{align}
```
The objective of the individual MPC problems is to track a given reference trajectory over the prediction horizon while also minimizing the control inputs and the terminal state violation, i.e.,
```math
\begin{align}
	&J_i(\mathbf{x}_i^k,\mathbf{u}_i^k) = (\mathbf{x}_i^k-\mathbf{x}_i^{\text{ref},k})^T \mathbf{H}_{\mathbf{x}_i}(\mathbf{x}_i^k-\mathbf{x}_i^{\text{ref},k}) + \mathbf{u}_i^{k,T} \mathbf{H}_{\mathbf{u}_i}\mathbf{u}_i^k,\\
&J_i^f(\mathbf{x}^{N_p}) = (\mathbf{x}^{N_p}-\mathbf{x}^{\text{ref},N_p})^T \mathbf{H}_{\mathbf{x}}^f(\mathbf{x}^{N_p}-\mathbf{x}^{\text{ref},N_p})
\end{align}
```
The number and size of the subproblems were varied as follows:
```math
\begin{align*}
	&\text{Number of subproblems: } N_s \in \{5,10,20,50\},\\
	&\text{Number of states: }n_{\mathbf{x}} \in \{2,3,4,5\},\\
	&\text{Number of inputs: }n_{\mathbf{u}} \in \{2,3,4,5\}, n_{\mathbf{x}} \ge n_{\mathbf{u}}\\
	&\text{Number of resources: }n_{\mathbf{r}} \in \{2,3,4,5\}, n_{\mathbf{u}} \ge n_{\mathbf{r}}\\
	&\text{Prediction horizon: }N_{p} \in \{10,15,20\}.
\end{align*}
```

The [data](ttps://github.com/VaYf/DMPC-Benchmark-Problems/tree/main/data) is stored in [Julia's .jld2 file format](https://docs.juliahub.com/JLD2/O1EyT/0.4.0/). An example how to read a benchmark file and how to build the MPC problem of a single subsystem is given in [example_julia.jl](https://github.com/VaYf/DMPC-Benchmark-Problems/blob/main/example_julia.jl):
```julia
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
     @constraint(m,x[:,k+1] .== A*x[:,k+1] + B*u[:,k])
 end
# Convex Constraints (number of constraints = number of states)
for c = 1:nx
    # States
    for k = 1:Np
        @constraint(m, x[:,k]'*Gx[c]*x[:,k] ≤ p_x[c]^2)
    end
     # Inputs
    for k = 1:Np-1
        @constraint(m,u[:,k]'*Gu[c]*u[:,k] ≤ p_u[c]^2)
    end
end
# Resources
global counter = 1
for j = 1:nr, k = 1:Np-1
    @constraint(m,Resources[counter] == R[j,:]'*u[:,k])
    global counter += 1
end
```
