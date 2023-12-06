module DMPCBenchmarks
    import JLD2, FileIO
    ##########################################################################################################
    """
    DMPCBenchmarks.ranges(Ns::Int,nx::Int,nu::Int,nr::Int,Np::Int,Run::Int)
    Check if specified DMPC benchmark exists. If not, print an error
    Arguments:
    Ns  ∈ [5,10,20,50]  Number of subsystems
    nx  ∈ [2,3,4,5]     Number of states
    nu  ∈ [2,3,4,5]     Number of control inputs
    nr  ∈ [2,3,4,5]     Number of shared resources
    Np  ∈ [10,15,20]    Prediction horizon
    Run ∈ [1,2,...,10]  Problem instance
    
    Additionally, nx ≥ nu && nu ≥ nr must hold
    """
    function ranges(Ns::Int,nx::Int,nu::Int,nr::Int,Np::Int,Run::Int)
        # Check number of subsystems
       if Ns ∉ [5,10,20,50]
        @error(string("Ns = ",Ns," invalid. Available numbers of subsystems: ", [5,10,20,50]))
       end
       # Check number of states
       if nx ∉ [2,3,4,5]
        @error(string("nx = ",nx," invalid. Available numbers of states: ", [2,3,4,5]))
       end
       # Check number of control inputs
       if nu ∉ [2,3,4,5]
        @error(string("nu = ",nu," invalid. Available numbers of control inputs: ", [2,3,4,5]))
       end
       # Check number of resources
       if nr ∉ [2,3,4,5]
        @error(string("nr = ",nr," invalid. Available numbers of resources: ", [2,3,4,5]))
       end
       # Check prediction horizon
       if Np ∉ [10,15,20]
        @error(string("Np = ",Np," invalid. Available prediction horizon lengths: ", [10,15,20]))
       end
       # Check run
       if Run ∉ collect(1:10)
        @error(string("Run = ",Run," invalid. Available runs: ", collect(1:10)))
       end
       # Check states-inputs relation
       if nx < nu
        @error(string("Number of states must be larger than number of control inputs (nx ≥ nu)"))
       end
       # Check inputs-resources relation
       if nu < nr
        @error(string("Number of control inputs must be larger than number of resources (nu ≥ nr)"))
       end
    end
    ##########################################################################################################
    """
    DMPCBenchmarks.read(Ns::Int,nx::Int,nu::Int,nr::Int,Np::Int,Run::Int)
    Read specified DMPC benchmark file
    Arguments:
    Ns  ∈ [5,10,20,50]  Number of subsystems
    nx  ∈ [2,3,4,5]     Number of states
    nu  ∈ [2,3,4,5]     Number of control inputs
    nr  ∈ [2,3,4,5]     Number of shared resources
    Np  ∈ [10,15,20]    Prediction horizon
    Run ∈ [1,2,...,10]  Problem instance
    
    Additionally, nx ≥ nu && nu ≥ nr must hold

    Output:
    Dataframe with the following keys:
    "r_max":     Maximum availability of resources
    "System i":  Data of subsystem i
    
    Keys for "System i"
        A     Dynamics matrix
        B     Input matrix
        Hx    State objective function weight
        Hu    Input obctive function weight
        Gx    State constraint matrix
        p_x   State constraint right-hand side
        Gu    Input constraint matrix
        p_u   Input constraint right-hand side
        x_ref State reference trajectory
        x0    State initial condition
        R     Resource utilization matrix
    """
    function read(Ns::Int,nx::Int,nu::Int,nr::Int,Np::Int,Run::Int)
        # Check input data
        ranges(Ns,nx,nu,nr,Np,Run)
        # Read file located in data
        File = string("data/DMPC_Ns_",Ns,"_nx_",nx,"_nu_",nu,"_nr_",nr,"_Np_",Np,"_Run_",Run,".jld2")
        return Data = FileIO.load(File)
    end
end
   