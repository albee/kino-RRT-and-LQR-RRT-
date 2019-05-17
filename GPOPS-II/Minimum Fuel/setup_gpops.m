% setup_gpops.m

function setup = setup_gpops(params)

    % Unpack parameters
    T     = params.T;
    x0    = params.x0;
    xF    = params.xF;
    m     = params.m;
    l     = params.l;
    I     = params.I;
    Fmax  = params.Fmax;
    pmax  = params.pmax;
    vmax  = params.vmax;
    hmax  = params.hmax;
    wmax  = params.wmax;
    state = params.state;
    
    % Define lower and upper state limits
    xmin = [-pmax;...
            -pmax;...
            -vmax;...
            -vmax;...
            -hmax;...
            -wmax];
        
    xmax = [pmax;...
            pmax;...
            vmax;...
            vmax;...
            hmax;...
            wmax];
    
    % Define auxdata
    auxdata = struct;
    
    auxdata.m = m;
    auxdata.l = l;
    auxdata.I = I;
        
    % Define bounds
    bounds = struct;
    
    bounds.phase.initialtime.lower  = 0;
    bounds.phase.initialtime.upper  = 0;
    bounds.phase.finaltime.lower    = T;
    bounds.phase.finaltime.upper    = T;
    bounds.phase.initialstate.lower = x0';
    bounds.phase.initialstate.upper = x0';
    bounds.phase.finalstate.lower   = xF';
    bounds.phase.finalstate.upper   = xF';
    bounds.phase.state.lower        = xmin';
    bounds.phase.state.upper        = xmax';
    bounds.phase.control.lower      = [0 0 0 0];
    bounds.phase.control.upper      = state'.*[Fmax Fmax Fmax Fmax];
    bounds.phase.integral.lower     = 0;
    bounds.phase.integral.upper     = 4*T*Fmax;
    
    % Define initial guess
    guess = struct;
    
    guess.phase.time     = [0;T];
    guess.phase.state    = [x0';xF'];
    guess.phase.control  = [[0 0 0 0];[0 0 0 0]];
    guess.phase.integral = 0;
    
    % Define mesh
    mesh = struct;
    
    mesh.method    = 'hp-PattersonRao';
    mesh.tolerance = 1e-7;
    
    % Pack setup structure
    setup = struct;
    
    setup.name                 = 'underactuated_minfuel';
    setup.functions.continuous = @continuous;
    setup.functions.endpoint   = @endpoint;
    
    setup.auxdata = auxdata;
    setup.bounds  = bounds;
    setup.guess   = guess;
    setup.mesh    = mesh;
    
    setup.nlp.solver                  = 'ipopt';
    setup.derivatives.supplier        = 'sparseCD';
    setup.derivatives.derivativelevel = 'second';
    setup.method                      = 'RPM-Differentiation';
    setup.nlp.ipoptoptions.tolerance  = 1e-7;

end