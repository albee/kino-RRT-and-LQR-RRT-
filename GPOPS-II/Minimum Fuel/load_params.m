% load_params.m

function params = load_params()

    % Spacecraft mass [kg]
    m = 4.1;
    
    % Spacecraft side length [m]
    l = 0.25;
    
    % Spacecraft rotational inertia [kgm^2]
    I = 1/6*m*l^2;
    
    % Maximum thrust output [N]
    Fmax = 0.1;
    
    % Maximum position limit [m]
    pmax = 50;
    
    % Maximum velocity limit [m/s]
    vmax = 50;
    
    % Maximum heading limit [rad]
    hmax = 5*pi;
    
    % Maximum angular velocity limit [rad/s]
    wmax = pi;

    % Pack parameters structure
    params = struct;
    
    params.m    = m;
    params.l    = l;
    params.I    = I;
    params.Fmax = Fmax;
    params.pmax = pmax;
    params.vmax = vmax;
    params.hmax = hmax;
    params.wmax = wmax;

end