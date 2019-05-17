% continuous.m

function output = continuous(input)

    % Unpack state
    x  = input.phase.state(:,1);
    y  = input.phase.state(:,2);
    vx = input.phase.state(:,3);
    vy = input.phase.state(:,4);
    h  = input.phase.state(:,5);
    w  = input.phase.state(:,6);
    
    % Unpack control
    f1 = input.phase.control(:,1);
    f2 = input.phase.control(:,2);
    f3 = input.phase.control(:,3);
    f4 = input.phase.control(:,4);
    
    % Unpack auxdata
    m = input.auxdata.m;
    l = input.auxdata.l;
    I = input.auxdata.I;
    
    % Calculate dynamics
    dx  = vx;
    dy  = vy;
    dvx = (f1+f2-f3-f4).*sin(h)/m;
    dvy = (-f1-f2+f3+f4).*cos(h)/m;
    dh  = w;
    dw  = (-f1+f2-f3+f4)*l/2/I;
    
    % Calculate integral
    int = f1.^2+f2.^2+f3.^2+f4.^2;

    % Pack output
    output = struct;
    
    output.dynamics  = [dx,dy,dvx,dvy,dh,dw];
    output.integrand = int;
    
end