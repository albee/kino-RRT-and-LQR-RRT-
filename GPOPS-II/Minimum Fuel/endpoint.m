% endpoint.m

function output = endpoint(input)

    % Unpack fuel usage
    F = input.phase.integral;
    
    % Pack output
    output = struct;
    
    output.objective = F;

end