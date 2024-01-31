function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, xi, u)
    
    a_min=-9;                                   % m/s^2
    a_max=6;                                    % m/s^2
    delta_min=-1.2;                             % rad
    delta_max=1.2;                              % rad
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
%     lb  = [];
%     ub  = [];
    lb  = [a_min delta_min];
    ub  = [a_max delta_max];
end