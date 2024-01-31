function cost = runningcosts_RefChange(t, x, u, kT,id_vehicle,states_ref)
    %global id_vehicle;         % id of vehicle
%     global states_ref;         % reference states  [position_x,position_y,psi,velocity]
    center_lane=7.8750;
    right_lane=2.6250;
    state_ref_id=states_ref{id_vehicle}; % current state reference of ego vehicle
    
    %%% reference change
    x_position_predict=x(1);
    if x_position_predict>120
        y_ref=center_lane;
    else
        y_ref=right_lane;
    end
    state_ref_id(2)=y_ref;
    Q=diag([0.00001,0.2,1,1]); % weighting matrix
    R=diag([0.33,5]);  % weighting matrix
    cost=(x-state_ref_id)*Q*(x-state_ref_id)'+u'*R*u;   % assume that the reference will not change during prediction
end
