function [maneuver_new,lane_ref]=maneuver_update(id_vehicle,number_vehicles,id_lane_vehicles,maneuver,states_vehicles)
%%% This function is designed to update the "maneuver" vector 

% maneuvers of all vehicles 0:LK  1:LC-left  -1:LC-right
% id_vehicle: the ID of the vehicle
% id_lane_vehicles: id of lanes where all vehicles stay


% global states_vehicles;             % current states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
%global number_vehicles;             % number of vehicles
% global id_vehicle;                  % Label of current car
% global id_lane_vehicles;            % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
% global maneuver;                    % maneuvers of all vehicles 0:LK  1:LC-left  -1:LC-right
% global states_ref;                  % reference states  [position_x,position_y,psi,velocity]

maneuver_last=maneuver(id_vehicle); % maneuver of vehicle id at last time step

%% % change cell array to matrix: states_vehicles--->states_vehicles_matrix
states_vehicles_matrix=[];
for i=1:number_vehicles
   states_vehicles_matrix_intermediate=states_vehicles{i};
   states_vehicles_matrix=[states_vehicles_matrix; states_vehicles_matrix_intermediate(end,:)];  % change cell array to matrix
end
velocity_vehicles=states_vehicles_matrix(:,4);                                                   % velocity of all vehicles
velocity=states_vehicles_matrix(id_vehicle,4);                                                   % velocity of the ego vehicle

%% maneuver selection
% [~,neighbor_front_nearst,neighbors_left_lane,neighbors_right_lane]=communication_topology(id_vehicle,maneuver_last,states_vehicles,id_lane_vehicles);
[~,neighbor_front_nearst,neighbors_left_lane,neighbors_right_lane]=communication_topology();
velocity_front_vehicle=states_vehicles_matrix(neighbor_front_nearst,4);                           % velocity of front vehicle
id_lane=id_lane_vehicles(id_vehicle);

switch id_lane
    case 0 % right lane
        if isempty(neighbor_front_nearst)==1  %no vehicle in front of the vehicle
            maneuver_new=0; % Lane keeping
        else
            if velocity<=velocity_front_vehicle                                                  % the vehicle in front moves with a higher or same velocity
%             if states_vehicles{id_vehicle}(end,4)<=states_vehicles{neighbor_front_nearst}(end,4) %the vehicle in front moves with a higher or same velocity
                maneuver_new=0; % Lane keeping
            else
                if isempty(neighbors_left_lane)==0         % neighbors exist at left lane
                    maneuver_new=0; % Lane keeping
                else                                       % no vehicles in left lane
                    maneuver_new=1; % LC-left
                end
            end
        end
        lane_ref=id_lane+maneuver_new;
        
    case 1 % center lane
        if isempty(neighbors_left_lane)==1          % no vehicles in left lane
            if isempty(neighbor_front_nearst)==1        % no vehicles in in front
                if isempty(neighbors_right_lane)==1         % no vehicles in right lane
                    maneuver_new=-1; % LC-right
                else                                        % neighbors in right lane
                    maneuver_new=0; % Lane keeping
                end
            else % vehicles in front
                if velocity<=velocity_front_vehicle     % the vehicle in front moves with a higher or same velocity
                    maneuver_new=0;  % Lane keeping
                else                                 % the vehicle in front moves with lower velocity                                                
                    maneuver_new=1; % LC-left   
                end
            end
        else                                        % vehicles in left lane
            maneuver_new=0; % Lane keeping   %%%%%%%%%%%%%%%% Too simple?
        end
        lane_ref=id_lane+maneuver_new;
            
    case 2 % left lane
        if isempty(neighbor_front_nearst)==1        % no vehicles in front
            if isempty(neighbors_right_lane)==1         % no vehicles in right lane
                maneuver_new=-1; % LC-right
            else                                        % vehicles in right lane
                maneuver_new=0; % Lane keeping
            end
        else                  %  vehicles in front                      % vehicles in front
            if isempty(neighbors_right_lane)==1         % no vehicles in right lane
                maneuver_new=-1; % LC-right
            else             % vehicles in right lane
                maneuver_new=0; % Lane keeping
            end
        end
        lane_ref=id_lane+maneuver_new;
end

% %% update maneuver of the vehicle with ID id_vehicle
% maneuver(id_vehicle)=maneuver_new;  % maneuver(id_vehicle): lateral maneuver of vehicle id_vehicle
% states_ref{id_vehicle}(2)=lane_ref;  % states_ref{id_vehicle}(2): reference lane of vehicle id_vehicle
end