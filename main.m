%CasADi+DMPC+Chance constraints
% addpath('./nmpcroutine');
clear all
close all
% clc 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------
%                                                       
%--------------------------------------------------------------------
%          car 2         
%--------------------------------------------------------------------
%                        car 1                                
%--------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulation repeat start here
simulation_repeat=1;   % number of repeated simulation100
Time_cost=zeros(1,simulation_repeat);
Collision_count_sum=0;
Collisions_record={};


FLAG1=0; % collision avoidance Flag
FLAG2=0; % collision avoidance Flag
FLAGErrorF1=0; % 'Error F' Flag
FLAGErrorF2=0; % 'Error F' Flag
for simu=1:simulation_repeat

%% global 
% global number_vehicles;             % number of vehicles
% global params_vehicles;             % parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,length,width]
% global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% global params_lane;                 % parameters of lane [Lane_width,Lane_length]
% global id_lane_vehicles;            % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
% global left_lane;                   % central position of left lane in lateral position
% global center_lane;                 % central position of center lane in lateral position
% global right_lane;                  % central position of right lane in lateral position
% global velocity_left_lane;          % max velocity of left lane
% global velocity_center_lane;        % max velocity of center lane
% global velocity_right_lane;         % max velocity of right lane
% global states_ref;                  % reference states  [position_x,position_y,psi,velocity]
% global distance_detectable;         % The maximam distance that a vehicle can detect other vehicles in longitudinal direction
% global control_vehicles;            % current (optimized) input u for current step (applied to x0)
% global N;                            % prediction horizon
% global risk_parameter;              % risk parameters of vehicles
% global predicted_states_EV;         % predicted_states of all ego vehicles
% global assumeded_states_TV;         % assumed states of all target vehicles
% global colors_vehicles;             % color of vehicles






%% % parameters preparation (I)
%%% lane
params_lane=[5.25,1500];            % parameters of lane [Lane_width,Lane_length]
width_lane=params_lane(1);
left_lane=2.5*width_lane;       % center line of left lane
center_lane=1.5*width_lane;     % center line of center lane
right_lane=0.5*width_lane;      % center line of right lane
velocity_left_lane=35;              % max velocity of left lane
velocity_center_lane=30;            % max velocity of center lane
velocity_right_lane=25;             % max velocity of right lane
distance_detectable=100;            % The maximam distance that a vehicle can detect other vehicles in longitudinal direction


mpciterations = 1; % number of MPC iterations
N             = 10; % prediction horizon

%% for function nmpc
T             = 0.2;                % sampling time
tol_opt       = 1e-6;
opt_option    = 0;
iprint        = 5;
%     type          = 'differential equation';
type          = 'difference equation';
atol_ode_real = 1e-8;
rtol_ode_real = 1e-8;
atol_ode_sim  = 1e-2;
rtol_ode_sim  = 1e-2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  % Initial parameters of all cars
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
number_vehicles=2; %number of vehicles
colors_vehicles=hsv(number_vehicles);
params_vehicles=[2,2,5,2]; %parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,Vehicle_length,Vehicle_width]
Vehicle_length=params_vehicles(3);
Vehicle_width=params_vehicles(4);


% simulations(end+1,:)=simu;
%% % cell array preparation
states_ref={};
states_vehicles={};
control_vehicles={};
predicted_states_EV={};
assumeded_states_TV={};
states={};
manuv={};
t0={};

Collisions_record{simu}=[];
risk_parameter=[];                  % risk parameters of vehicles
id_lane_vehicles=[];                % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0

%%% vehicle 1
t0{1}=0.0; % 
velocity_car1=27;
position_y_car1=center_lane;
% states{1}=[50+0.1*randn(1) position_y_car1+0.01*randn(1) 0 velocity_car1+0.01*randn(1)];   % [position_x,position_y,psi,velocity]
states{1}=[50 position_y_car1 0 velocity_car1];   % [position_x,position_y,psi,velocity]
u01 = zeros(2,N); % initial control input
manuv{1}=0; % maneuver of the vehicle  0:LK  1:LC-left  -1:LC-right
states_ref{1}=[500 position_y_car1 0 velocity_car1];  % states reference 
risk_parameter(1)=0.70;             % risk parameter of vehicle 1
id_lane_vehicles(1)=1;              % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
predicted_states_EV{1}=[];          % predicted_states of all ego vehicles
assumeded_states_TV{1}=[];          % assumed states of all target vehicles
states_vehicles{1}=states{1};       % states of all vehicles: states_vehicles{id_vehicle},  [position_x,position_y,psi,velocity]
control_vehicles{1}=u01;            % current (optimized) input u for current step (applied to x0)

%%% vehicle 2
t0{2}=0.0; % 
velocity_car2=25;
position_y_car2=right_lane;
states{2}=[67 position_y_car2 0 velocity_car2];   % [position_x,position_y,psi,velocity]
% states{2}=[70+0.1*randn(1) position_y_car2+0.01*randn(1) 0 velocity_car2+0.01*randn(1)];   % [position_x,position_y,psi,velocity]
u02 = zeros(2,N); % initial control input
manuv{2}=1; % maneuver of the vehicle  0:LK  1:LC-left  -1:LC-right
states_ref{2}=[500 center_lane 0 27];  % states reference 
risk_parameter(2)=0.70;             % risk parameter of vehicle 1
id_lane_vehicles(2)=0;              % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
predicted_states_EV{2}=[];          % predicted_states of all ego vehicles
assumeded_states_TV{2}=[];          % assumed states of all target vehicles
states_vehicles{2}=states{2};       % states of all vehicles: states_vehicles{id_vehicle},  [position_x,position_y,psi,velocity]
control_vehicles{2}=u02;            % current (optimized) input u for current step (applied to x0)


%% %%%%%%%%%%% variables need to be update every time
% states_vehicles
% id_lane_vehicles
% maneuver
% states_ref
% control_vehicles    % (optimized) input u for( applied to x0)
%%%%%%%%%%%%%%%%%%

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% analysis of the vehicles simultaneously
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%  % printHeader
%     fprintf('ID    t |        a        delta       x      y     psi      v       Time\n');
%     fprintf('------------------------------------------------------------------------------------------------\n');
    
%% MPC calculation
% Maneuver1=maneuver(1);
% Maneuver2=maneuver(2);
% Maneuver3=maneuver(3);


tic
% figure
iteration_step=100;

FLAG_c_all1=zeros(iteration_step,3);
FLAG_c_all2=zeros(iteration_step,3);
Flag_ErrorF_All1=zeros(iteration_step,number_vehicles);
Flag_ErrorF_All2=zeros(iteration_step,number_vehicles);


for iter=1:iteration_step
    iter;
   
     %%% car 1
    id_vehicle=1;
    [Flag_Error_f,Flag_c_allVehicle,predicted_states_EV,t0{1}, states{1}, u01] = nmpc(@runningcosts, @terminalcosts, @constraints, ...
        @terminalconstraints, @chance_constraints, @linearconstraints, @system_EV, ...
        id_vehicle, params_vehicles, params_lane, states_ref,number_vehicles,id_lane_vehicles,...
        distance_detectable,risk_parameter,states_vehicles, control_vehicles, mpciterations, N, T,...
        t0{1}(end,:), states{1}(end,:), u01, ...
        tol_opt, opt_option, ...
        type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
        iprint, @printHeader, @printClosedloopData); 
    
    %update Flag for 'Error F'      
    Flag_ErrorF_All1(iter,id_vehicle)=Flag_Error_f;
    Flag_c_allVehicle1=Flag_c_allVehicle{1};
    FLAG_c_all1(iter,:)=Flag_c_allVehicle1;
    
     %%% Update some important (global) variables
   
    %%% car 2
    id_vehicle=2;
    [Flag_Error_f,Flag_c_allVehicle,predicted_states_EV,t0{2}, states{2}, u02] = nmpc(@runningcosts, @terminalcosts, @constraints, ...
        @terminalconstraints, @chance_constraints, @linearconstraints, @system_EV, ...
        id_vehicle, params_vehicles, params_lane, states_ref,number_vehicles,id_lane_vehicles,...
        distance_detectable,risk_parameter,states_vehicles, control_vehicles, mpciterations, N, T,...
        t0{2}(end,:), states{2}(end,:), u02, ...
        tol_opt, opt_option, ...
        type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
        iprint, @printHeader, @printClosedloopData); 
    %update Flag for 'Error F'    
    Flag_ErrorF_All2(iter,id_vehicle)=Flag_Error_f;
    Flag_c_allVehicle2=Flag_c_allVehicle{2};
    FLAG_c_all2(iter,:)=Flag_c_allVehicle2;
    
    %%% Update some important (global) variables
    %%% Update variables
    % car 1
    id_vehicle=1;
    % update state vector
    states_vehicles{id_vehicle}(end+1,:)=states{1}(end,:);  % continue to add a 1x4 vector to last row of states_vehicles{id_vehicle}
    % update input vector
    u01 = [u01(:,2:size(u01,2)) u01(:,size(u01,2))];
    control_vehicles{id_vehicle}(end+1:end+2,:)=u01; % continue to add a 2xN vector to the last row of control_vehicles{id_vehicle}
    
    % update  id_lane_vehicles
    [id_lane]=lane_calculate(id_vehicle,number_vehicles,params_vehicles,params_lane,states_vehicles); % calculate the lane where the vehicle stay
    id_lane_vehicles(id_vehicle)=id_lane;  % update id_lane_vehicles

    y_ref=center_lane;
    v_ref=velocity_car1;
    states_ref{id_vehicle}(2)=y_ref;  % reference states   [position_x,position_y,psi,velocity]
    states_ref{id_vehicle}(4)=v_ref;  % reference states   [position_x,position_y,psi,velocity]
%     figure(1)    
%     MovingTrajectory_car1View(states_vehicles,params_lane,predicted_states_EV,colors_vehicles,iter)  % From view of car 2
    
     
    
    
    % car 2 
    id_vehicle=2; 
    % update state vector
    states_vehicles{id_vehicle}(end+1,:)=states{2}(end,:);  % continue to add a 1x4 vector to last row of states_vehicles{id_vehicle}
    % update input vector
    u02 = [u02(:,2:size(u02,2)) u02(:,size(u02,2))];
    control_vehicles{id_vehicle}(end+1:end+2,:)=u02; % continue to add a 2xN vector to the last row of control_vehicles{id_vehicle}
    
    % update  id_lane_vehicles
    [id_lane]=lane_calculate(id_vehicle,number_vehicles,params_vehicles,params_lane,states_vehicles); % calculate the lane where the vehicle stay
    id_lane_vehicles(id_vehicle)=id_lane;  % update id_lane_vehicles

    y_ref=center_lane;
    v_ref=27;
    states_ref{id_vehicle}(2)=y_ref;  % reference states   [position_x,position_y,psi,velocity]
    states_ref{id_vehicle}(4)=v_ref;  % reference states   [position_x,position_y,psi,velocity]
    
%     figure(2)    
%     MovingTrajectory_car2View(states_vehicles,params_lane,predicted_states_EV,colors_vehicles,iter)  % From view of car 2
    
    
end


s_vehicle1=states_vehicles{1};
s_vehicle2=states_vehicles{2};

% s_vehicle1
% s_vehicle2

%%% save states of vehicles
s1='StatesOfvehicles_';
s2=num2str(simu);
fullfilename1 = strcat(s1,s2);
savetofile(states_vehicles,fullfilename1);

%%% Save collision avoidance Flag --- Vehicle 1
s3='FLAG1_c_all_';
fullfilename2 = strcat(s3,s2);
savetofile(FLAG_c_all1,fullfilename2);
avoidance_found1=find(FLAG_c_all1>0);
if ~isempty(avoidance_found1)  %empty 1; noempty 0
FLAG1=FLAG1+1;
else
end

%%% save Flags for 'Error F' --- Vehicle 1
s4='FLAG1_ErrorF_all_';
fullfilename3 = strcat(s4,s2);
savetofile(Flag_ErrorF_All1,fullfilename3);
if any(Flag_ErrorF_All1(:)>0)
    FLAGErrorF1=FLAGErrorF1+1;
else
end

%%% Save collision avoidance Flag --- Vehicle 2
s5='FLAG2_c_all_';
fullfilename2 = strcat(s5,s2);
savetofile(FLAG_c_all2,fullfilename2);
avoidance_found2=find(FLAG_c_all2>0);
if ~isempty(avoidance_found2)  %empty 1; noempty 0
FLAG2=FLAG2+1;
else
end

%%% save Flags for 'Error F' --- Vehicle 2
s6='FLAG2_ErrorF_all_';
fullfilename3 = strcat(s6,s2);
savetofile(Flag_ErrorF_All2,fullfilename3);
if any(Flag_ErrorF_All2(:)>0)
    FLAGErrorF2=FLAGErrorF2+1;
else
end


Time_cost(simu)=toc;

%%% Check collision
% positions of car 1
states_1=states_vehicles{1};
position_x_1=states_1(:,1);
position_y_1=states_1(:,2);

% check collision 
for i=2:number_vehicles
   %calculate distance^2 between vehicle 1 and vehicle i
   states_i=states_vehicles{i};
   position_x_i=states_i(:,1);
   position_y_i=states_i(:,2);
   delta_x_i=abs(position_x_i-position_x_1);
   delta_y_i=abs(position_y_i-position_y_1);
   % search collision
   collision_potential_x=find(delta_x_i<Vehicle_length); %too close in x direstion 
   collision_potential_y=find(delta_y_i<Vehicle_width); %too close in y direction
   [collision_mark]=intersect(collision_potential_x,collision_potential_y);
%    % count the number of collision
%    collision_count=length(collision_mark);  %collisions in current simulation 
%    Collision_count_sum=Collision_count_sum+collision_count;  %all collisions in all previous simulation till now
   % detailed collision record
   Collisions_record_simu=Collisions_record{simu};
   Collisions_record_simu=[Collisions_record_simu;collision_mark];
   Collisions_record{simu}=Collisions_record_simu;
end


end
FLAG1
FLAGErrorF1
FLAG2
FLAGErrorF2

save('Time_cost.mat','Time_cost');
save('Collisions_record.mat','Collisions_record');
% rmpath('./nmpcroutine');
% Time_cost
% Collisions_record
% PlotStates(number_vehicles,colors_vehicles,states_vehicles)
% PlotControls(number_vehicles,colors_vehicles,control_vehicles)
% PlotTrajectory(number_vehicles,params_lane,colors_vehicles,states_vehicles)
% PlotTrajectory_MoreDetails(number_vehicles,states_vehicles,params_lane,colors_vehicles,risk_parameter)

% MovieCreatCar1()
% MovieCreatCar2()
% MovieCreatCar3()
% MovieCreatCar4()
%% necessary functions
%1 [cost] = runningcosts(t, x, u)         ~~~  maneuver_update--Xi_ref
%2 cost = terminalcosts(t, x)             ~~~  maneuver_update--Xi_ref
%3 [c,ceq] = constraints(t, x, u)         ~~~  communication_topology; system_TV(t, x, u, T); chance constraints
%4 [c,ceq] = terminalconstraints(t, x)    ~~~  communication_topology; system_TV(t, x, u, T); chance constraints
%5 [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)      ~~~communication_topology; system_TV(t, x, u, T); chance constraints
%6 [y] = system_EV(t, x, u, T)
%7 [y] = system_TV(t, x, u, T)            ~~~ disturbance

function printHeader()
%     fprintf('ID    t    |    a     delta      x     y    psi    v    Time\n');
%     fprintf('------------------------------------------------------------------------------------------------\n');
end

% function printClosedloopData(mpciter, u, x, t_Elapsed)
%     
%    global iterations;
%    global id_vehicle
%     
%    fprintf('%3d %3d  | %+11.6f %+11.6f %+6.3f %+6.3f %+6.3f  %+6.3f  %+6.3f', id_vehicle, iterations(end,:), ...
%            u(1), u(2), x(1), x(2), x(3), x(4) ,t_Elapsed);
% end
function savetofile(data,fullfilename)
    save(fullfilename,'data');
end

