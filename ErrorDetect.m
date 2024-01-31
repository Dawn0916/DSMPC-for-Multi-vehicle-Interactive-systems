%%% Error detect
width_vehicle=2;
width_lane=5.25;
simulation_repeat=100;
number_vehicles=2;
ErrorFlag=zeros(1,simulation_repeat);

%%
for simu=1:simulation_repeat

% positions of car 1
s1='StatesOfvehicles_';
s2=num2str(simu);
filename=strcat(s1,s2);
states_vehicles_struct=load(filename,'-mat');
states_vehicles_cell0=struct2cell(states_vehicles_struct);
states_vehicles=states_vehicles_cell0{1};

% error check
 for i=1:number_vehicles
    %calculate distance^2 between vehicle 1 and vehicle i
    states_i=states_vehicles{i};
    position_x_i=states_i(:,1);
    position_y_i=states_i(:,2);
    [ErrorID1]=find(position_y_i<width_vehicle/2);
    [ErrorID2]=find(position_y_i>(3*width_lane-width_vehicle/2));
    ErrorID=[ErrorID1;ErrorID2];
    if isempty(ErrorID)==0
        ErrorFlag(simu)=1;
    else
    end
 end

end
ErrorFlag
ErrorNum=sum(ErrorFlag)

