%%% Trajectories without error

%% Error detect
width_vehicle=2;
width_lane=5.25;
simulation_repeat=100;
number_vehicles=2;
ErrorFlag=zeros(1,simulation_repeat);


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
        ErrorFlag(simu)=1;  %% note the simulation with error
    else
    end
 end
end
ErrorNum=sum(ErrorFlag)
%%
params_vehicles=[2,2,5,2]; %parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,Vehicle_length,Vehicle_width]
Vehicle_length=params_vehicles(3);
Vehicle_width=params_vehicles(4);
colors_vehicles=hsv(number_vehicles);
Collisions_record={};
collision_num=0;

%% Fig praparation
figure
fontsize_labels=12;
set(gcf,'Units','normalized','OuterPosition',[0.2 0.4 0.3 0.3]);
%%
for simu=1:simulation_repeat

%%% Check collision
Collisions_record{simu}=[];

% positions of car 1
s1='StatesOfvehicles_';
s2=num2str(simu);
filename=strcat(s1,s2);
states_vehicles_struct=load(filename,'-mat');
states_vehicles_cell0=struct2cell(states_vehicles_struct);
states_vehicles=states_vehicles_cell0{1};
states_1=states_vehicles{1};
position_x_1=states_1(:,1);
position_y_1=states_1(:,2);
end_x=1200;

if ErrorFlag(simu)==0
    plot(position_x_1', position_y_1','linewidth',1,'color',colors_vehicles(1,:))
    hold on
    %%  Method 1
    for i=2:number_vehicles
        %calculate distance^2 between vehicle 1 and vehicle i
        states_i=states_vehicles{i};
        position_x_i=states_i(:,1);
        position_y_i=states_i(:,2);
        plot(position_x_i', position_y_i','linewidth',1,'color',colors_vehicles(i,:))
    end
else
end

end
title(sprintf('Trajectories of vehicles'),'interpreter','latex','FontSize',fontsize_labels)
axis([0 end_x -1 16])
ylabel('$y$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$x$','interpreter','latex','FontSize',fontsize_labels)
hold on

saveas(gcf,'single_SMPC_trajectories_without_error','fig')
saveas(gcf,'single_SMPC_trajectories_without_error','png')
% 
% saveas(gcf,'multi_SMPC_trajectories','fig')
% saveas(gcf,'multi_SMPC_trajectories','png')
