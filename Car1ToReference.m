%%% distance between car 1 to reference lane

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

params_vehicles=[2,2,5,2]; %parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,Vehicle_length,Vehicle_width]
Vehicle_length=params_vehicles(3);
Vehicle_width=params_vehicles(4);
colors_simulations=hsv(simulation_repeat);



%% Fig praparation
figure
fontsize_labels=12;
set(gcf,'Units','normalized','OuterPosition',[0.25 0.25 0.4 0.4])
%%
ref=1.5*width_lane*ones(200,1);
Distance_y2ref=zeros(simulation_repeat,200);
for simu=1:simulation_repeat

if ErrorFlag(simu)==0
% positions of car 1
s1='StatesOfvehicles_';
s2=num2str(simu);
filename=strcat(s1,s2);
states_vehicles_struct=load(filename,'-mat');
states_vehicles_cell0=struct2cell(states_vehicles_struct);
states_vehicles=states_vehicles_cell0{1};
states_1=states_vehicles{1};
% position_x_1=states_1(:,1);
position_y_1=states_1(:,2);
distance_y_ref=abs(position_y_1(1:200,:)-ref);
Distance_y2ref(simu,:)=distance_y_ref';
else
end
end
AverDistance_y2ref=(sum(Distance_y2ref,1))/(simulation_repeat-ErrorNum);

    axis([0 200 0 10])
    t=1:1:200;
    colors_vehicles=hsv(5);
    plot(t,AverDistance_y2ref,'linewidth',1,'color',colors_vehicles(1,:))
    title(sprintf('Distance to target lane'),'interpreter','latex','FontSize',fontsize_labels);
    ylabel('Distance to target lane','interpreter','latex','FontSize',fontsize_labels);
    xlabel('$t$','interpreter','latex','FontSize',fontsize_labels);
    grid on
    grid minor
    hold on
    saveas(gcf,'AverDistance_y2ref','fig')
    saveas(gcf,'AverDistance_y2ref','png')
  save('Distance_y2ref.mat','Distance_y2ref');
 save('AverDistance_y2ref.mat','AverDistance_y2ref');

