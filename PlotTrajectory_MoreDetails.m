function PlotTrajectory_MoreDetails(number_vehicles,states_vehicles,params_lane,colors_vehicles,risk_parameter)
% number_vehicles;             % number of vehicles
% states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% params_lane;                 % parameters of lane [Lane_width,Lane_length]
% colors_vehicles;             % color of vehicles
% risk_parameter;              % risk parameters of vehicles
w_lane=params_lane(1);
fontsize_labels=12;
states_1=states_vehicles{1};
positions_x_1=states_1(:,1);
positions_y_1=states_1(:,2);
states_2=states_vehicles{2};
positions_x_2=states_2(:,1);
positions_y_2=states_2(:,2);
delta_x_squ=(positions_x_1-positions_x_2).^2;
delta_y_squ=(positions_y_1-positions_y_2).^2;
delta_s=sqrt(delta_x_squ+delta_y_squ);   % Distance between 2 vehicles

risk=risk_parameter(2);
save(['states_vehicles_',num2str(risk),'.mat'],'states_vehicles');
save(['Distances_3_',num2str(risk),'.mat'],'delta_s');

figure
set(gcf,'Units','normalized','OuterPosition',[0.0 0.4 0.99 0.35]);
%% Plot the trajectories of vehicles
for i=1:number_vehicles
states=states_vehicles{i};
position_x=states(:,1);
position_y=states(:,2);
end_x=400;
plot(position_x', position_y','linewidth',3,'color',colors_vehicles(i,:))
title(sprintf('Trajectory of vehicles'),'interpreter','latex','FontSize',fontsize_labels)
axis([0 end_x -1 16])
ylabel('$y$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$x$','interpreter','latex','FontSize',fontsize_labels)
hold on
end

%% plot the lines between lanes
plot([0 end_x], [0 0], '-k', 'LineWidth', 1.5);
hold on;
plot([0 end_x], [w_lane w_lane], '--k','LineWidth', 1.5);
hold on;
plot([0 end_x], [2*w_lane 2*w_lane], '--k', 'LineWidth', 1.5);
hold on;
plot([0 end_x], [3*w_lane 3*w_lane], '-k', 'LineWidth', 1.5);
hold on;
saveas(gcf,'Trajectory of vehicles','fig');
saveas(gcf,'TrajectoryOfVehicles','png');
end
