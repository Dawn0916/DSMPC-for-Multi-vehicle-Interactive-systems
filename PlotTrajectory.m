function PlotTrajectory(number_vehicles,params_lane,colors_vehicles,states_vehicles)
%global number_vehicles;             % number of vehicles
% global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% global params_lane;                 % parameters of lane [Lane_width,Lane_length]
% global colors_vehicles;             % color of vehicles

w_lane=params_lane(1);
fontsize_labels=12;
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
