function MovingTrajectory_car2View(states_vehicles,params_lane,predicted_states_EV,colors_vehicles,iterations)
% params_lane;                 % parameters of lane [Lane_width,Lane_length]
% predicted_states_EV;         % predicted_states of all ego vehicles
% assumeded_states_TV;         % assumed states of all target vehicles
% colors_vehicles;             % color of vehicles
% iterations;                  % tracking of the time skips

iterations_step=iterations;
w_lane=params_lane(1);
fontsize_labels=12;
% figure
set(gcf,'Units','normalized','OuterPosition',[0.0 0.3 0.99 0.35]);
%% Plot the predicted trajectory of ego vehicle 2
EV_predicted_car2=predicted_states_EV{2};
% TV_assumed_car1=assumeded_states_TV{1}
% TV_assumed_car3=assumeded_states_TV{3}

% predicted_states of ego vehicle 2
position_x_EV=predicted_states_EV{2}(1,1);
position_y_EV=predicted_states_EV{2}(1,2);
predicted_positions_x_EV=predicted_states_EV{2}(:,1);
predicted_positions_y_EV=predicted_states_EV{2}(:,2);

% plot trajectory of ego vehicle 2
plot(position_x_EV', position_y_EV','o','linewidth',3,'color',colors_vehicles(2,:));
hold on
EV2=plot(predicted_positions_x_EV', predicted_positions_y_EV','-.','linewidth',1,'color',colors_vehicles(2,:));

% hold on

%% Plot the assumeded trajectory of  vehicle 1
states_vehicle_TV=states_vehicles{1}(end,:);
position_x_TV=states_vehicle_TV(1);   % The center of the ellipse should be the assumed TV center
position_y_TV=states_vehicle_TV(2);



% assumed states of target vehicle 1
% if isempty(assumeded_states_TV{1})==0
% position_x_TV=assumeded_states_TV{1}(1,1);
% position_y_TV=assumeded_states_TV{1}(1,2);
Ellipse_x1=position_x_TV-15;
Ellipse_x2=position_x_TV+15;
Ellipse_y1=position_y_TV;
Ellipse_y2=position_y_TV;
eccentricity = 0.98;
numPoints = 300; % Less for a coarser ellipse, more for a finer resolution.
% Make equations:
Ellipse_a = (1/2) * sqrt((Ellipse_x2 - Ellipse_x1) ^ 2 + (Ellipse_y2 - Ellipse_y1) ^ 2);
Ellipse_b = Ellipse_a * sqrt(1-eccentricity^2);
t = linspace(0, 2 * pi, numPoints); % Absolute angle parameter
X = Ellipse_a * cos(t);
Y = Ellipse_b * sin(t);
% Compute angles relative to (x1, y1).
angles = atan2(Ellipse_y2 - Ellipse_y1, Ellipse_x2 - Ellipse_x1);
Ellipse_x = (Ellipse_x1 + Ellipse_x2) / 2 + X * cos(angles) - Y * sin(angles);
Ellipse_y = (Ellipse_y1 + Ellipse_y2) / 2 + X * sin(angles) + Y * cos(angles);
% Plot the ellipse as a blue curve.
EllipsePlot=plot(Ellipse_x,Ellipse_y,'linewidth',1,'color',colors_vehicles(1,:));	% Plot ellipse
hold on

% assumeded_positions_x_TV=assumeded_states_TV{1}(:,1);
% assumeded_positions_y_TV=assumeded_states_TV{1}(:,2);  
% plot trajectory of target vehicle 1
TV1=plot(position_x_TV', position_y_TV','x','linewidth',3,'color',colors_vehicles(1,:));
hold on
% TV1=plot(assumeded_positions_x_TV', assumeded_positions_y_TV',':','linewidth',1,'color',colors_vehicles(1,:));

% hold on
% else
% end

%% Plot the assumeded trajectory of ego vehicle 3
% % assumed states of target vehicle 2
% if isempty(assumeded_states_TV{3})==0
% position_x_TV=assumeded_states_TV{3}(1,1);
% position_y_TV=assumeded_states_TV{3}(1,2);
% assumeded_positions_x_TV=assumeded_states_TV{3}(:,1);
% assumeded_positions_y_TV=assumeded_states_TV{3}(:,2);  
% % plot trajectory of target vehicle 1
% plot(position_x_TV', position_y_TV','x','linewidth',3,'color',colors_vehicles(3,:));
% hold on
% plot(assumeded_positions_x_TV', assumeded_positions_y_TV',':','linewidth',1,'color',colors_vehicles(3,:));
% hold on
% else
% end


%% figure setting
start_x=position_x_EV-25;
end_x=position_x_EV+55;
title(sprintf('Moving trajectories from the view of vehicle 2'),'interpreter','latex','FontSize',fontsize_labels)
axis([start_x end_x -1 16])
ylabel('$y$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$x$','interpreter','latex','FontSize',fontsize_labels)
hold on

%% plot the lines between lanes
plot([start_x end_x], [0 0], '-k', 'LineWidth', 1.5);
hold on;
plot([start_x end_x], [w_lane w_lane], '--k','LineWidth', 1.5);
hold on;
plot([start_x end_x], [2*w_lane 2*w_lane], '--k', 'LineWidth', 1.5);
hold on;
plot([start_x end_x], [3*w_lane 3*w_lane], '-k', 'LineWidth', 1.5);
hold on;
saveas(gcf,sprintf('Moving Trajectories from the view of vehicle 2 FIG%d.fig',iterations_step));
saveas(gcf,sprintf('MovingTrajectoryOfvehicle2FIG%d.png',iterations_step));
set(EV2,'Visible','off');
% if isempty(assumeded_states_TV{1})==0
set(TV1,'Visible','off');
set(EllipsePlot,'Visible','off');
% else
% end
% saveas(gcf,'Moving Trajectory of vehicle 2-FIG%d%','fig',iterations_step);
% saveas(gcf,'MovingTrajectoryOfvehicle2-FIG%d%','png',iterations_step);
end
