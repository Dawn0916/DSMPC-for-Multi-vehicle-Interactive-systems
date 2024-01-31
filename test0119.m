params_lane=[5.25,1500];            % parameters of lane [Lane_width,Lane_length]
width_lane=params_lane(1);
left_lane=2.5*width_lane;       % center line of left lane
center_lane=1.5*width_lane;     % center line of center lane
right_lane=0.5*width_lane;      % center line of right lane

velocity_car1=1;
position_y_car1=center_lane;
states{1}=[80 position_y_car1 0 velocity_car1];   % [position_x,position_y,psi,velocity]

velocity_car2=24;
position_y_car2=right_lane;
states{2}=[10 position_y_car2 0 velocity_car2];  
x_k(1)=80;
z_k(1)=10;

a=15;
b=3;
p=0.6;
k=1;
Sum_e=[];
    Sum_e(1:4,:)=diag([0,0,0,0]);   %initial matrix of Sum_e is put in the fist 4 rows
row_start_point=4*(k-1)+1;
        row_end_point=4*k;
        Sum_e_k=Sum_e((row_start_point:row_end_point),:);
x_k(2)=position_y_car1;
z_k(2)=position_y_car2;
Delta_x_k=x_k(1)-z_k(1);
Delta_y_k=x_k(2)-z_k(2);
Gradient_d=[-2*Delta_x_k/(a^2),-2*Delta_y_k/(b^2),0,0];
erfinv(2*p-1)
cnew=sqrt(2*Gradient_d*Sum_e_k*Gradient_d')*erfinv(2*p-1)