%% Custom Serial Manipulator for 3D Calligraphy

% Load the Robotics Toolbox into Matlab's path
run('../Matlab Robotics Toolbox/robot-10.1/rvctools/startup_rvc.m') % you only have to run this once

%% Setup model of robot
clear,clc, close all

mdl_calligbot % custom model for serial manipulator

calligbot % display robot DH table and info

n = calligbot.n
k = 2;

% Extract waypoints from workspace struct 'richie_cursive' to waypoints
waypoints = 1 - extract_waypoints('richie_cursive')/1000; % Scale name to workspace

dot_the_eyes = 1 - [226; 588]/1000;

% Path = Pathx(x1, x2, y1, y2, k)
for i=1:1:length(waypoints)-1
    Paths = Pathx(waypoints(i,1), waypoints(i+1,1), waypoints(i,2), waypoints(i+1,2), k);
    for ii=1:1:k
        config_IK((i-1)*k+ii,1:3) = calligbot.ikine(T_eex(0.0,Paths(ii,1),Paths(ii,2)),'mask',[1 1 1 0 0 0]);
        ee_pos = T_eex(0.0,Paths(ii,1),Paths(ii,2));
        ee_traj((i-1)*k+ii,1:3) = ee_pos(1:3,4);
    end
end
for i=1:1:length(waypoints)-1
    i
    for ii=1:1:k
        ii
        plot2(ee_traj(1:(i-1)*k+ii,1:3),'r-');
        calligbot.plot(config_IK((i-1)*k+ii,1:3));
    end
    if i == length(waypoints)-1
        plot2(ee_traj, 'r-');
%         plot_point(dot_the_eyes,'r.');
        calligbot.plot([0 0 0]);
    end
end

% Q = calligbot.jtraj(T_eex(0.5,0.5,0.5),T_eex(0.5,0.5,1.0),k,'mask',[1 1 1 0 0 0])
% 
% for i=1:1:length(Q)
%     ee_T = calligbot.fkine(Q(i,1:3));
%     ee_traj(i,1:3) = ee_T.t.';
%     plot2(ee_traj(1:i,1:3),'r-');
%     calligbot.plot(Q(i,1:3));
% end

% generate a linear path between config_i and config_f
% config_i = zeros(1,n);
% config_f = zeros(1,n); %rand(1,n)*2*pi; % a random final configuration
% config_f(2) = pi/2;
% config_f(3) = pi/2;
% Q = zeros(k, n);
% for i = 1:n
%     Q(:, i) = linspace(config_i(i), config_IK(i), k);
% end
% 
% for i=1:1:k-1
%     ee_T = calligbot.fkine(Q(i,1:3));
%     ee_pos = ee_T.t.';
%     ee_traj(i,1:3) = ee_pos;
%     plot2(ee_traj,'r-')
%     calligbot.plot(Q(i,1:3))
% end

%% Forward Kinematics

T = @(theta,d,a,alpha) [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
        sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
        0           sind(alpha)             cosd(alpha)              d;
        0           0                       0                        1];

T_0_to_4_mat = zeros(4*n,4);
    
for i = 1 : 5
    T_0_to_4_mat((i-1)*4+1:(i-1)*4+4,1:4) = T(calligbot.theta(i),calligbot.d(i),calligbot.a(i),calligbot.alpha(i));
end

T_0_to_4_mat

%% Name image

I = imread('my_name_messy_cursive.PNG');
imshow(I)



