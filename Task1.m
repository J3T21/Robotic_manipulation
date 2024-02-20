close all;
% Example usage
theta_o = asin(0.024/0.130); % offset link angle
theta1 = [-pi/2:0.01:pi/2]; % Joint 1 angle in rads
length1=length(theta1)
theta2 = [theta_o:0.01:pi/3+theta_o] ; % Joint 2 angle in rads
theta2 = [theta2, zeros(1, 315 - length(theta2))];
theta3 = [zeros(length1)]; % Joint 3 angle in rads
theta4 = [zeros(length1)]; % Joint 4 angle in rads

plot_OpenManipX(theta1, theta2, theta3, theta4);

function plot_OpenManipX(theta1, theta2, theta3, theta4)
    % DH parameters for OpenManipulator-X
    % alpha a d theta (degrees)
    figure;
    
    if length(theta1) ~= length(theta2) || length(theta2) ~= length(theta3) || length(theta3) ~= length(theta4)
        error('Input angle arrays must have the same length');
    end
    for i = 1:length(theta1)
        clf;
        hold on;
        grid on;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        
        view(3);
        DH = DH_table(theta1(i), theta2(i), theta3(i), theta4(i));
        T0_1 = Ti( DH(1,:));
        T1_2 = Ti( DH(2,:));
        T2_3 = Ti( DH(3,:));
        T3_4 = Ti( DH(4,:));
        T0_2 = T0_1*T1_2;
        T0_3 = T0_2*T2_3;
        T0_4 = T0_3*T3_4;
        L0=eye(4);
        L1 = T0_1*L0;
        L2 = T0_2*L0;
        L3 = T0_3*L0;
        L4 = T0_4*L0;
        plot3([L0(1,4),L1(1,4)],[L0(2,4),L1(2,4)],[L0(3,4),L1(3,4)],'red','LineWidth', 3);
        plot3([L1(1,4),L2(1,4)],[L1(2,4),L2(2,4)],[L1(3,4),L2(3,4)],'green','LineWidth', 3);
        plot3([L2(1,4),L3(1,4)],[L2(2,4),L3(2,4)],[L2(3,4),L3(3,4)],'blue','LineWidth', 3);
        plot3([L3(1,4),L4(1,4)],[L3(2,4),L4(2,4)],[L3(3,4),L4(3,4)],'black','LineWidth', 3);  
        % Plotting the joint axes
        axis_length = 0.1;
        
        
        plot3(L4(1,4), L4(2,4), L4(3,4), 'rx', 'MarkerSize', 10);
        axis([0 0.38 -0.38 0.38 0 0.457]);
        view(45,45);
        hold off;
        pause(0.01);
    end
end

function T = Ti(dh_row)
    % Plot a link based on DH parameters
    alpha = dh_row(1);
    a = dh_row(2);
    d = dh_row(3);
    theta = dh_row(4);

    % Transformation matrix
    %T = transformation_matrix_craig(alpha, a, d, theta);
    T = transformation_matrix(alpha, a, d, theta);
end


function T = transformation_matrix(alpha, a, d, theta)
    % Calculate DH transformation matrix
    T = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1];
end

function T = transformation_matrix_craig(alpha, a, d, theta)
    % Calculate DH transformation matrix
    T = [cos(theta) -sin(theta)  0 a;
        sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
        sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
        0 0 0 1];
end

function DH = DH_table(theta1, theta2, theta3, theta4)
    % alpha a d theta (radians)
    theta_o = asin(0.024/0.130); % offset link angle
    disp(theta_o)
    Joint1 = [deg2rad(90) 0 0.077 theta1];
    Joint2 = [0 0.13 0 theta2-theta_o];
    Joint3 = [0 0.124 0 theta3+theta_o];
    Joint4 = [0 0.126 0 theta4];
    DH = [Joint1; Joint2; Joint3; Joint4];
end
