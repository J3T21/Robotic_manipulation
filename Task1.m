



% Example usage
theta1 = 0; % Joint 1 angle in degrees
theta2 = 0; % Joint 2 angle in degrees
theta3 = 0; % Joint 3 angle in degrees
theta4 = 0; % Joint 4 angle in degrees

plot_OpenManipX(theta1, theta2, theta3, theta4);


function plot_OpenManipX(theta1, theta2, theta3, theta4)
    % DH parameters for OpenManipulator-X
    % alpha a d theta (degrees)
    DH = OpenManipX(theta1, theta2, theta3, theta4);

    % Plot robot
    figure;
    hold on;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);

    % Plot links
    base = [0; 0; 0];
    L1 = plot_link(base, DH(1,:), 'r');
    L2 = plot_link(L1, DH(2,:), 'g');
    L3 = plot_link(L2, DH(3,:), 'b');
    L4 = plot_link(L3, DH(4,:), 'k');

    % Plot end-effector
    plot3(L4(1), L4(2), L4(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);

    title('OpenManipulator-X');
    axis equal;
    hold off;
end

function link_end = plot_link(base, dh, color)
    % Plot a link based on DH parameters
    alpha = dh(1);
    a = dh(2);
    d = dh(3);
    theta = dh(4);

    % Transformation matrix
    T = dh_matrix(alpha, a, d, theta);

    % Define points
    P1 = [0; 0; 0; 1];
    P2 = [a; 0; 0; 1];
    P3 = [a; 0; -d; 1];
    P4 = [0; 0; -d; 1];
    
    % Apply transformation
    P1 = T * P1;
    P2 = T * P2;
    P3 = T * P3;
    P4 = T * P4;
    
    % Plot link
    plot3([P1(1) P2(1) P3(1) P4(1)], [P1(2) P2(2) P3(2) P4(2)], [P1(3) P2(3) P3(3) P4(3)], color, 'LineWidth', 3);

    % Compute end point of the link
    link_end = base + T(1:3,4);
end


function T = dh_matrix(alpha, a, d, theta)
    % Calculate DH transformation matrix
    T = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*cos(alpha) a*sin(theta);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1];
end

function DH = OpenManipX(theta1, theta2, theta3, theta4)
    % alpha a d theta metres radians
    theta_o = asin(0.024/0.128); %offset link angle
    Joint1 = [deg2rad(90) 0 0.077 theta1];
    Joint2 = [0 0.13 0 theta2-theta_o];
    Joint3 = [0 0.124 0 theta3+theta_o];
    Joint4 = [0 0.126 0 theta4];
    DH = [Joint1;Joint2;Joint3;Joint4]; 
end
