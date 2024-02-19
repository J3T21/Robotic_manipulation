close all;
% Example usage
theta1 = 0; % Joint 1 angle in degrees
theta2 = 0; % Joint 2 angle in degrees
theta3 = 0; % Joint 3 angle in degrees
theta4 = 0; % Joint 4 angle in degrees

plot_OpenManipX(theta1, theta2, theta3, theta4);

function plot_OpenManipX(theta1, theta2, theta3, theta4)
    % DH parameters for OpenManipulator-X
    % alpha a d theta (degrees)
    DH = DH_table(theta1, theta2, theta3, theta4);

    % Plot robot
    figure;
    hold on;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);

    % Plot links
    base = [0; 0; 0; 1]; % Changed to column vector
    L1 = plot_link(base, DH(1,:), 'r');
    L2 = plot_link(L1, DH(2,:), 'g');
    L3 = plot_link(L2, DH(3,:), 'b');
    L4 = plot_link(L3, DH(4,:), 'k');
    disp(eye(4))
    disp(L1)
    disp(L2)
    disp(L3)
    disp(L4)
    % Plot end-effector
    plot3(L4(1), L4(2), L4(3), 'kx', 'MarkerSize', 8, 'LineWidth', 2);
    title('OpenManipulator-X');
    axis equal;
    hold off;
end

function link_end = plot_link(start, dh_row, color)
    % Plot a link based on DH parameters
    alpha = dh_row(1);
    a = dh_row(2);
    d = dh_row(3);
    theta = dh_row(4);

    % Transformation matrix
    T = transformation_matrix(alpha, a, d, theta);
    
    final = T*start; % Corrected order of multiplication
    % Plot link
    plot3([start(1), final(1)], [start(2), final(2)], [start(3), final(3)], color, 'LineWidth', 3);

    % Compute end point of the link
    link_end = final; % Use the transformed endpoint
end

function T = Ti(dh_row)
    % Plot a link based on DH parameters
    alpha = dh_row(1);
    a = dh_row(2);
    d = dh_row(3);
    theta = dh_row(4);

    % Transformation matrix
    T = transformation_matrix_craig(alpha, a, d, theta);
end


function T = transformation_matrix(alpha, a, d, theta)
    % Calculate DH transformation matrix
    T = [cosd(theta) -sind(theta)*cosd(alpha)  sind(theta)*sind(alpha) a*cosd(theta);
        sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
        0 sind(alpha) cosd(alpha) d;
        0 0 0 1];
end

function T = transformation_matrix_craig(alpha, a, d, theta)
    % Calculate DH transformation matrix
    T = [cosd(theta) -sind(theta)  0 a;
        sind(theta)*cosd(alpha) cosd(theta)*cosd(alpha) -sind(alpha) -sind(alpha)*d;
        sind(theta)*sind(alpha) cosd(theta)*sind(alpha) cosd(alpha) cosd(alpha)*d;
        0 0 0 1];
end

function DH = DH_table(theta1, theta2, theta3, theta4)
    % alpha a d theta (radians)
    theta_o = asind(0.024/0.130); % offset link angle
    disp(theta_o)
    Joint1 = [90 0 0.077 theta1];
    Joint2 = [0 0.13 0 theta2-theta_o];
    Joint3 = [0 0.124 0 theta3+theta_o];
    Joint4 = [0 0.126 0 theta4];
    DH = [Joint1; Joint2; Joint3; Joint4];
    disp(DH)
end
