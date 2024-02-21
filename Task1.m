close all;
% Example usage
theta_o = asin(0.024/0.130); % offset link angle
% theta1 = [-pi/2:0.01:pi/2]; % Joint 1 angle in rads
% length1=length(theta1)
% theta2 = [theta_o:0.01:pi/3+theta_o] ; % Joint 2 angle in rads
% theta2 = [theta2, zeros(1, 315 - length(theta2))];
% theta3 = [zeros(length1)]; % Joint 3 angle in rads
%theta4 = [zeros(length1)]; % Joint 4 angle in rads
angles=inverse_kinematic(0.22,0.17,0);
%disp(([angles(1,1), angles(1,2)+theta_o, angles(1,3)-theta_o, angles(1,4)]));
disp(([angles(2,1), angles(2,2)+theta_o, angles(2,3)-theta_o, angles(2,4)]));
%plot_OpenManipX(theta1, theta2, theta3, theta4);
plot_OpenManipX(0, 0, 0, 0);
plot_OpenManipX(angles(2,1), angles(2,2)+theta_o, angles(2,3)-theta_o, angles(2,4));
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
        
        plot3(L4(1,4), L4(2,4), L4(3,4), 'rx', 'MarkerSize', 10);
        axis([-0.13 0.38 -0.38 0.38 0 0.457]);
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
function IK_final = choose_kinematic(joint_angles)
    j1=rad_to_j1(joint_angles(1,1));
    j2=rad_to_j2(joint_angles(1,2)+theta_o); 
    j3=rad_to_j3(joint_angles(1,3)-theta_o);
    j4=rad_to_j4(joint_angles(1,4));

    if (j2 < 760) || (j2 > 3290) || (j3 < 695) || (j3 > 3060) || (j4 < 820) || (j4 > 3450)
        j1=rad_to_j1(joint_angles(2,1));
        j2=rad_to_j2(joint_angles(2,2)+theta_o); 
        j3=rad_to_j3(joint_angles(2,3)-theta_o);
        j4=rad_to_j4(joint_angles(2,4));
        if (j2 < 760) || (j2 > 3290) || (j3 < 695) || (j3 > 3060) || (j4 < 820) || (j4 > 3450)
            error('Joint angles are out of range for both IK solvers');
        end
    end
    IK_final = [j1, j2, j3, j4];   
end
function IK = inverse_kinematic(x_ef,y_ef,z_ef)
    z_ef=z_ef-0.077;
    theta1 = atan(y_ef/x_ef);
    r_ef = sqrt(x_ef^2+y_ef^2);
    phi = atan(z_ef/r_ef);
    r_2 = r_ef-0.126*cos(phi);
    z_2 = z_ef-0.126*sin(phi);
    theta3_plus = acos((r_2^2+z_2^2-0.13^2-0.124^2)/(2*0.13*0.124));
    theta3_minus = -acos((r_2^2+z_2^2-0.13^2-0.124^2)/(2*0.13*0.124));
    costheta2_plus = (r_2*(0.13+0.124*cos(theta3_plus))+z_2*0.124*sin(theta3_plus))/(r_2^2+z_2^2);
    costheta2_minus = (r_2*(0.13+0.124*cos(theta3_minus))+z_2*0.124*sin(theta3_minus))/(r_2^2+z_2^2);
    %sintheta2_plus = (z_2*(0.13+0.124*cos(theta3_plus))+r_2*0.124*sin(theta3_plus))/(r_2^2+z_2^2);
    %sintheta2_minus = (z_2*(0.13+0.124*cos(theta3_minus))+r_2*0.124*sin(theta3_minus))/(r_2^2+z_2^2);
    sintheta2_plus = -sqrt(1-costheta2_plus^2);
    sintheta2_minus = sqrt(1-costheta2_minus^2);
    theta2_plus = atan(sintheta2_plus/costheta2_plus);
    theta2_minus = atan(sintheta2_minus/costheta2_minus);
    theta4_plus = phi-theta2_plus-theta3_plus;
    theta4_minus = phi-theta2_minus-theta3_minus;
    IK = [theta1, theta2_plus, theta3_plus, theta4_plus; theta1, theta2_minus, theta3_minus, theta4_minus];
end

function encoder1 = rad_to_j1(theta1)
    encoder1=2048+rads_to_steps(theta1);
end

function encoder2 = rad_to_j2(theta2)
    encoder2=3072-rads_to_steps(theta2);
    if (encoder2 < 760) || (encoder2 > 3290)
        warning('Joint 2 angle is out of range');
    end
end
function encoder3 = rad_to_j3(theta3)
    encoder3=1024-rads_to_steps(theta3);
    if (encoder3 < 695) || (encoder3 > 3060)
        warning('Joint 3 angle is out of range');
    end
end
function encoder4 = rad_to_j4(theta4)
    encoder4=2048-rads_to_steps(theta4);
    if (encoder4 < 820) || (encoder4 > 3450)
        warning('Joint 4 angle is out of range');
    end
end
function steps = rads_to_steps(rads)
    step = (2*pi)/4096;
    steps = rads/step;
end