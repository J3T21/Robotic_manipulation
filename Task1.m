close all;
% Example usage
%theta_o = asin(0.024/0.130); % offset link angle
%open 2000 closed on block 2350 
%%% Forward kinematic simulation 
%{
theta1 = [-pi/2:0.01:pi/2, pi/2:-0.01:0,zeros(1,777)]; % Joint 1 angle in rads
theta2 = [zeros(1,473),0:-0.01:-0.33,-0.33:0.01:3.54,3.54:-0.01:0];
theta3 = [zeros(1,473),0:0.01:0.504, 0.504:-0.01:-3.12 , -3.12:0.01:0 , zeros(1,50)];
theta4 = [zeros(1,441),0:0.01:1.8837,1.8837:-0.01:-2.15 , -2.15:0.01:0];
plot_OpenManipX(theta1, theta2, theta3, theta4,0);
%}
%%% Backwards kinematic simuation 
%{
%%% xy-plane
angles_xy=[choose_kinematic(inverse_kinematic(0.1,-0.1,0,-pi/2)); 
        choose_kinematic(inverse_kinematic(0.2,-0.1,0,-pi/2));
        choose_kinematic(inverse_kinematic(0.2,-0.0,0,-pi/2));
        choose_kinematic(inverse_kinematic(0.1,-0.0,0,-pi/2));
        choose_kinematic(inverse_kinematic(0.1,-0.1,0,-pi/2))];
plot_OpenManipX(angles_xy(:,1), angles_xy(:,2), angles_xy(:,3), angles_xy(:,4),1);
%%% xz-plane
angles_xz=[choose_kinematic(inverse_kinematic(0.05,0,0,-pi/2)); 
        choose_kinematic(inverse_kinematic(0.15,0,0,-pi/2));
        choose_kinematic(inverse_kinematic(0.15,0,0.1,-pi/2));
        choose_kinematic(inverse_kinematic(0.05,0,0.1,-pi/2));
        choose_kinematic(inverse_kinematic(0.05,0,0,-pi/2))];
plot_OpenManipX(angles_xz(:,1), angles_xz(:,2), angles_xz(:,3), angles_xz(:,4),1);
%%% yz-plane
angles_yz=[choose_kinematic(inverse_kinematic(0,-0.05,0,-pi/2)); 
        choose_kinematic(inverse_kinematic(0,-0.15,0,-pi/2));
        choose_kinematic(inverse_kinematic(0,-0.15,0.1,-pi/2));
        choose_kinematic(inverse_kinematic(0,-0.05,0.1,-pi/2));
        choose_kinematic(inverse_kinematic(0,-0.05,0,-pi/2))];
plot_OpenManipX(angles_yz(:,1), angles_yz(:,2), angles_yz(:,3), angles_yz(:,4),1);
%}
% x=inverse_kinematic(0.225,0,0.02);
% j1=rad_to_j1(x(1))
% j2=rad_to_j2(x(2))
% j3=rad_to_j3(x(3))
% j4=rad_to_j4(x(4))
% plot_OpenManipX(x(1,1),x(1,2),x(1,3),x(1,4),0)
% plot_OpenManipX(x(2,1),x(2,2),x(2,3),x(2,4),0)
% x2=choose_kinematic(inverse_kinematic(0.1,0.15,0.02));
% j1=rad_to_j1(x2(1))
% j2=rad_to_j2(x2(2))
% j3=rad_to_j3(x2(3))
% j4=rad_to_j4(x2(4))
% plot_OpenManipX(x2(1),x2(2),x2(3),x2(4),0)
%plot_OpenManipX(x2(2,1),x2(2,2),x2(2,3),x2(2,4),0)
xyz = coords_to_meters(3,-8,0.08);
position = inverse_kinematic(xyz(1),xyz(2),xyz(3),-pi/2);
position_kinematic = choose_kinematic(position);
plot_OpenManipX(position(1,1),position(1,2),position(1,3),position(1,4),0);
plot_OpenManipX(position(2,1),position(2,2),position(2,3),position(2,4),0);
plot_OpenManipX(position_kinematic(1),position_kinematic(2),position_kinematic(3),position_kinematic(4),0);
function plot_OpenManipX(theta1, theta2, theta3, theta4, draw_lines)
    % DH parameters for OpenManipulator-X
    % alpha a d theta (degrees)
    figure;
    set(gcf, 'Position', get(0, 'Screensize'));
    colororder("glow12");
    if length(theta1) ~= length(theta2) || length(theta2) ~= length(theta3) || length(theta3) ~= length(theta4)
        error('Input angle arrays must have the same length');
    end
    trace = [];
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
       
        quiver3(L0(1,4), L0(2,4), L0(3,4), L0(1,1), L0(2,1), L0(3,1), 0.01, 'r', 'LineWidth', 1);
        quiver3(L0(1,4), L0(2,4), L0(3,4), L0(1,2), L0(2,2), L0(3,2), 0.01, 'g', 'LineWidth', 1);
        quiver3(L0(1,4), L0(2,4), L0(3,4), L0(1,3), L0(2,3), L0(3,3), 0.01, 'b', 'LineWidth', 1);
        quiver3(L1(1,4), L1(2,4), L1(3,4), L1(1,1), L1(2,1), L1(3,1), 0.01, 'r', 'LineWidth', 1);
        quiver3(L1(1,4), L1(2,4), L1(3,4), L1(1,2), L1(2,2), L1(3,2), 0.01, 'g', 'LineWidth', 1);
        quiver3(L1(1,4), L1(2,4), L1(3,4), L1(1,3), L1(2,3), L1(3,3), 0.01, 'b', 'LineWidth', 1);
        quiver3(L2(1,4), L2(2,4), L2(3,4), L2(1,1), L2(2,1), L2(3,1), 0.01, 'r', 'LineWidth', 1);
        quiver3(L2(1,4), L2(2,4), L2(3,4), L2(1,2), L2(2,2), L2(3,2), 0.01, 'g', 'LineWidth', 1);
        quiver3(L2(1,4), L2(2,4), L2(3,4), L2(1,3), L2(2,3), L2(3,3), 0.01, 'b', 'LineWidth', 1);
        quiver3(L3(1,4), L3(2,4), L3(3,4), L3(1,1), L3(2,1), L3(3,1), 0.01, 'r', 'LineWidth', 1);
        quiver3(L3(1,4), L3(2,4), L3(3,4), L3(1,2), L3(2,2), L3(3,2), 0.01, 'g', 'LineWidth', 1);
        quiver3(L3(1,4), L3(2,4), L3(3,4), L3(1,3), L3(2,3), L3(3,3), 0.01, 'b', 'LineWidth', 1);
        quiver3(L4(1,4), L4(2,4), L4(3,4), L4(1,1), L4(2,1), L4(3,1), 0.01, 'r', 'LineWidth', 1);
        quiver3(L4(1,4), L4(2,4), L4(3,4), L4(1,2), L4(2,2), L4(3,2), 0.01, 'g', 'LineWidth', 1);
        quiver3(L4(1,4), L4(2,4), L4(3,4), L4(1,3), L4(2,3), L4(3,3), 0.01, 'b', 'LineWidth',1);

        plot3([L0(1,4),L1(1,4)],[L0(2,4),L1(2,4)],[L0(3,4),L1(3,4)],'red','LineWidth', 3);
        plot3([L1(1,4),L2(1,4)],[L1(2,4),L2(2,4)],[L1(3,4),L2(3,4)],'green','LineWidth', 3);
        plot3([L2(1,4),L3(1,4)],[L2(2,4),L3(2,4)],[L2(3,4),L3(3,4)],'blue','LineWidth', 3);
        plot3([L3(1,4),L4(1,4)],[L3(2,4),L4(2,4)],[L3(3,4),L4(3,4)],'black','LineWidth', 3);          
        plot3(L4(1,4), L4(2,4), L4(3,4), 'rx', 'MarkerSize', 10);
        if draw_lines
            trace=[trace,L4(1:3,4)];
            disp(size(trace));
            for j=1:(size(trace,2)-1)
                plot3([trace(1,j),trace(1,j+1)],[trace(2,j), trace(2,j+1)],[trace(3,j),trace(3,j+1)],'LineWidth', 3);
            end            
        end        
        axis([-0.15 0.38 -0.38 0.38 0 0.457]);
        view(45,45);
        hold off;
        pause(1);
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
    Joint1 = [deg2rad(90) 0 0.077 theta1];
    Joint2 = [0 0.13 0 theta2-theta_o];
    Joint3 = [0 0.124 0 theta3+theta_o];
    Joint4 = [0 0.126 0 theta4];
    DH = [Joint1; Joint2; Joint3; Joint4];
end

function IK_final = choose_kinematic(joint_angles)
    j1=rad_to_j1(joint_angles(2,1));
    j2=rad_to_j2(joint_angles(2,2)); 
    j3=rad_to_j3(joint_angles(2,3));
    j4=rad_to_j4(joint_angles(2,4));
    if (j2 < 760) || (j2 > 3290) || (j3 < 695) || (j3 > 3060) || (j4 < 820) || (j4 > 3450)
        warning('IK using elbow up out of range')
        j1=rad_to_j1(joint_angles(1,1));
        j2=rad_to_j2(joint_angles(1,2)); 
        j3=rad_to_j3(joint_angles(1,3));
        j4=rad_to_j4(joint_angles(1,4));
        if (j2 < 760) || (j2 > 3290) || (j3 < 695) || (j3 > 3060) || (j4 < 820) || (j4 > 3450)
            error('Joint angles are out of range for both IK solvers');
        else
            IK_final = joint_angles(1,:);
        end
    else
        IK_final = joint_angles(2,:);
    end
end

function IK = inverse_kinematic(x_ef,y_ef,z_ef,phi)
    theta_o = asin(0.024/0.130);
    z_ef=z_ef-0.077;
    theta1 = atan2(y_ef,x_ef);
    r_ef = sqrt(x_ef^2+y_ef^2);
    r_2 = r_ef-0.126*cos(phi);
    z_2 = z_ef-0.126*sin(phi);
    theta3_plus = acos((r_2^2+z_2^2-0.13^2-0.124^2)/(2*0.13*0.124));
    theta3_minus = -acos((r_2^2+z_2^2-0.13^2-0.124^2)/(2*0.13*0.124));
    costheta2_plus = (r_2*(0.13+0.124*cos(theta3_plus))+z_2*0.124*sin(theta3_plus))/(r_2^2+z_2^2);
    costheta2_minus = (r_2*(0.13+0.124*cos(theta3_minus))+z_2*0.124*sin(theta3_minus))/(r_2^2+z_2^2);
    sintheta2_plus = sqrt(1-costheta2_plus^2);
    sintheta2_minus = sqrt(1-costheta2_minus^2);
    theta2_plus = atan2(sintheta2_plus,costheta2_plus);
    theta2_minus = atan2(sintheta2_minus,costheta2_minus);
    theta4_plus = phi-theta2_plus-theta3_plus;
    theta4_minus = phi-theta2_minus-theta3_minus;
    IK = [theta1, theta2_plus+theta_o, theta3_plus-theta_o, theta4_plus; theta1, theta2_minus+theta_o, theta3_minus-theta_o, theta4_minus];
end

%function IK = inverse_kinematic_2(x_ef, y_ef, z_ef)

function encoder1 = rad_to_j1(theta1)
    encoder1=2048+rads_to_steps(theta1);
end

function encoder2 = rad_to_j2(theta2)
    encoder2=3072-rads_to_steps(theta2);
    % if (encoder2 < 760) || (encoder2 > 3290)
    %     warning('Joint 2 angle is out of range');
    % end
end
function encoder3 = rad_to_j3(theta3)
    encoder3=1024-rads_to_steps(theta3);
    % if (encoder3 < 695) || (encoder3 > 3060)
    %     warning('Joint 3 angle is out of range');
    % end
end
function encoder4 = rad_to_j4(theta4)
    encoder4=2048-rads_to_steps(theta4);
    % if (encoder4 < 820) || (encoder4 > 3450)
    %     warning('Joint 4 angle is out of range');
    % end
end
function steps = rads_to_steps(rads)
    step = (2*pi)/4096;
    steps = rads/step;
end

function coords = coords_to_meters(x,y,z)
    coords = [x*0.025, y*0.025, z];
end
