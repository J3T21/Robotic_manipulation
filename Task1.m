close all;
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

position2 = inverse_kinematic(0.075,-0.2,0.025,-pi/2);
position2_kinematic = choose_kinematic(position2);
plot_OpenManipX(position2_kinematic(1),position2_kinematic(2),position2_kinematic(3),position2_kinematic(4),0);
%}

% xyz = coords_to_meters(3,-8,0.07);
% position = inverse_kinematic(xyz(1),xyz(2),xyz(3),-pi/2);
% position_kinematic = choose_kinematic(position);
% plot_OpenManipX(position(1,1),position(1,2),position(1,3),position(1,4),0);
% plot_OpenManipX(position(2,1),position(2,2),position(2,3),position(2,4),0);
% plot_OpenManipX(position_kinematic(1),position_kinematic(2),position_kinematic(3),position_kinematic(4),0);
angles1=[choose_kinematic(inverse_kinematic(0,0.2,0,-pi/2))];
angles2=[choose_kinematic(inverse_kinematic(0.2,0.1,0.1,pi/2))];

Interp = cubic_intermediate_joints(angles1,angles2,5);
plot_OpenManipX(Interp(1,:),Interp(2,:),Interp(3,:),Interp(4,:),1);


%disp(cubic_interp_cartesian([0,0,0,0],[0.1,0.1,0.1,0.1]));
function plot_OpenManipX(theta1, theta2, theta3, theta4, draw_lines)
    % DH parameters for OpenManipulator-X
    % alpha a d theta (degrees)
    figure;
    set(gcf, 'Position', get(0, 'Screensize'));
    %colororder("glow12");
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
        % TODO: plot axis instead of points
        

        plot3([L0(1,4),L1(1,4)],[L0(2,4),L1(2,4)],[L0(3,4),L1(3,4)],'red','LineWidth', 3);
        plot3([L1(1,4),L2(1,4)],[L1(2,4),L2(2,4)],[L1(3,4),L2(3,4)],'green','LineWidth', 3);
        plot3([L2(1,4),L3(1,4)],[L2(2,4),L3(2,4)],[L2(3,4),L3(3,4)],'blue','LineWidth', 3);
        plot3([L3(1,4),L4(1,4)],[L3(2,4),L4(2,4)],[L3(3,4),L4(3,4)],'black','LineWidth', 3);          
        plot3(L4(1,4), L4(2,4), L4(3,4), 'rx', 'MarkerSize', 10);
        if draw_lines
            trace=[trace,L4(1:3,4)];
            %disp(size(trace));
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

function joint_thetas = cubic_intermediate_joints(pos_start, pos_end, time)
    theta1_start = pos_start(1);
    theta2_start = pos_start(2);
    theta3_start = pos_start(3);
    theta4_start = pos_start(4);
    theta1_end = pos_end(1);
    theta2_end = pos_end(2);
    theta3_end = pos_end(3);
    theta4_end = pos_end(4);
    t = 0:0.1:time;
    theta1 = cubic_theta(t, theta1_start, theta1_end, time);
    theta2 = cubic_theta(t, theta2_start, theta2_end, time);
    theta3 = cubic_theta(t, theta3_start, theta3_end, time);
    theta4 = cubic_theta(t, theta4_start, theta4_end, time);
    joint_thetas = [theta1; theta2; theta3; theta4];  
end

function angle = cubic_theta(t, theta_start, theta_end, tf)
    a0=theta_start;
    a1=0;
    a2=3*(theta_end-theta_start)/tf^2;
    a3=-2*(theta_end-theta_start)/tf^3;
    angle= a0+a1*t+a2*t.^2+a3*t.^3;
end

function encoder1 = rad_to_j1(theta1)
    encoder1=2048+rads_to_steps(theta1);
end

function encoder2 = rad_to_j2(theta2)
    encoder2=3072-rads_to_steps(theta2);

end
function encoder3 = rad_to_j3(theta3)
    encoder3=1024-rads_to_steps(theta3);

end
function encoder4 = rad_to_j4(theta4)
    encoder4=2048-rads_to_steps(theta4);

end
function steps = rads_to_steps(rads)
    step = (2*pi)/4096;
    steps = rads/step;
end
function coords = coords_to_meters(x,y,z)
    coords = [x*0.025, y*0.025, z];
end