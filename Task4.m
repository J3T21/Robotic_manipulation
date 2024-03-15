close all;
clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116;
ADDR_PRO_PRESENT_POSITION    = 132;
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_PRO_GOAL_VELOCITY       = 112;
ADDR_ACCEL = 108;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                      = 11;            % Dynamixel ID: 1
DXL_ID_2                      = 12;            % Dynamixel ID: 2
DXL_ID_3                     = 13;            % Dynamixel ID: 3
DXL_ID_4                      = 14;            % Dynamixel ID: 4
DXL_ID_5                      = 15;            % Dynamixel ID: 5
BAUDRATE                    = 115200;
DEVICENAME                  = '/dev/tty.usbserial-FT5WJ6Z6';       % Check which port is being used on your controller
% ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4096;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
global port_num;
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end



% ----- SET MOTION LIMITS ----------- %
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
% MAX_POS = 3400;
% MIN_POS = 600;
% % Set max position limit
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MAX_POS, MAX_POS);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MAX_POS, MAX_POS);
% % Set min position limit
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MIN_POS, MIN_POS);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MIN_POS, MIN_POS);
% ---------------------------------- %


dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% enter move function here depending on the position of the dynamixel

% set to control mode
command(1,ADDR_PRO_OPERATING_MODE,3);
% limit speed so it doesnt spaz
command(4,ADDR_PRO_GOAL_VELOCITY,200);
%limit accel
command(4,ADDR_ACCEL,1);
% enable tourque
command(1,ADDR_PRO_TORQUE_ENABLE,1);

%mary_had = 'AGFG|AAA2|GGG2|AAA2|AGFG|AAAA|GGAG|F4|]';
%abc_to_instr(mary_had);try this --james

%sailors = 'cB|c2C2 C2GF|EGc2 cedc|d2D2 D2GF|EFGA G4|fefg agfe|fedc cBAG|AcBd cedf|e2c2 c2:||:GF|EGcG EGcG|A2F2 F2FE|DAdA DAdA|B2G2 G2e2|fefg agfe|fdec cBAG|AcBd cedf|e2c2 c2:|';
%abc_to_instr(sailors);

notes = ["G" , "A" ,"B","C","C1","D1","E1"];
%notes = ["D" , "D" ,"D"];


%2.8 -7.3
% encoders_cubic=cubic_trajectory(target_pos, finish_pos, 5);
% encoders_rot = simple_trajectory(target_pos_rot,finish_pos_rot);
move(2000,2000,2000,2000,2400);
encoders=simple_trajectory(notes);

% for i=1:height(encoders)
%     % move(encoders_rot(i,1),encoders_rot(i,2),encoders_rot(i,3),encoders_rot(i,4),encoders_rot(i,5))
%     move(encoders(i,1),encoders(i,2),encoders(i,3),encoders(i,4),encoders(i,5));
%     %move(encoders_cubic(i,1),encoders_cubic(i,2),encoders_cubic(i,3),encoders_cubic(i,4),encoders_cubic(i,5));
% end
%move(2000,2000,2000,2000,2450);

command(1,ADDR_PRO_TORQUE_ENABLE,1);
% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);


close all;
clear all;
%pos([x,y,z,phi])





function move(encoder1,encoder2,encoder3,encoder4,encoder5)
    global port_num;
    ADDR_PRO_GOAL_POSITION       = 116;
    PROTOCOL_VERSION            = 2.0;

    DXL_ID                      = 11;            % Dynamixel ID: 1
    DXL_ID_2                      = 12;            % Dynamixel ID: 2
    DXL_ID_3                     = 13;            % Dynamixel ID: 3
    DXL_ID_4                      = 14;            % Dynamixel ID: 4
    DXL_ID_5                      = 15;




    % Write goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, encoder1);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder5);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, encoder4);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, encoder3);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, encoder2);

    pause(1.2);

end
function move_fast(encoder1,encoder2,encoder3,encoder4,encoder5)
    global port_num;
    ADDR_PRO_GOAL_POSITION       = 116;
    PROTOCOL_VERSION            = 2.0;

    DXL_ID                      = 11;            % Dynamixel ID: 1
    DXL_ID_2                      = 12;            % Dynamixel ID: 2
    DXL_ID_3                     = 13;            % Dynamixel ID: 3
    DXL_ID_4                      = 14;            % Dynamixel ID: 4
    DXL_ID_5                      = 15;

    % Write goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, encoder1);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder5);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, encoder4);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, encoder3);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, encoder2);

    pause(0.1);

end
function note_coords = getnote(note)
    z = 0.035;
    saggin_z = 0.01;
    centretocentre = 0.03;
    keywidth =2.16;
    gaps = 0.93;
    x = 0.31;
    switch note
        case 'G'
            note_coords = [x,-0.17,z,-pi/10];
        case 'A'
            note_coords = [x,-0.145,z,-pi/10];
        case 'B'
            note_coords = [x,-0.11,z,-pi/10];
        case 'C'
            note_coords = [x,-0.077,z,-pi/10];
        case 'D'
            note_coords = [x-0.13,-0.044,z-0.2];
        case 'E'
            note_coords = [x-0.05,-0.031,saggin_z];
        case 'F'
            note_coords = [x-0.05,0,saggin_z];
        case 'G1'
            note_coords = [x-0.05,0.031,saggin_z];
        case 'A1'
            note_coords = [x-0.05,0.062,saggin_z];
        case 'B1'
            note_coords = [x,0.080,z];
        case 'C1'
            note_coords = [x,0.12,z,-pi/10];
        case 'D1'
            note_coords = [x,0.145,z,-pi/10];
        case 'E1'
            note_coords = [x,0.186,z,-pi/10];

        otherwise
            note_coords = [x,0,z];       
    end
end

function note_coords = getnote_abs(note)

    switch note
        case 'G'
            note_coords = [1703,1331,2648,2426];
        case 'A'
            note_coords = [1750,1236,2648,2534];
        case 'B'
            note_coords = [1800,1090,2737,2560];
        case 'C'
            note_coords = [1863,966,2740,2670];
        case 'D'
            note_coords = [1920,966,2740,2670];
        case 'E'
            note_coords = [1970,883,2740,2760];
        case 'F'
            note_coords = [2038,883,2740,2760];
        case 'G1'
            note_coords = [2104,883,2740,2761];
        case 'A1'
            note_coords = [2158,883,2740,2761];
        case 'B1'
            note_coords = [2217,877,2791,2700];
        case 'C1'
            note_coords = [2271,1080,2704,2635];
        case 'D1'
            note_coords = [2330,926,2878,2547];
        case 'E1'
            note_coords = [2382,926,2943,2445];

        otherwise
            note_coords = [2038,883,2740,2760];       
    end
end
function IK_final = choose_kinematic(joint_angles)
    j1=rad_to_j1(joint_angles(2,1));
    j2=rad_to_j2(joint_angles(2,2)); 
    j3=rad_to_j3(joint_angles(2,3));
    j4=rad_to_j4(joint_angles(2,4));
    if (j2 < 704) || (j2 > 3290) || (j3 < 695) || (j3 > 3060) || (j4 < 820) || (j4 > 3450)
        warning('IK using elbow up out of range')
        j1=rad_to_j1(joint_angles(1,1));
        j2=rad_to_j2(joint_angles(1,2)); 
        j3=rad_to_j3(joint_angles(1,3));
        j4=rad_to_j4(joint_angles(1,4));
        if (j2 < 704) || (j2 > 3290) || (j3 < 695) || (j3 > 3060) || (j4 < 820) || (j4 > 3450)
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
    r_2 = r_ef-0.385*cos(phi);
    z_2 = z_ef-0.385*sin(phi);
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

function trajec_encoders = simple_trajectory(notes)
    encoder_sequence = [];
    intermediate_height=0.2; %height for block moving
    mouth_close=2400;
    for i=1:length(notes)
        xyz = getnote(notes(i));
        % go above note
        %TO DO MAKE IT SLOWER AND RESET BAK SPEED AFTER KEY MOVEMENT
        disp(xyz)
        position = inverse_kinematic(xyz(1),xyz(2),intermediate_height,pi/20);
        position_kinematic = choose_kinematic(position);
        encoder1_int = rad_to_j1(position_kinematic(1));
        encoder2_int = rad_to_j2(position_kinematic(2));
        encoder3_int = rad_to_j3(position_kinematic(3));
        encoder4_int = rad_to_j4(position_kinematic(4));
        % encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_close];
        move(encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_close);
        % go to note
        position = inverse_kinematic(xyz(1),xyz(2),xyz(3),xyz(4));
        position_kinematic = choose_kinematic(position);
        encoder1 = rad_to_j1(position_kinematic(1));
        encoder2 = rad_to_j2(position_kinematic(2));
        encoder3 = rad_to_j3(position_kinematic(3));
        encoder4 = rad_to_j4(position_kinematic(4));
        % encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_close];
        move_fast(encoder1,encoder2,encoder3,encoder4,mouth_close);
        % go above note
        move_fast(encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_close);
    end
    trajec_encoders = encoder_sequence;
end
%function IK = inverse_kinematic_2(x_ef, y_ef, z_ef)
function trajec_encoders = super_simple_trajectory(notes)
    encoder_sequence = [];
    intermediate_height=-100; %height for block moving
    mouth_close=2400;
    for i=1:length(notes)
        xyz = getnote_abs(notes(i));
        % go above note
        %TO DO MAKE IT SLOWER AND RESET BAK SPEED AFTER KEY MOVEMENT
        % encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_close];
        move(xyz(1),xyz(2),xyz(3)+intermediate_height,xyz(4),mouth_close);
        % go to note
        % encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_close];
        move_fast(xyz(1),xyz(2),xyz(3),xyz(4),mouth_close);
        % go above note
        move_fast(xyz(1),xyz(2),xyz(3)+intermediate_height,xyz(4),mouth_close);
    end
    trajec_encoders = encoder_sequence;
end

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

function command(bytes,register,value)
    global port_num;
    DXL_ID                      = 11;            % Dynamixel ID: 1
    DXL_ID_2                      = 12;            % Dynamixel ID: 2
    DXL_ID_3                     = 13;            % Dynamixel ID: 3
    DXL_ID_4                      = 14;            % Dynamixel ID: 4
    DXL_ID_5                      = 15;
    if bytes == 1
        write1ByteTxRx(port_num, 2, DXL_ID, register, value);
        write1ByteTxRx(port_num, 2, DXL_ID_2, register, value);
        write1ByteTxRx(port_num, 2, DXL_ID_3, register, value);
        write1ByteTxRx(port_num, 2, DXL_ID_4, register, value);
        write1ByteTxRx(port_num, 2, DXL_ID_5, register, value);
    else
        write4ByteTxRx(port_num, 2, DXL_ID, register, value);
        write4ByteTxRx(port_num, 2, DXL_ID_2, register, value);
        write4ByteTxRx(port_num, 2, DXL_ID_3, register, value);
        write4ByteTxRx(port_num, 2, DXL_ID_4, register, value);
        write4ByteTxRx(port_num, 2, DXL_ID_5, register, value);
    end
end

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



function DH = DH_table(theta1, theta2, theta3, theta4)
    % alpha a d theta (radians)
    theta_o = asin(0.024/0.130); % offset link angle
    Joint1 = [deg2rad(90) 0 0.077 theta1];
    Joint2 = [0 0.13 0 theta2-theta_o];
    Joint3 = [0 0.124 0 theta3+theta_o];
    Joint4 = [0 0.126 0 theta4];
    DH = [Joint1; Joint2; Joint3; Joint4];
end

function instr = abc_to_instr(abc) 
    % split string into chars
    abc=strrep(abc, ']','');
    chars = char(strsplit(strrep(abc, '|', ''), ''));
    notes=chars;
    default_pause=1;
    fast_pause = 0.1;
    mouth_close=2400;
    intermediate_height=-100;
    % disp(notes);
    % disp(isstrprop(notes(1),"alpha"))
    for i = 1:length(notes)
        if (i<length(notes))
            if (isstrprop(notes(i),"alpha") && ~contains(notes(i+1),','))%normal note case
                xyz = getnote_abs(notes(i));
                % disp(i);
                % disp(xyz)
            move_variable(xyz(1),xyz(2),xyz(3)+intermediate_height,xyz(4),mouth_close,default_pause);
            move_variable(xyz(1),xyz(2),xyz(3),xyz(4),mouth_close,fast_pause);
            elseif isstrprop(notes(i),"digit")%pause case with following note
                pause_time = str2double(notes(i));
                xyz = getnote_abs(notes(i+1));
                % disp(i+1);
                % disp(xyz)
                move_variable(xyz(1),xyz(2),xyz(3)+intermediate_height,xyz(4),mouth_close,pause_time*default_pause-default_pause);
            else
                xyz = getnote_abs(strcat(notes(i),','));
                % disp(i);
                % disp(xyz)
            move_variable(xyz(1),xyz(2),xyz(3)+intermediate_height,xyz(4),mouth_close,default_pause);
            move_variable(xyz(1),xyz(2),xyz(3),xyz(4),mouth_close,fast_pause);    
            end%account for movement time
        elseif (isstrprop(notes(i),"digit"))%pause at end of song
            xyz = getnote_abs(notes(i-1));
            % disp(i-1);
            % disp(xyz)
            move_variable(xyz(1),xyz(2),xyz(3)+intermediate_height,xyz(4),mouth_close,pause_time*default_pause-default_pause);
            %account for movement time
        elseif isstrprop(notes(i),"alpha")%final note
            xyz = getnote_abs(notes(i));
            % disp(i);
            % disp(xyz)
            move_variable(xyz(1),xyz(2),xyz(3)+intermediate_height,xyz(4),mouth_close,default_pause);
            move_variable(xyz(1),xyz(2),xyz(3),xyz(4),mouth_close,fast_pause);
        else
            warning('Invalid note');
        end
        
    end
    %move_variable(2048,2048,2048,2048,mouth_close,default_pause);
end

function note_coords = getnote_abs_new(note)

    switch note
        case ("G,")
            note_coords = [1703,1331,2648,2426];
        case ("A,")
            note_coords = [1750,1236,2648,2534];
        case ("B,")
            note_coords = [1800,1090,2737,2560];
        case 'C'
            note_coords = [1863,966,2740,2670];
        case 'D'
            note_coords = [1920,966,2740,2670];
        case 'E'
            note_coords = [1970,883,2740,2760];
        case 'F'
            note_coords = [2038,883,2740,2760];
        case 'G'
            note_coords = [2104,883,2740,2761];
        case 'A'
            note_coords = [2158,883,2740,2761];
        case 'B'
            note_coords = [2217,877,2791,2700];
        case 'c'
            note_coords = [2271,1080,2704,2635];
        case 'd'
            note_coords = [2330,926,2878,2547];
        case 'e'
            note_coords = [2382,926,2943,2445];

        otherwise
            note_coords = [2038,883,2740,2760];       
    end
end


function move_variable(encoder1,encoder2,encoder3,encoder4,encoder5,pause_val)
    global port_num;
    ADDR_PRO_GOAL_POSITION       = 116;
    PROTOCOL_VERSION            = 2.0;

    DXL_ID                      = 11;            % Dynamixel ID: 1
    DXL_ID_2                      = 12;            % Dynamixel ID: 2
    DXL_ID_3                     = 13;            % Dynamixel ID: 3
    DXL_ID_4                      = 14;            % Dynamixel ID: 4
    DXL_ID_5                      = 15;

    % Write goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, encoder1);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder5);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, encoder4);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, encoder3);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, encoder2);

    pause(pause_val);
end

