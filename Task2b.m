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
DEVICENAME                  = '/dev/cu.usbserial-FT5WJ6Z6';       % Check which port is being used on your controller
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
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, 3);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_OPERATING_MODE, 3);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_OPERATING_MODE, 3);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_OPERATING_MODE, 3);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_OPERATING_MODE, 3);
command(1,ADDR_PRO_OPERATING_MODE,3);
% limit speed so it doesnt spaz
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_VELOCITY, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_VELOCITY, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_VELOCITY, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_VELOCITY, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_VELOCITY, 1000);
%limit accel
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_ACCEL, 10);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_ACCEL, 10);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_ACCEL, 10);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_ACCEL, 10);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_ACCEL, 10);
% enable tourque
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 1);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 1);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 1);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, 1);
command(1,ADDR_PRO_TORQUE_ENABLE,1);

target_pos = [3, -8, 0.058, -pi/2; 9, 0, 0.058, -pi/4; 6, 6, 0.058, -pi/4;];
finish_pos = [4.87, -5.15, 0.058, -pi/2; 4.87, -5.15, 0.058+0.025, -pi/4; 4.87, -5.15, 0.058+0.05, -pi/4; ];

angle_off1= atan2(3,-8);
angle_off2= pi/2;
angle_off3= atan2(6,6);
target_pos_rot = [3,-8,0.06, -pi/2;                                           9, 0, 0.06, -pi/2.2;                                    9, 0, 0.06, -pi/2.2;6, 6, 0.06, -pi/2];
finish_pos_rot = [(3-0.25*sin(angle_off1)), (-8-0.25*cos(angle_off1)), 0.06, 0 ;(9-0.22*sin(angle_off2)), (0-0.22*cos(angle_off2)), 0.06, 0;    (9-0.22*sin(angle_off2)), (0-0.22*cos(angle_off2)), 0.06, 0;      (6-0.28*sin(angle_off3)), (6-0.28*cos(angle_off3)), 0.06, 0];

encoders_rot = simple_trajectory(target_pos_rot,finish_pos_rot);
move(2000,2000,2000,2000,2000);
for i=1:height(encoders_rot)
    move(encoders_rot(i,1),encoders_rot(i,2),encoders_rot(i,3),encoders_rot(i,4),encoders_rot(i,5));
end

move(2000,2000,2000,2000,2000);

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

pause(2);

% pause(0.2);

% pause(0.2);




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
    r_2 = r_ef-0.13*cos(phi);
    z_2 = z_ef-0.13*sin(phi);
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

% function joint_thetas = cubic_intermediate_joints(pos_start, pos_end, time, mouth)
%     theta1_start = pos_start(1);
%     theta2_start = pos_start(2);
%     theta3_start = pos_start(3);
%     theta4_start = pos_start(4);
%     theta1_end = pos_end(1);
%     theta2_end = pos_end(2);
%     theta3_end = pos_end(3);
%     theta4_end = pos_end(4);
%     t = 0:0.1:time;
%     theta1 = cubic_theta(t, theta1_start, theta1_end, time)';
%     theta2 = cubic_theta(t, theta2_start, theta2_end, time)';
%     theta3 = cubic_theta(t, theta3_start, theta3_end, time)';
%     theta4 = cubic_theta(t, theta4_start, theta4_end, time)';
%     joint_thetas = [rad_to_j1(theta1) rad_to_j2(theta2) rad_to_j3(theta3) rad_to_j4(theta4) mouth*ones(1,length(t))'];
% end

% function angle = cubic_theta(t, theta_start, theta_end, tf)
%     a0=theta_start;
%     a1=0;
%     a2=3*(theta_end-theta_start)/tf^2;
%     a3=-2*(theta_end-theta_start)/tf^3;
%     angle= a0+a1*t+a2*t.^2+a3*t.^3;
% end

% function trajec_encoders = cubic_trajectory(target_pos, finish_pos,time)
%     encoder_sequence = [];
%     intermediate_height=0.08; %height for block moving
%     mouth_open=2000;
%     mouth_close=2250;
%     if height(target_pos) ~= height(finish_pos) 
%         error('Input target/finish arrays must have the same length');
%     end
%     for i=1:height(target_pos)
%         xyz = coords_to_meters(target_pos(i,1),target_pos(i,2),intermediate_height);     
%         position = inverse_kinematic(xyz(1),xyz(2),xyz(3),target_pos(i,4));
%         position_kinematic1 = choose_kinematic(position);
%         encoder1_int = rad_to_j1(position_kinematic1(1));
%         encoder2_int = rad_to_j2(position_kinematic1(2));
%         encoder3_int = rad_to_j3(position_kinematic1(3));
%         encoder4_int = rad_to_j4(position_kinematic1(4));
%         encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_open];
%         xyz = coords_to_meters(target_pos(i,1),target_pos(i,2),target_pos(i,3));
%         position = inverse_kinematic(xyz(1),xyz(2),xyz(3),target_pos(i,4));
%         position_kinematic = choose_kinematic(position);
%         encoder1 = rad_to_j1(position_kinematic(1));
%         encoder2 = rad_to_j2(position_kinematic(2));
%         encoder3 = rad_to_j3(position_kinematic(3));
%         encoder4 = rad_to_j4(position_kinematic(4));
%         encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_open];
%         encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_close];
%         encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_close];       
%         xyz = coords_to_meters(finish_pos(i,1),finish_pos(i,2),intermediate_height);
%         position = inverse_kinematic(xyz(1),xyz(2),xyz(3),finish_pos(i,4));
%         position_kinematic2= choose_kinematic(position);
%         %cubic interpolation
%         interp = cubic_intermediate_joints(position_kinematic1,position_kinematic2,time,mouth_close);                
%         encoder_sequence = [encoder_sequence; interp];
%         encoder1_int = rad_to_j1(position_kinematic2(1));
%         encoder2_int = rad_to_j2(position_kinematic2(2));
%         encoder3_int = rad_to_j3(position_kinematic2(3));
%         encoder4_int = rad_to_j4(position_kinematic2(4));
%         encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_close];
%         xyz = coords_to_meters(finish_pos(i,1),finish_pos(i,2),finish_pos(i,3));
%         position = inverse_kinematic(xyz(1),xyz(2),xyz(3),finish_pos(i,4));
%         position_kinematic = choose_kinematic(position);
%         encoder1 = rad_to_j1(position_kinematic(1));
%         encoder2 = rad_to_j2(position_kinematic(2));
%         encoder3 = rad_to_j3(position_kinematic(3));
%         encoder4 = rad_to_j4(position_kinematic(4));
%         encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_close];
%         encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_open];
%         encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_open];
%         if (i<height(target_pos))
%             xyz_next=coords_to_meters(target_pos(i+1,1),target_pos(i+1,2),intermediate_height);
%             position_kinematic4=choose_kinematic(inverse_kinematic(xyz_next(1),xyz_next(2),xyz_next(3),target_pos(i+1,4)));
%             interp = cubic_intermediate_joints(position_kinematic2,position_kinematic4,time,mouth_close);                
%             encoder_sequence = [encoder_sequence; interp];
%         end
%     end
%     trajec_encoders = encoder_sequence;
% end

function trajec_encoders = simple_trajectory(target_pos, finish_pos)
    encoder_sequence = [];
    intermediate_height=0.084;
    mouth_open=1700;
    mouth_close=2250;

    if height(target_pos) ~= height(finish_pos) 
        error('Input target/finish arrays must have the same length');
    end
    for i=1:height(target_pos)
        xyz = coords_to_meters(target_pos(i,1),target_pos(i,2),intermediate_height);
        disp(xyz)
        position = inverse_kinematic(xyz(1),xyz(2),xyz(3),target_pos(i,4));
        position_kinematic = choose_kinematic(position);
        encoder1_int = rad_to_j1(position_kinematic(1));
        encoder2_int = rad_to_j2(position_kinematic(2));
        encoder3_int = rad_to_j3(position_kinematic(3));
        encoder4_int = rad_to_j4(position_kinematic(4));
        encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_open];
        xyz = coords_to_meters(target_pos(i,1),target_pos(i,2),target_pos(i,3));
        position = inverse_kinematic(xyz(1),xyz(2),xyz(3),target_pos(i,4));
        position_kinematic = choose_kinematic(position);
        encoder1 = rad_to_j1(position_kinematic(1));
        encoder2 = rad_to_j2(position_kinematic(2));
        encoder3 = rad_to_j3(position_kinematic(3));
        encoder4 = rad_to_j4(position_kinematic(4));
        encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_open];
        encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_close];
        encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_close];
        xyz = coords_to_meters(finish_pos(i,1),finish_pos(i,2),intermediate_height+0.040);
        position = inverse_kinematic(xyz(1),xyz(2),xyz(3),finish_pos(i,4));
        position_kinematic = choose_kinematic(position);
        encoder1_int = rad_to_j1(position_kinematic(1));
        encoder2_int = rad_to_j2(position_kinematic(2));
        encoder3_int = rad_to_j3(position_kinematic(3));
        encoder4_int = rad_to_j4(position_kinematic(4));
        encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_close];
        xyz = coords_to_meters(finish_pos(i,1),finish_pos(i,2),finish_pos(i,3));
        position = inverse_kinematic(xyz(1),xyz(2),xyz(3),finish_pos(i,4));
        position_kinematic = choose_kinematic(position);
        encoder1 = rad_to_j1(position_kinematic(1));
        encoder2 = rad_to_j2(position_kinematic(2));
        encoder3 = rad_to_j3(position_kinematic(3));
        encoder4 = rad_to_j4(position_kinematic(4));
        encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_close];
        encoder_sequence = [encoder_sequence; encoder1,encoder2,encoder3,encoder4,mouth_open];
        encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_open]; 
        encoder_sequence = [encoder_sequence; encoder1_int,encoder2_int,encoder3_int,encoder4_int,mouth_open];      

    end
    trajec_encoders = encoder_sequence;
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
