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
DEVICENAME                  = '/dev/cu.usbserial-FT5WJ7EO';       % Check which port is being used on your controller
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
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_OPERATING_MODE, 3);
% limit speed so it doesnt spaz
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_VELOCITY, 700);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_VELOCITY, 700);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_VELOCITY, 700);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_VELOCITY, 700);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_VELOCITY, 700);
% enable tourque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, 1);
% TODO: Make the things above into a function
% move function
position1 = inverse_kinematic(0.075,-0.2,0.025,-pi/2);
position1_kinematic = choose_kinematic(position1);
encoder1 = rad_to_j1(position1_kinematic(1));
encoder2 = rad_to_j2(position1_kinematic(2));
encoder3 = rad_to_j3(position1_kinematic(3));
encoder4 = rad_to_j4(position1_kinematic(4));


move(encoder1,encoder2,encoder3,encoder4,2000);
move(encoder1,encoder2,encoder3,encoder4,3000);

position2 = inverse_kinematic(0.125,-0.125,0.025,-pi/2);
position2_kinematic = choose_kinematic(position2);
encoder1 = rad_to_j1(position2_kinematic(1));
encoder2 = rad_to_j2(position2_kinematic(2));
encoder3 = rad_to_j3(position2_kinematic(3));
encoder4 = rad_to_j4(position2_kinematic(4));
move(encoder1,encoder2,encoder3,encoder4,3000);
move(encoder1,encoder2,encoder3,encoder4,2000);


%%% xy-plane

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, 0);
% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);


close all;
clear all;

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
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder5);
    pause(2);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, encoder1);
    pause(2);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, encoder4);
    pause(2);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, encoder3);
    pause(2);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, encoder2);
    pause(2);
    
        

end

function IK_final = choose_kinematic(joint_angles)
    j1=rad_to_j1(joint_angles(1,1));
    j2=rad_to_j2(joint_angles(1,2)); 
    j3=rad_to_j3(joint_angles(1,3));
    j4=rad_to_j4(joint_angles(1,4));
    if (j2 < 760) || (j2 > 3290) || (j3 < 695) || (j3 > 3060) || (j4 < 820) || (j4 > 3450) || any(isnan(joint_angles(1,:)))
        %warning('IK using positive cosine component out of range')
        j1=rad_to_j1(joint_angles(2,1));
        j2=rad_to_j2(joint_angles(2,2)); 
        j3=rad_to_j3(joint_angles(2,3));
        j4=rad_to_j4(joint_angles(2,4));
        if (j2 < 760) || (j2 > 3290) || (j3 < 695) || (j3 > 3060) || (j4 < 820) || (j4 > 3450) || any(isnan(joint_angles(2,:)))
            error('Joint angles are out of range for both IK solvers');
        else
            IK_final = joint_angles(2,:);
        end
    else
        IK_final = joint_angles(1,:);
    end
end

function IK = inverse_kinematic(x_ef,y_ef,z_ef,phi)
    theta_o = asin(0.024/0.130);
    z_ef=z_ef-0.077;
    theta1 = atan2(y_ef,x_ef);
    r_ef = sqrt(x_ef^2+y_ef^2);
    r_2 = r_ef-0.126*cos(phi);
    z_2 = z_ef-0.126*sin(phi);
    theta3_plus = acos((r_2^2+z_2^2-0.13^2-0.124^2)/(2*0.13*0.124))
    theta3_minus = -acos((r_2^2+z_2^2-0.13^2-0.124^2)/(2*0.13*0.124))
    costheta2_plus = (r_2*(0.13+0.124*cos(theta3_plus))+z_2*0.124*sin(theta3_plus))/(r_2^2+z_2^2);
    costheta2_minus = (r_2*(0.13+0.124*cos(theta3_minus))+z_2*0.124*sin(theta3_minus))/(r_2^2+z_2^2);
    sintheta2_plus = -sqrt(1-costheta2_plus^2)
    sintheta2_minus = sqrt(1-costheta2_minus^2)
    theta2_plus = atan2(sintheta2_plus,costheta2_plus)
    theta2_minus = atan2(sintheta2_minus,costheta2_minus)
    theta4_plus = phi-theta2_plus-theta3_plus
    theta4_minus = phi-theta2_minus-theta3_minus
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