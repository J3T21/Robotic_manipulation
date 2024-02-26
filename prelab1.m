% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


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

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                      = 11;            % Dynamixel ID: 1
DXL_ID_2                      = 12;            % Dynamixel ID: 2
DXL_ID_3                     = 13;            % Dynamixel ID: 3
DXL_ID_4                      = 14;            % Dynamixel ID: 4
DXL_ID_5                      = 15;            % Dynamixel ID: 5
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM4';       % Check which port is being used on your controller
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


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


% enter move function here depending on the position of the dynamixel
move(port_num,)




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




function move(port_num,encoder1,encoder2,encoder3,encoder4,encoder5)
    ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_POSITION       = 116; 
    ADDR_PRO_PRESENT_POSITION    = 132; 
    ADDR_PRO_OPERATING_MODE      = 11;
    PROTOCOL_VERSION            = 2.0; 

    DXL_ID                      = 11;            % Dynamixel ID: 1
    DXL_ID_2                      = 12;            % Dynamixel ID: 2
    DXL_ID_3                     = 13;            % Dynamixel ID: 3
    DXL_ID_4                      = 14;            % Dynamixel ID: 4
    DXL_ID_5                      = 15;   
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 1);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, 1);
   
    
    % Write goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(encoder1, 'uint32'));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, typecast(encoder2, 'uint32'));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, typecast(encoder3, 'uint32'));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, typecast(encoder4, 'uint32'));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, typecast(encoder5, 'uint32'));


    while 1
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position_3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position_4 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position_5 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_PRESENT_POSITION);

        if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
        elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
            printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
        end

        fprintf('[ID:%03d] GoalPos: %03d  PresPos:%03d\n', DXL_ID, dxl_goal_position(index), typecast(uint32(dxl_present_position), 'int32'));
        fprintf('[ID:%03d] GoalPos 2: %03d  PresPos:%03d\n', DXL_ID_2, dxl_goal_position(index), typecast(uint32(dxl_present_position_2), 'int32'));
        fprintf('[ID:%03d] GoalPos 3: %03d  PresPos:%03d\n', DXL_ID_3, dxl_goal_position(index), typecast(uint32(dxl_present_position_3), 'int32'));
        fprintf('[ID:%03d] GoalPos 4: %03d  PresPos:%03d\n', DXL_ID_4, dxl_goal_position(index), typecast(uint32(dxl_present_position_4), 'int32'));
        fprintf('[ID:%03d] GoalPos 5: %03d  PresPos:%03d\n', DXL_ID_5, dxl_goal_position(index), typecast(uint32(dxl_present_position_5), 'int32'));

        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position_2), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position_3), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position_4), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position_5), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end
end

function task2()
    

    num_steps = 800;
    

    frequency = 20;  % 1 Hz for example
    

    t = linspace(0, 1, num_steps);  % Assuming one second duration
    

    amplitude = 500;  
    sin_wave = amplitude * sin(2 * pi * frequency * t)+2046;
    disp(sin_wave)
    for i=1:400

        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(sin_wave(i)), 'int32'));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, typecast(int32(sin_wave(i+100)), 'int32'));

        while 1
            % Read present position
            dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
            dxl_present_position_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
                printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
            elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
                printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
            end

            fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL_ID, sin_wave(i), typecast(uint32(dxl_present_position), 'int32'));
            fprintf('[ID:%03d] GoalPos 2:%03d  PresPos:%03d\n', DXL_ID_2, sin_wave(i), typecast(uint32(dxl_present_position_2), 'int32'));

            if ~(abs(sin_wave(i) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
                break;
            end
            if ~(abs(sin_wave(i) - typecast(uint32(dxl_present_position_2), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
                break;
            end
        end
    end
end




