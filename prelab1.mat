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
BAUDRATE                    = 115200;
DEVICENAME                  = '/dev/tty.usbserial-FT5NSOX1';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 600;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 3400;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
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
MAX_POS = 3400;
MIN_POS = 600;
% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MAX_POS, MAX_POS);
% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MIN_POS, MIN_POS);
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

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_OPERATING_MODE, 3);


% Disable Dynamixel Torque
 % write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
 % write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 0);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end
% enter function here measure / task1 /task2
t0 = [-1 0 0; 0 -1 0; 0 0 1];
t1 = t1m(5);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 0);

i = 0;
        j = 0;
    while (j<200)
        j = j+1;

        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);

        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end
        big_angle = 360*(typecast(uint32(dxl_present_position), 'int32'))/4096;
        small =360*(typecast(uint32(dxl_present_position_2), 'int32'))/4096;
        fprintf('[ID:%03d] Position: %03d\n', DXL_ID, 360*(typecast(uint32(dxl_present_position), 'int32'))/4096);

        fprintf('[ID:%03d] Position 2: %03d\n', DXL_ID, 360*(typecast(uint32(dxl_present_position_2), 'int32'))/4096);
        fprintf('[ID:%03d] Position of stick: %03d\n',t0*t1(big_angle)*t2(big_angle)*t3(small)*t4(small));

        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end



% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);


close all;
clear all;
function t=t1(theta)
    t =[cos(theta) -sin(theta) 0 ; sin(theta) cos(theta) 0 ; 0 0 1];  
end
function t=t2_t3(theta)
    t =[cos(theta) -sin(theta) 0 ; sin(theta) cos(theta) 80 ; 0 0 1];  
end

function t=t4(theta)
    t =[cos(theta) -sin(theta) 0 ; sin(theta) cos(theta) 160 ; 0 0 1];  
end
function measure()

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 0);

    i = 0;
            j = 0;
        while (j<200)
            j = j+1;
    
            % Read present position
            dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
            dxl_present_position_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_PRESENT_POSITION);
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    
            dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
            end
    
            fprintf('[ID:%03d] Position: %03d\n', DXL_ID, 360*(typecast(uint32(dxl_present_position), 'int32'))/4096);
    
            fprintf('[ID:%03d] Position 2: %03d\n', DXL_ID, 360*(typecast(uint32(dxl_present_position_2), 'int32'))/4096);
    
            if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
                break;
            end
        end
end
function move()
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    while 1
    
    
        % Write goal position
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_goal_position(index)), 'uint32'));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_goal_position(index)), 'uint32'));
    
    
        while 1
            % Read present position
            dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
            dxl_present_position_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
                printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
            elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
                printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
            end
    
            fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL_ID, dxl_goal_position(index), typecast(uint32(dxl_present_position), 'int32'));
            fprintf('[ID:%03d] GoalPos 2:%03d  PresPos:%03d\n', DXL_ID_2, dxl_goal_position(index), typecast(uint32(dxl_present_position_2), 'int32'));
    
            if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
                break;
            end
            if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position_2), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
                break;
            end
        end
    
        % Change goal position
        if index == 1
            index = 2;
        else
            index = 1;
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

