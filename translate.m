mary_had = 'AGFG|AAA2|GGG2|AAA2|AGFG|AAAA|GGAG|F4|]';


abc_to_instr(mary_had);


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

