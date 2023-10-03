send_depth = 25;
wait = 3; move_read = 4; try_read = 5; % task state
move_targ = 6; tracking = 7; align = 8; insert = 9; grip = 10; back = 11; orig_P = 12;
turn_th1 = 13; shelve1 = 14; shelve2 = 15; fail = 20;succeed = 21;end_mat = 22; % task true or false
wait_serial = 23; m_return = 24;

commu = Communication('127.0.1.1',2012,'/dev/ttyUSB0');
[t,s] = commu.Open(1,1); % tcp, serial open
flush(s);
status = wait;
working = true;
while(working)
    if(status == wait)
        flush(t)
        disp("status = wail");
        data_uart = commu.receive_s(s);
        data_uart = str2double(data_uart);
        if(data_uart == send_depth)
            commu.send_t(t,"/2.5/")
            status = send_depth;
        end
    elseif(status == send_depth)
        depth = commu.receive_t(t);
        depth = round(str2double(depth))
        commu.send_s_depth(s,depth)
    end
end
