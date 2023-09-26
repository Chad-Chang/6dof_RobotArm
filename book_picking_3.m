robot_validate_now
test = false;
q_off = [269;178.43;180.11;181.37;179.2;270;197.67;]; %7x1
%q : 6x1
robot_op = Robot_operator(robot, Cconfig, tform, q, q_off);
commu = Communication('127.0.1.1',2012,'/dev/ttyUSB0');
%% update robot properties

[robot_op.m_T,robot_op.m_T_cam, robot_op.m_q] = robot_op.update_pose(q);

%% unlock when tcp 
if(test)
    [t,s]=commu.Open(0,0); % tcp, serial open
    visual = true;
else
    [t,s] = commu.Open(1,1); % tcp, serial open
    flush(s);
    visual = false;
end

%NRead =1; end = 2; -> usual =0, not_read=1, end:

send_depth = 25;
wait = 3; move_read = 4; try_read = 5; % task state

move_targ = 6; tracking = 7; align = 8; insert = 9; grip = 10; back = 11; orig_P = 12;
turn_th1 = 13; shelve1 = 14; shelve2 = 15; fail = 20;succeed = 21;end_mat = 22; % task true or false
wait_serial = 23; m_return = 24;

if(test)
    robot_op.m_status = move_read;
else
    robot_op.m_status = wait;
end

%% loop 
working = true;
while(working)
    if(robot_op.m_status == wait) % wait untill serial signal comes
        flush(t)
        disp("status = wait");
        data_uart = commu.receive_s(s);
        data_uart = str2double(data_uart);
        if(data_uart == send_depth)
            commu.send_t(t,"/25/")
            robot_op.m_status = send_depth; 
        end
    elseif(robot_op.m_status == send_depth)
        depth = commu.receive_t(t);
        depth = round(str2double(depth))
        commu.send_s_depth(s,depth)

    elseif(robot_op.m_status == move_read) % move to read label
        if(~test)
            commu.send_t(t,"/1/");
        end
        [robot_op,data_appr0] = run_approach0(robot_op,commu,150,1000,s,t,test,visual); %containing the communcation thing
        pause(0.8);
        % when I test
%         working = false
        data_appr0
        if(data_appr0~="b")
            robot_op.m_status = try_read;
        end
        clear data_appr0

    elseif(robot_op.m_status == try_read) % move if it is not able to grab
        disp("status = try_read");
        commu.send_t(t,"/2/"); % reading book
        data_t = commu.receive_t(t);
        pause(0.3);%run_align
        if(data_t == "2.5") % found the target book
            disp("status = found the label");
            robot_op.m_status = move_targ;
            disp('3 and sent');
        elseif(data_t == "u" || data_t == "d")
            disp("in the u and d");
            robot_op = run_not_read(robot_op,commu,data_t,50,1000,s,t, visual);
            pause(0.2);
            commu.send_t(t,"/2/");
        end

    elseif(robot_op.m_status == move_targ) % after reading a book, go to the position of the target
        disp('status = move_targ');
        pause(3);
        commu.send_t(t,"/9/"); % read book
        data_t = commu.receive_t(t);%start signal
        data_t
        if(data_t == "3") % approaching target
            [robot_op,data_appr] = run_approach(robot_op,commu,120,1000,s,t,test, visual);
            disp('status = move_targ_approached');
            pause(0.5);
            data_appr
            if(data_appr ~='b')
                robot_op.m_status = align;
            end
            clear data_appr

%         elseif(data_t == "8") % fail to read book cause of some image problems
%             disp('status = not_read');
%             robot_op = run_not_read(robot_op,commu,data_t,50,1000,s,t, visual);
%             commu.send_t(t,"/2/");

        elseif(data_t == "1") % the book is not in the frame
            disp("couldn't read ocr return wait")
            robot_op = run_return(robot_op,commu,100,s,visual); % must return pose before send NRead to ardu
            commu.send_s(s,1,[0,0,0,0,0,0]'); % after this
            commu.send_t(t,"/0/"); % python wait status 
            robot_op.m_status = wait;
            %return initial pose
        end
        
%     elseif robot_op.m_status == 4 % after reading a book, go to the position of the target
% robot_op = run_approach(robot_op,commu,50,1000,s,t,false, visual);
%         robot_op = run_approach(robot_op,commu,50,1000,s,t);
%         disp('status 4 done ')
%         commu.send_t(t,"/5/");
%         robot_op.m_status = 5;

    elseif(robot_op.m_status == align) % align only th6..
        pause(1);
        disp('status = align');
        commu.send_t(t,"/5/");
%         data_t = commu.receive_t(t); % if it is neccessary to get angle several time
        robot_op = run_align(robot_op,commu,50 ,s,t, visual);
        pause(0.2);
        disp('align complete');
        robot_op.m_status = insert;

    elseif(robot_op.m_status == insert) % insert gripper
        disp('status = insert');
        commu.send_t(t,"/6/");
        robot_op = run_push(robot_op,commu,2000,1000,s,t,visual );
        pause(0.3);%run_align
        robot_op.m_status = wait_serial;

    elseif(robot_op.m_status == wait_serial) % decide whether proceed or repeat.
        disp('status = wait_serial');
        data_uart = commu.receive_s(s);
        data_uart = str2double(data_uart);
        pause(0.3);%run_align
        if(data_uart == grip) % 
            disp("wait_serial- grip")
            robot_op.m_status = grip;
        elseif(data_uart == wait) % return
            disp("wait_serial- return")
            robot_op = run_return(robot_op,commu,150,s,visual);
            robot_op.m_status = wait;
        end

    elseif(robot_op.m_status == grip) % check gripping Arduino -> matlab
        disp('status = grip');
        %stop the process untill signal received
        commu.send_s(s,23,robot_op.m_q); % grip commend in arduino
        data = commu.receive_s(s);
        if(data=='f')
            robot_op.m_status = fail;
        elseif(data=='s')
            pause(0.4);
            robot_op.m_status = back; % succeed
        end
        
    elseif(robot_op.m_status == back) % draw back the book
        disp('status = back');
        robot_op = run_back(robot_op,commu,100,1000,s,t,visual);
%         pause(0.3);
        disp('back')
        data_uart = commu.receive_s(s);
        data_uart = str2double(data_uart);
        if(data_uart == wait)
            robot_op = run_return(robot_op,commu,150,s,visual);
            robot_op.m_status = wait;
        end
        robot_op.m_status = orig_P;
    
    elseif(robot_op.m_status == orig_P) % return1 
        disp('status = orig_P');
        robot_op = run_return(robot_op,commu,100,s,visual);
        pause(0.1);
        disp('set')
        robot_op.m_status = turn_th1;

    elseif(robot_op.m_status == turn_th1) % only move th1
        disp('status = turn_th1');
        robot_op = run_th1(robot_op,commu,100,s,visual);
        pause(0.2);
        disp("th1 turn")
        robot_op.m_status = shelve2;

    elseif(robot_op.m_status == shelve1)
        disp('status = shelve1');
        robot_op = run_shelve1(robot_op,commu,100,1000,s,visual);
        pause(0.3);
        disp("bookshelf1")
        robot_op.m_status = shelve2;

    elseif robot_op.m_status == shelve2 % move to the bookshelf2
        disp('status = shelve2');
        robot_op = run_shelve2(robot_op,commu,100,1000,s,visual);
        pause(0.3);
        disp("bookshelf2")
        robot_op.m_status = m_return;
%     
    elseif(robot_op.m_status == m_return) % return after all done
        disp('status = return');
        pause(0.3);
        robot_op = run_return(robot_op,commu,150,s,visual);
        robot_op.m_status = end_mat;
        working = false;

%% need some revise : return status 2 position instead of returning original status => memorize the angle of the reading position or H
    elseif(robot_op.m_status == 'f') % gripping has been failed -> need some improvement
        disp('status = fail');
        robot_op = run_return(robot_op,commu,50,s,visual);
        disp('retry');
        robot_op.m_status = 1;
        commu.send_t(t,"/1/"); % read the book again

    elseif robot_op.m_status == end_mat
        disp('status = end_mat');
        commu.send_t(t,"/22/");
        commu.send_s(s,end_mat,[0,0,0,0,0,0]);
        commu.Close(t,s);
    end

end
