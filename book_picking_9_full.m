%% totally automated code
clear
clc
%% setting 
robot_validate_now
robot_op = Robot_operator(robot, Cconfig, tform, q, q_off);
commu = Communication('127.0.1.1',2016,'/dev/ttyUSB0');
%% update robot properties
[robot_op.m_T,robot_op.m_T_cam, robot_op.m_q] = robot_op.update_pose(q);
%% unlock when tcp 
[t,s] = commu.Open(1,1); % tcp, serial open
flush(s);

%% flag & sequence definition
robot_status_num

%% loop 
while(working)
    switch robot_op.m_status
    %% wait
        case wait %3
            if(book_shelve<=2) % maximum 2 books
                flush(t)
                disp("status = wait");
                data_uart = commu.receive_s(s)
                if(data_uart) % recieve_s returns false when packet is bad
                    data_uart = str2double(data_uart);
                    if(data_uart == send_depth || data_uart == approach) % start signal
                        if(restart) % repeating
                            robot_op.m_status = approach; 
                            commu.send_t(t,"/5/");
                        else
                            robot_op.m_status = send_depth;
                            commu.send_t(t,"/4/");
                        end
                    end 
                end
                clear data_uart
            else % more than 2 books
                flush(t)
                disp("status = wait2");
                data_uart = commu.receive_s(s)
                if(data_uart) % recieve_s returns false when packet is bad
                    data_uart = str2double(data_uart);
                    if(data_uart == send_depth || data_uart == approach) % start signal
                        robot_op.m_status = end_mat;
                    end 
                end
                clear data_uart
            end
    %% send_depth(python -> matlab -> arduino -> matlab)
        case send_depth % 4
            disp("status = send_depth");
            depth = commu.receive_t(t);
            depth = round(str2double(depth));
            commu.send_s_depth(s,depth);
            data_uart = commu.receive_s(s);
            pause(1);
            if(data_uart) % recieve_s returns false when packet is bad
                data_uart = str2double(data_uart);
                if(data_uart == approach)
                    robot_op.m_status = approach;
                end
            end
            clear data_uart

    %% approach
        case approach % 5
            disp("status = approach");
            pause(1.5);
                commu.send_t(t,"/5/");
            disp("status = approach");
            [robot_op,data_appr0] = run_approach0(robot_op,commu,150,1000,s,t,test,visual); %containing the communcation thing
            pause(0.3);
            if(data_appr0 == "2")
                restart = true;
            end
            if(data_appr0~="b"&& data_appr0~="r"&&data_appr0~="l"&&data_appr0~="2")
                robot_op.m_status = readable;
            end
            clear data_appr0

    %% readable
        case readable % 6
            disp("status = readable");
            commu.send_t(t,"/6/"); % reading book
            data_t = commu.receive_t(t);
            if(data_t == "6.5") % found the target book
                disp("status = found the label");
                robot_op.m_status = appr2book;
                disp('6.5 and sent');
            elseif(data_t == "u" || data_t == "d")
                disp("in the u and d");
                robot_op = run_not_read(robot_op,commu,data_t,50,1000,s,t, visual);
                pause(0.2);
                commu.send_t(t,"/6/"); % check if it is readable
            end
            clear data_t

    %% appr2book
        case appr2book % 8
            disp('status = appr2book'); %reading book
            pause(1.5);
            commu.send_t(t,"/7/"); % read book
            flush(t)
            data_t = commu.receive_t(t); %start signal
            if(data_t == "s") % approaching target : read and pose estimation
                [robot_op,data_appr] = run_approach(robot_op,commu,120,1000,s,t,test, visual);
                disp('status = appr2book');
                pause(0.5);
                if(data_appr~="b"&& data_appr~="r"&&data_appr~="l"&&data_appr~="2")
                    robot_op.m_status = align;
                end
                clear data_appr
            elseif(data_t == "f") % fail to read book -> returning position
                disp("couldn't read ocr return wait")
                robot_op = run_return(robot_op,commu,100,s,visual); % must return pose before send NRead to ardu
                commu.send_s(s,1,deg2rad([-90,-90,0,0,0,0])); % after this
                robot_op.m_status = wait;
                restart = true; % neccessary to reset
            end
            clear data_t
    %% align
        case align % 10
            pause(0.5);
            disp('status = align');
            commu.send_t(t,"/10/"); %status_align
            robot_op = run_align(robot_op,commu,50 ,s,t, visual);
            disp('align complete');
            robot_op.m_status = insert;
    %% insert
        case insert % 11
            disp('status = insert');
            commu.send_t(t,"/11/"); % status_wait_insert
            robot_op = run_push(robot_op,commu,2000,1000,s,t,visual );
            robot_op.m_status = grip;
    %% grip
        case grip % 12
            disp('status = grip');
            commu.send_s(s,12,robot_op.m_q); % grip commend in arduino
            pause(1);
            robot_op.m_status = back;

    %% back
        case back % 13
            disp('status = back');
            robot_op = run_back(robot_op,commu,150,1000,s,t,visual);
            disp('back')
            robot_op.m_status = BS_back;
            
    %% BallScrew back 
        case BS_back % 24
            pause(0.5);
            commu.send_s(s,BS_back,deg2rad([-90,-90,0,0,0,0]));
            data_uart = commu.receive_s(s);
            if(data_uart == "s")
                robot_op.m_status = orig_P;
            end
            clear data_uart
            
    %% orig_P
        case orig_P % 14
            pause(0.5);
            robot_op = run_orig_P(robot_op,commu,150,s,visual);
            if(book_shelve == 1) % first shelve is available
                robot_op.m_status = pick_place1_1;
            elseif(book_shelve == 2) % second shelve is available
                robot_op.m_status = pick_place2_1;                
            end

%======================arrangement1===========================

    %% book shelve
            %% put down book to align == 1 
        case pick_place1_1 % 28
            pause(0.5)
            disp('status = pick_place1_1')
            robot_op = run_place1_1(robot_op,commu,100,s,visual);
            robot_op.m_status = pick_place1_m;
    
    %% go grap upper side == 1
        case pick_place1_m % 29
            pause(0.5);
            disp('status = pick_place1_m')
            commu.send_s(s,grip_open,deg2rad([-90,-90,0,0,0,0]));
            pause(1);
            robot_op = run_place1_m(robot_op,commu,100,1000,s,visual);
            robot_op.m_status = pick_place1_f;
    
    %% approach the book to grap == 1
        case pick_place1_f % 30
            disp('status = pick_place1_f')
            pause(0.5);
            robot_op = run_place1_f(robot_op,commu,100,1000,s,visual);
            commu.send_s(s,grip_close,deg2rad([-90,-90,0,0,0,0]));
            pause(1);
            robot_op.m_status = pick_place1_2;
    
    %% lift book == 1
        case pick_place1_2 % 31
            pause(0.5)
            disp('status = pick_place1_2')
            robot_op = run_place1_2(robot_op,commu,100,1000,s,visual);
            robot_op.m_status = shelve1_1;
    
    %% approach the shelve == 1
        case shelve1_1 % 16
            disp('status = shelve1_1');
            pause(0.5);
            robot_op = run_shelve1_1(robot_op,commu,100,s,visual);
            robot_op.m_status = shelve1_2;
    
    %% put book on the shelve1 == 1
        case shelve1_2 % 17
            pause(0.5);
            disp('status = shelve1_2');
            robot_op = run_shelve1_2(robot_op,commu,100,1000,s,visual);
            disp("bookshelf1")
            robot_op.m_status = shelve1_2f;
    
    %% release it == 1
        case shelve1_2f % 18
            disp("release gripper");
            pause(0.5)
            commu.send_s(s,grip_open,deg2rad([-90,-90,0,0,0,0]));
            pause(1);
            robot_op.m_status = shelve1_3;
    
    %% raise up == 1
        case shelve1_3 % 40
            disp("status = put up");
            pause(0.4);
            robot_op = run_shelve1_3(robot_op,commu,150,1000,s,visual);
            robot_op.m_status = m_return;

%======================arrangement2===========================
%% put down book to align == 2
        case pick_place2_1 % 32
            pause(0.5)
            disp('status = pick_place1_1')
            robot_op = run_place2_1(robot_op,commu,100,s,visual);
            robot_op.m_status = pick_place2_m;

    %% go grap upper side == 2
        case pick_place2_m % 33
            pause(0.5);
            disp('status = pick_place1_m')
            commu.send_s(s,grip_open,deg2rad([-90,-90,0,0,0,0]));
            pause(1);
            robot_op = run_place2_m(robot_op,commu,200,1000,s,visual);
            robot_op.m_status = pick_place2_f;

    %% approach the book to grap == 2
        case pick_place2_f % 34
            disp('status = pick_place1_f')
            pause(0.5);
            robot_op = run_place2_f(robot_op,commu,100,1000,s,visual);
            commu.send_s(s,grip_close,deg2rad([-90,-90,0,0,0,0]));
            pause(1);
            robot_op.m_status = pick_place2_2;

    %% lift book == 2
        case pick_place2_2 % 35
            pause(0.5)
            disp('status = pick_place1_2')
            robot_op = run_place2_2(robot_op,commu,100,1000,s,visual);
            robot_op.m_status = shelve2_1;

    %% approach the shelve == 2
        case shelve2_1 % 19
            disp('status = shelve1_1');
            pause(0.5);
            robot_op = run_shelve2_1(robot_op,commu,100,s,visual);
            robot_op.m_status = shelve2_2;

    %% put book in the shelve2 == 2
        case shelve2_2 % 21
            pause(0.5);
            disp('status = shelve1_2');
            robot_op = run_shelve2_2(robot_op,commu,150,1000,s,visual);
            disp("bookshelf1")
            robot_op.m_status = shelve2_2f;

    %% release it == 2
        case shelve2_2f % 20
            disp("release gripper");
            pause(0.5)
            commu.send_s(s,grip_open,deg2rad([-90,-90,0,0,0,0]));
            pause(1);
            robot_op.m_status = shelve2_3;

    %% raise up == 2
        case shelve2_3 % 22
            disp("status = put up");
            pause(0.4);
            robot_op = run_shelve2_3(robot_op,commu,150,1000,s,visual);
            robot_op.m_status = m_return;
%===============================================================================
    %% m_return
        case m_return
            if(book_shelve == 1)
                book_shelve = book_shelve + 1;
            elseif(book_shelve == 2)
                book_shelve = book_shelve + 1;
                disp("book case are not available")
            end
            disp('status = return');
            robot_op = run_return(robot_op,commu,120,s,visual);
            robot_op.m_status = end_mat;

    %% end_mat
        case end_mat
            disp('status = end_mat');
            commu.send_t(t,"/2/");
            commu.send_s(s,end_mat,deg2rad([-90,-90,0,0,0,0])); % if arduino got end_mat -> last_signal = 2
            robot_op.m_status = wait;
            restart = false;

    %% otherwise
        otherwise
            disp("end_already")
    end
end

