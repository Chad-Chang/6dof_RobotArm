%% totally automated code
clear
clc
%% setting 
robot_validate_now
test = false;
q_off = [270.7;178.43;180.11;181.37;179.3;270;197.67;]; %7x1
%q : 6x1
robot_op = Robot_operator(robot, Cconfig, tform, q, q_off);
commu = Communication('127.0.1.1',2016,'/dev/ttyUSB0');
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

read_fail = 1; end_mat = 2;
wait = 3; send_depth = 4; approach = 5; readable = 6; % task state
appr2book = 8; tracking = 9; align = 10; insert = 11; grip = 12; back = 13; orig_P = 14;
turn_th1 = 15; 
shelve1_1 = 16; shelve1_1f = 17;  shelve1_2 = 18;
shelve2_1 = 19; shelve2_1f = 20;  shelve2_2 = 21;
succeed = 2; % task true or false
wait_serial = 22; m_return = 23; BS_back = 24; gripOpen = 25; 
pick_place1_1 = 28; pick_place1_2 = 29; pick_place2_1 = 30; pick_place2_2 = 31;
reset = 50;
restart = false; is_shelve1 = false; is_shelve2 = false;

if(test)
    robot_op.m_status = approach;
else
    robot_op.m_status = wait;
end

%% loop 
working = true;
while(working)
    switch robot_op.m_status
    %% wait
        case wait
            flush(t)
            disp("status = wait");
            data_uart = commu.receive_s(s)
            if(data_uart) % recieve_s returns false when packet is bad
                data_uart = str2double(data_uart);
                if(data_uart == send_depth)
                    if(restart) % repeating
                        robot_op.m_status = approach; 
                        commu.send_t(t,"/5/")
                    else
                        robot_op.m_status = send_depth
                        commu.send_t(t,"/4/")
                    end
                end 
            end
            clear data_uart
            
    %% send_depth(python -> matlab -> arduino -> matlab)
        case send_depth
            disp("status = send_depth");
            depth = commu.receive_t(t);
            depth = round(str2double(depth))
            commu.send_s_depth(s,depth);
            data_uart = commu.receive_s(s)
            pause(1);
            if(data_uart) % recieve_s returns false when packet is bad
                data_uart = str2double(data_uart)
                if(data_uart == approach)
                    robot_op.m_status = approach;
                end
            end
            clear data_uart
    %% approach
        case approach
            disp("status = approach");
            pause(1.5);
            if(~test)
                commu.send_t(t,"/5/");
            end
            disp("status = approach");
            [robot_op,data_appr0] = run_approach0(robot_op,commu,150,1000,s,t,test,visual); %containing the communcation thing
            if(test)
                robot_op.m_status = appr2book;
            else
                pause(0.3);
                if(data_appr0 == "2")
                    restart = true;
                end
                if(data_appr0~="b"&& data_appr0~="r"&&data_appr0~="l"&&data_appr0~="2")
                    robot_op.m_status = readable;
                end
            end
            clear data_appr0
    %% readable
        case readable
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
        case appr2book
            disp('status = appr2book'); %reading book
            if(~test)
                pause(1.5);
                commu.send_t(t,"/7/"); % read book
                flush(t)
                data_t = commu.receive_t(t); %start signal
                data_t
                if(data_t == "s") % approaching target : read and pose estimation
                    [robot_op,data_appr] = run_approach(robot_op,commu,120,1000,s,t,test, visual);
                    disp('status = appr2book');
                    pause(0.5);
                    data_appr
                    if(data_appr~="b"&& data_appr~="r"&&data_appr~="l"&&data_appr~="2")
                        robot_op.m_status = align;
                    end
                    clear data_appr
                elseif(data_t == "f") % fail to read book -> returning position
                    disp("couldn't read ocr return wait")
                    robot_op = run_return(robot_op,commu,100,s,visual); % must return pose before send NRead to ardu
                    commu.send_s(s,1,[0,0,0,0,0,0]'); % after this
                    robot_op.m_status = wait;
                    restart = true; % neccessary to reset
                    %return initial pose
                end
            else
                [robot_op,data_appr] = run_approach(robot_op,commu,120,1000,s,t,test, visual);
                robot_op.m_status = align;
            end
            clear data_t
    %% align
        case align
            if(~test)
                pause(0.5);
                disp('status = align');
                commu.send_t(t,"/10/"); %status_align
                robot_op = run_align(robot_op,commu,50 ,s,t, visual);
                disp('align complete');
                robot_op.m_status = insert;
            else
                robot_op = run_align(robot_op,commu,50 ,s,t, visual);
                robot_op.m_status = insert;
            end

    %% insert
        case insert
            if(~test)
                disp('status = insert');
                commu.send_t(t,"/11/"); % status_wait_insert
                robot_op = run_push(robot_op,commu,2000,1000,s,t,visual );
                robot_op.m_status = grip;
            else
                robot_op = run_push(robot_op,commu,2000,1000,s,t,visual );
                robot_op.m_status = back;
            end
    %% grip
        case grip
            disp('status = grip');
            %stop the process untill signal received
            commu.send_s(s,12,robot_op.m_q); % grip commend in arduino
            pause(1.5);
            % makesure to return 9/30
%             robot_op.m_status = back;
            

%             pressur sensor
    %% back
        case back
            if(~test)
                disp('status = back');
                robot_op = run_back(robot_op,commu,100,1000,s,t,visual);
                disp('back')
                robot_op.m_status = BS_back;
            end
    %% BallScrew back
        case BS_back
            pause(0.5);
            commu.send_s(s,BS_back,[0,0,0,0,0,0]);
            data_uart = commu.receive_s(s);
            if(data_uart == "s")
                robot_op.m_status = orig_P;
            end
            clear data_uart
            
    %% orig_P
        case orig_P
            pause(0.5);
            disp('status = turn_th1');
            robot_op = run_orig_P(robot_op,commu,150,s,visual);
            disp("th1 turn")
%             robot_op.m_status = shelve1_1;
            robot_op.m_status = m_return;
            
% shelve1_1 = 16; shelve1_1f = 17;  shelve1_2 = 18;
%     %% shelve1
%         case shelve1_1
%             pause(1.5);
%             disp('status = shelve1');
%             robot_op = run_shelve1_1(robot_op,commu,150,s,visual);
%             disp("bookshelf1")
%             robot_op.m_status = shelve1_2;
%     %% shelve1_1f
%         case shelve1_1f
%             commu.send_s(s,shelve1_1f,[0,0,0,0,0,0]);
%             data_uart = commu.receive_s(s);
%             if(data_uart=="s")
%                 robot_op = run_shelve1_3(robot_op,commu,150,1000,s,t,test,visual);
%                 robot_op.m_status = m_return;
%             end
%             clear data_uart
%     %% shelve1_2
%         case shelve1_2
%             pause(1.5);
%             disp('status = shelve2');
%             robot_op = run_shelve1_2(robot_op,commu,150,1000,s,t,test,visual);
%             disp("bookshelf1")
%             robot_op.m_status = shelve1_1f;
    %% m_return
        case m_return
            disp('status = return');
            robot_op = run_return(robot_op,commu,150,s,visual);
            robot_op.m_status = end_mat;
    %% end_mat
        case end_mat
            disp('status = end_mat');
            commu.send_t(t,"/2/");
            commu.send_s(s,end_mat,[0,0,0,0,0,0]);
            disp("press 8 if you want to reset");
            data_uart = commu.receive_s(s);
            if(data_uart == "r")
                robot_op.m_status = reset;
            end
%             commu.Close(t,s);
    %% reset
        case reset
            restart = false; is_shelve1 = false; is_shelve2 = false; 
            robot_op.m_status = wait;
    %% otherwise
        otherwise
            disp("end_already")
        
    end
end

