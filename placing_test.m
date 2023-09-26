%% totally automated code
clear
clc
%% setting 
robot_validate_now
q_off = [270.7;178.43;180.11;181.37;179.3;270;197.67;]; %7x1
%q : 6x1
robot_op = Robot_operator(robot, Cconfig, tform, q, q_off);
commu = Communication('127.0.1.1',2013,'/dev/ttyUSB0');
%% update robot properties
[robot_op.m_T,robot_op.m_T_cam, robot_op.m_q] = robot_op.update_pose(q);

%% unlock when tcp 
[t,s] = commu.Open(0,1); % tcp, serial open
flush(s);
visual = false;
robot_status_num % robot status definition
robot_op.m_status = wait;

%% loop 
working = true;
while(working)
    switch robot_op.m_status
    %% wait
        case wait
            disp("status = wait") % 3
            data_uart = commu.receive_s(s)
            data_uart = str2double(data_uart);
            if(data_uart == orig_P) % 14
                robot_op.m_status = orig_P; 
            elseif(data_uart == 1) %restart and 
                robot_op = run_return(robot_op,commu,150,s,visual);
                disp("return")
            end 
            clear data_uart
%%
        case orig_P % 14
            pause(0.5)
            disp('status = original');
            robot_op = run_orig_P(robot_op,commu,150,s,visual);
            robot_op.m_status = pick_place1_1;
%% put down book to align
        case pick_place1_1 % 28
            pause(0.5)
            disp('status = pick_place1_1')
            robot_op = run_place1_1(robot_op,commu,100,s,visual);
            robot_op.m_status = pick_place1_m;
%% go grap upper side
        case pick_place1_m % 29
            pause(0.5);
            disp('status = pick_place1_m')
            commu.send_s(s,grip_open,deg2rad([-90,-90,0,0,0,0]));
            pause(1);
            robot_op = run_place1_m(robot_op,commu,200,1000,s,visual);
%             robot_op.m_status = end_mat
            robot_op.m_status = pick_place1_f;
%% approach the book to grap
        case pick_place1_f
            disp('status = pick_place1_f')
            pause(0.5);
            robot_op = run_place1_f(robot_op,commu,100,1000,s,visual);
            commu.send_s(s,grip_close,deg2rad([-90,-90,0,0,0,0]));
            pause(1);
%             robot_op.m_status = end_mat
            robot_op.m_status = pick_place1_2;
%% lift book
        case pick_place1_2 % 30
            pause(0.5)
            disp('status = pick_place1_2')
            robot_op = run_place1_2(robot_op,commu,100,1000,s,visual);
            robot_op.m_status = shelve1_1;
%% approach the shelve
        case shelve1_1 
            disp('status = shelve1_1');
            pause(0.5);
            robot_op = run_shelve1_1(robot_op,commu,100,s,visual);
            robot_op.m_status = shelve1_2;
%% put book on the shelve1
        case shelve1_2
            pause(0.5);
            disp('status = shelve1_2');
            robot_op = run_shelve1_2(robot_op,commu,150,1000,s,visual);
            disp("bookshelf1")
            robot_op.m_status = shelve1_2f;
%% release it
        case shelve1_2f
            disp("release gripper");
            pause(0.5)
            commu.send_s(s,100,[0,0,0,0,0,0]);
            pause(1);
            robot_op.m_status = shelve1_3;
%% put up
        case shelve1_3
            disp("status = put up");
            pause(0.4);
            robot_op = run_shelve1_3(robot_op,commu,150,1000,s,visual);
            robot_op.m_status = end_mat;
%% end_matrix
        case end_mat
            disp("status = end")
            commu.send_s(s,end_mat,deg2rad([-90,-90,0,0,0,0]));
            robot_op.m_status = wait;
    %% otherwise
        otherwise
            disp("end_already")
            
    end
end

