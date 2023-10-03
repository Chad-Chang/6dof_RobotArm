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
            robot_op = run_place1_m(robot_op,commu,200,1000,s,visual);
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
            robot_op = run_shelve1_2(robot_op,commu,150,1000,s,visual);
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