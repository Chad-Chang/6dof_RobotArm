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
