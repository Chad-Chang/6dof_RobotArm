
%% Yuseuk signal
read_fail = 1; end_mat = 2;

%% general operating signal
wait = 3; send_depth = 4; approach = 5; readable = 6; % task state
appr2book = 8; tracking = 9; align = 10; insert = 11; grip = 12; back = 13; orig_P = 14;

m_return = 23; BS_back = 24; 
%% first_shelve
pick_place1_1 = 28; %1_1 rectifying position
pick_place1_m = 29; %1_1 moving position + grip
pick_place1_f = 30;
pick_place1_2 = 31; %1_2 pick up
shelve1_1 = 16; shelve1_2f = 17;  shelve1_2 = 18; shelve1_3 = 40;
%% gripper
grip_open = 100;
grip_close = 101;


%% second_shelve
pick_place2_1 = 32; 
pick_place2_m = 33; %1_1 moving position + grip
pick_place2_f = 34;
pick_place2_2 = 35;
shelve2_1 = 19; shelve2_2f = 20;  shelve2_2 = 21; shelve2_3 = 22;

%% flag
restart = false; visual = false; working = true; test = false;

%% bookshelve number
book_shelve = 3; % which book shelve is available

%% robot initial status
robot_op.m_status = wait;