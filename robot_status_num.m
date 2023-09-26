read_fail = 1; end_mat = 2;
wait = 3; send_depth = 4; approach = 5; readable = 6; % task state
appr2book = 8; tracking = 9; align = 10; insert = 11; grip = 12; back = 13; orig_P = 14;
turn_th1 = 15; 
shelve1_1 = 16; shelve1_2f = 17;  shelve1_2 = 18;
shelve2_1 = 19; shelve2_f = 20;  shelve2_2 = 21;
shelve1_3 = 40;

succeed = 22; % task true or false
wait_serial = 22; m_return = 23; BS_back = 24; gripOpen = 25; 
reset = 50;

%% first_shelve
pick_place1_1 = 28; %1_1 rectifying position
pick_place1_m = 29; %1_1 moving position + grip
pick_place1_f = 30
pick_place1_2 = 31; %1_2 pick up

grip_open = 100;
grip_close = 101;


%% second_shelve
pick_place2_1 = 31; 
pick_place2_m = 32; %1_1 moving position + grip
pick_place2_2 = 33;


restart = false; 