test_robot2
% 
s = serialport('/dev/ttyUSB0',115200);
fopen(s);
t = tcpclient('127.0.1.1', 2016, "Timeout",20); % 동방
set(t, 'InputBufferSize', 100000); 
fopen(t); 

% t= 0;
count = 0;
% s = 1;
trajectory_index = 0;
mode = 1;

joint_angle = [th1,th2,th3,th4,th5,th6];

while t.BytesAvailable == 0 end
while mode 
%         % orientation 맞춘 상태
        trajectory_index = 0  
        [curr,joint_angle, th6_ang ,ttform] = simulate_test3(robot,Cconfig,Fk_0e,Fk_0_cam,0,0,50,1000,joint_angle,s,t,trajectory_index);
        ttform_t = ttform;
        pause(1);

        init_pose
        trajectory_index = 1
        [curr,joint_angle, th6_ang, ttform_t] = simulate_test3(robot,Cconfig, curr, Fk_0_cam,ttform_t,th6_ang,30,1000,joint_angle,s,t,trajectory_index);
        ttform_t;
        pause(1);

        init_pose
        trajectory_index = 2
        [curr,joint_angle,th6_ang, ttform_t] = simulate_test3(robot,Cconfig,curr, Fk_0_cam,ttform_t,th6_ang,50,1000,joint_angle,s,t,trajectory_index);
        ttform_t;
        init_pose;
        pause(1);

        trajectory_index = 3
        [curr,joint_angle,th6_ang, ttform_t] = simulate_test3(robot,Cconfig,curr, Fk_0_cam,ttform_t,th6_ang,100,1000,joint_angle,s,t,trajectory_index);
        ttform_t;
        init_pose;
        
% 
        trajectory_index = 4
        th1 = -deg2rad(90);th2 = -deg2rad(90);th3= 0;th4= 0;th5= 0;th6= 0;
        joint_angle = [th1,th2,th3,th4,th5,th6];
        [a, b] = Fk_chanmin_s2(th1,th2,th3,th4,th5,th6);  
        curr(1:3,1:3) = a;
        curr(:,4) = b;
        c = "["+rad2deg(joint_angle(1))+","+rad2deg(joint_angle(2))+","+rad2deg(joint_angle(3))+","+rad2deg(joint_angle(4))+","+rad2deg(joint_angle(5))+","+rad2deg(joint_angle(6))+"]"
        fprintf(s,'%s\n',c);

%         ttform_t = [0.0000 ,   0.0000 ,   1.0000, 2;   -1.0000    0.0000         0, 0;   -0.0000 ,  -1.0000,    0.0000,2;0,0,0,1;]
% %         ttform_t(1:3,4) = [0;-0.3;3]
%         [curr, joint_angle, th6_ang, ttform_t] = simulate_test3(robot,Cconfig,curr, Fk_0_cam,ttform_t,th6_ang,100,1000,joint_angle,s,t,trajectory_index);
%         curr
%         joint_angle
%         ttform_t;
%         init_pose
%         curr = Fk_0e
%         th6_ang = 0

%         trajectory_index = 4
%         th1 = -90;
%         joint_angle(1) = th1
%         [a,b] = Fk_chanmin_s2(joint_angle(1),joint_angle(2),joint_angle(3),joint_angle(4),joint_angle(5),joint_angle(6))
%         curr = eye(4)
%         curr(1:3,1:3) = a;
%         curr(:,4) = p;
%         
%         Cconfig(1).JointPosition = joint_angle(1);
%         Cconfig(2).JointPosition = joint_angle(2);
%         Cconfig(3).JointPosition = joint_angle(3);
%         Cconfig(4).JointPosition = joint_angle(4);
%         Cconfig(5).JointPosition = joint_angle(5);
%         Cconfig(6).JointPosition = joint_angle(6);
%         c = "["+rad2deg(joint_angle(1))+","+rad2deg(joint_angle(2))+","+rad2deg(joint_angle(3))+","+rad2deg(joint_angle(4))+","+rad2deg(joint_angle(5))+","+rad2deg(joint_angle(6))+"]"
%         fprintf(s,'%s\n',c);
          pause(1)
          th1 = 0;
          curr = eye(4);
          joint_angle(1) = th1;
          c = "["+rad2deg(joint_angle(1))+","+rad2deg(joint_angle(2))+","+rad2deg(joint_angle(3))+","+rad2deg(joint_angle(4))+","+rad2deg(joint_angle(5))+","+rad2deg(joint_angle(6))+"]";
          fprintf(s,'%s\n',c);
        [a, b] = Fk_chanmin_s2(th1,th2,th3,th4,th5,th6);  
        curr(1:3,1:3) = a;
        curr(:,4) = b;

        th1 = 0; th2 = -deg2rad(78); th3 = deg2rad(36.13); th4 =0; th5 =0; th6=0;
        curr = eye(4);
        joint_angle = [th1,th2,th3,th4,th5,th6]
        
        c = "["+rad2deg(joint_angle(1))+","+rad2deg(joint_angle(2))+","+rad2deg(joint_angle(3))+","+rad2deg(joint_angle(4))+","+rad2deg(joint_angle(5))+","+rad2deg(joint_angle(6))+"]";
        fprintf(s,'%s\n',c);
        [a, b] = Fk_chanmin_s2(th1,th2,th3,th4,th5,th6);  
        curr(1:3,1:3) = a;
        curr(:,4) = b;


%         th2 = -90;
%         th3 = 0;
%         th4 = 0;
%         th5 = 0;
%         th6 = 0;
%         [curr, joint_angle, th6_ang, ttform_t] = simulate_test3(robot,Cconfig,curr, Fk_0_cam,ttform_t,th6_ang,100,1000,joint_angle,s,t,trajectory_index);

%         curr
%         joint_angle
%         ttform_t;
%         init_pose
%         curr = Fk_0e
%         th6_ang = 0
        
        trajectory_index = 5
        ttform_t = [0,-1,0,1.5;-1,0,0,0;0,0,-1,0.8;0,0,0,1;]
        [curr,joint_angle,th6_ang, ttform_t] = simulate_test3(robot,Cconfig,curr, Fk_0_cam,ttform_t,th6_ang,100,1000,joint_angle,s,t,trajectory_index);
        curr
        trajectory_index = 6
        ttform_t = [0,-1,0,2;-1,0,0,0;0,0,-1,0.8;0,0,0,1;];
        [curr,joint_angle,th6_ang, ttform_t] = simulate_test3(robot,Cconfig,curr, Fk_0_cam,ttform_t,th6_ang,30,1000,joint_angle,s,t,trajectory_index);

    while t.BytesAvailable == 0 end
    while t.BytesAvailable ~= 0
        data = fread(t, t.BytesAvailable);
        str_data = char(data)';
    end
    
    if(str_data == 'E') % return and initialize
        theta_F= [-90,-90,0,0,0,0];
        for(i = 1: 1 : 50)
            theta_out = j_space_tr(theta_F,rad2deg(joint_angle),50,0,i)
            Fk_0e = init_pose
            Fk_0_cam
            
%             total_ttform = init_pose;
            c = "["+(theta_out(1))+","+(theta_out(2))+","+theta_out(3)+","+theta_out(4)+","+theta_out(5)+","+theta_out(6)+"]"
            fprintf(s,'%s\n',c);
            
        end
        th1 = -deg2rad(90);
        th2 = -deg2rad(90)
        th3= -deg2rad(0);
        th4 = -deg2rad(0);
        th5 = -deg2rad(0);
        th6 = -deg2rad(0);
        Cconfig(1).JointPosition = th1;
        Cconfig(2).JointPosition = th2;
        Cconfig(3).JointPosition = th3;
        Cconfig(4).JointPosition = th4;
        Cconfig(5).JointPosition = th5;
        Cconfig(6).JointPosition = th6;
        Fk_0e = init_pose;
        Fk_0_cam = init_pose_cam
        joint_angle = [th1,th2,th3,th4,th5,th6]
%         joint_ang= deg2rad([theta_out(1),theta_out(2),theta_out(3),theta_out(4),theta_out(5),theta_out(6)]);
        disp('restart')
    end
    



%         카메라 실시간 피드백 받고 앞으로 가는 코드
    %     trajectory_index = 2
    %     [ang1,ang2,ang3,ang4,ang5,ang6,curr,joint_angle,th6_ang, ttform_t] = simulate_test2(robot,Cconfig, curr, Fk_0_cam,ttform_t,th6_ang,100,500,joint_angle,s,t,trajectory_index);
        
    %     %return
    %     ttform_return = Fk_0e;
    %     trajectory_index = 2
    %     [ang1,ang2,ang3,ang4,ang5,ang6,curr,joint_angle,ttform_t] = simulate_test(robot,Cconfig,curr, Fk_0_cam,ttform_t,30,1000,joint_angle,s,t,trajectory_index);
%     mode = 0
end