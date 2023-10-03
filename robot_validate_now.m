robot = rigidBodyTree;
% its the basic model of robot which has no offset angles.

%%%%%%%%%%%%%%dh
%%%%%%%%%%%%%%parameter%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d0 = 0.381; d1 = 0.487;d2 = 0;d3= 0;d4 = 1.575;d5 = 0;d6 =1.524;
d_cam0 = 0.506; 
a1= 0;a2 = 1.0154;a3 = 0;a4 = 0;a5 = -0.218; %a6 = -0.07;
a6 = 0;
th1 = 0;th2 = 0;th3 = 0;th4 =0;th5 = 0; th6 = 0;
alp1= -pi/2;alp2= 0;alp3 = pi/2; alp4 =pi/2; alp5 =pi/2; alp6 =0;
th_off1 = 0;th_off2 = -pi/2+0.224; th_off3 = pi-0.224; th_off4 = pi; th_off5 = -pi/2; th_off6 = -pi/2; % basic pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%transform
%%%%%%%%%%%%%%%%%%%%matrix%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tform = dh(0,d0,0,0);
tform2 = dh(th1+th_off1, d1, a1,alp1);
tform3 = dh(th2+th_off2,d2,a2,alp2);
tform4 = dh(th3+th_off3,d3,a3,alp3);
tform5 = dh(th4+th_off4,d4,a4,alp4);
tform6 = dh(th5+th_off5,d5,a5,alp5);
tform7 = dh(th6+th_off6,d6,a6,alp6);% tool coordinate

T_total = tform*tform2*tform3*tform4*tform5*tform6*tform7;

%% current cam
tform_cam1 = dh(th6+th_off6,d_cam0,0,0);
tform_cam2 = trvec2tform([0,-0.4547,0]);
tform_cam3 = trvec2tform([-0.33,0,0]); % -0.33

T_cam = tform*tform2*tform3*tform4*tform5*tform6*tform_cam1*tform_cam2*tform_cam3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1,tform); 
body1.Joint = jnt1;
robot = rigidBodyTree;
addBody(robot,body1,'base'); % 로봇의 시작점이 base와 붙어있다.

body2 = rigidBody('body2'); % 강체 객체 정의 
jnt2 = rigidBodyJoint('jnt2','revolute');% joint 종류 정의
jnt2.JointAxis = [0 0 1]; % 축정의
% jnt2.PositionLimits = [deg2rad(70)-mot2mat2_ofs, deg2rad(296.37)-mot2mat2_ofs];
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); %addBody(robot,body,parentname) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2');
% % 
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
% jnt4.PositionLimits = [0-mot2mat4_ofs, 2*pi-mot2mat4_ofs];
setFixedTransform(jnt4,tform4);
body4.Joint = jnt4;
addBody(robot,body4,'body3');
% 
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
% jnt5.PositionLimits = [deg2rad(158.12)-mot2mat5_ofs, deg2rad(327.13)-mot2mat5_ofs];
setFixedTransform(jnt5,tform5);
body5.Joint = jnt5;
addBody(robot,body5,'body4');

% % 
body6 = rigidBody('body6');
fix = rigidBodyJoint('jnt6','revolute');
setFixedTransform(fix,tform6);
body6.Joint = fix;
addBody(robot,body6,'body5');
% 
body_cam1 = rigidBody('body_cam1');
jnt5_1 = rigidBodyJoint('jnt5_1','fixed');
setFixedTransform(jnt5_1,tform_cam1);
body_cam1.Joint = jnt5_1;
addBody(robot,body_cam1,'body6');

body_cam2 = rigidBody('body_cam2');
jnt5_2 = rigidBodyJoint('jnt5_2','fixed');
setFixedTransform(jnt5_2,tform_cam2);
body_cam2.Joint = jnt5_2;
addBody(robot,body_cam2,'body_cam1');

body_cam3 = rigidBody('body_cam3');
jnt5_3 = rigidBodyJoint('jnt5_3','fixed');
setFixedTransform(jnt5_3,tform_cam3);
body_cam3.Joint = jnt5_3;
addBody(robot,body_cam3,'body_cam2');

body7 = rigidBody('body7');
jnt6 = rigidBodyJoint('fix','fixed');
setFixedTransform(jnt6,tform7);
body7.Joint = jnt6;
addBody(robot,body7,'body6');


th1 = -deg2rad(90);
th2 = -deg2rad(90);
th3= deg2rad(0);
th4 = deg2rad(0);
th5 = deg2rad(0);
th6 = deg2rad(0);

Cconfig = homeConfiguration(robot);
Cconfig(1).JointPosition = -pi/2;
Cconfig(2).JointPosition = -pi/2;
Cconfig(3).JointPosition = 0;
Cconfig(4).JointPosition = 0;
Cconfig(5).JointPosition = 0;
Cconfig(6).JointPosition = 0;
% show(robot,Cconfig);
% hold on


%% previous camera model
a = [a1;a2;a3;a4;a5;a6;];
d = [d1;d2;d3;d4;d5;d6;];
alp = [alp1;alp2;alp3;alp4;alp5;alp6;];
th_off = [th_off1;th_off2;th_off3;th_off4;th_off5;th_off6;];
q = [th1;th2;th3;th4;th5;th6;];
% 
dh_cam = [d(1:5,1),th_off(1:5,1),a(1:5,1),alp(1:5,1)];
DH = [d,th_off,a,alp];

q_off = [270.7;178.43;180.11;181.37;179.3;270;197.67;]; %7x1

