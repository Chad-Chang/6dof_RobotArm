robot_validate_now
diary('/home/chad-chang/Desktop/last_robot_class/diary/T_cam_sym.m')
T_cam_sym = cam_FK_test % FK_cam
diary off

robot = RobotKinematics_chad(6,'rrrrrr',[],T_cam,DH);
diary('/home/chad-chang/Desktop/last_robot_class/diary/FK_T_sym.m')
FK_T_sym = robot.m_T_th_sym % FK wrt tool coord - checked
diary off

diary('/home/chad-chang/Desktop/last_robot_class/diary/Jaco_sym.m')
Jaco_sym = robot.m_Jaco_th_sym % tool coor jacobian - checked
diary off

