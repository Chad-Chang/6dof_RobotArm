function [robot_op] = run_shelve1_1(robot_op,commu, n,s,visual) % strech the arm to insert
%     if robot_op.m_status ==4 % 
        q = robot_op.m_q;
        q2 = q;
        q2 = deg2rad([0,-35,11,0,15,-90].');
        T = FK(q2);
        robot_op.m_targ = T;
        for i = 1/n: 1/n:1
            q3 = j_space_tr(q2,q,1,1/n,i);
            [robot_op.m_T, robot_op.m_T_cam, robot_op.m_q, robot_op.m_q_ardu] = robot_op.update_pose(q3);
            if(visual)
                robot_op.robot_vis();
                robot_op.target_vis(1,[]);
            end
            commu.send_s(s,robot_op.m_status,q2);
        end
    
end
