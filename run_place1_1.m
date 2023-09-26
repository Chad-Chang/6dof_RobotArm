function [robot_op] = run_place1_1(robot_op,commu,n,s,visual)
    q = [0;deg2rad(-30);deg2rad(15);0;0;0;];
    robot_op.m_targ = FK(q);
    robot_op.m_targ_f = FK(q);
    for i = 1/n:1/n:1
        q2 = j_space_tr(q,robot_op.m_q,1,1/n,i);
        [robot_op.m_T, robot_op.m_T_cam, robot_op.m_q, robot_op.m_q_ardu] = robot_op.update_pose(q2);
        if(visual)
            robot_op.robot_vis();
            robot_op.target_vis(1,[]);
        end
        commu.send_s(s,robot_op.m_status,q2);
    end
end

