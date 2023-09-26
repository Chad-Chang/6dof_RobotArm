function [robot_op] = run_align(robot_op,commu,n,s, t, visual) % align only th6...
%     if robot_op.m_status ==1 % run to see the label
        %% get angle 
%% an6_str: angle wrt camera frame : has to consider the angle difference of endeffector and camera.
        ang6_str = commu.receive_t(t); 
        ang6 = str2double(ang6_str);
        ang6 = deg2rad(ang6);

        q = robot_op.m_q;
        q2 = q;
        q2(6) = q2(6) + ang6;
        
        ttform = FK(q2);
        robot_op.m_targ = ttform;
        ttform_f = robot_op.m_targ_f;

        ttform_f(1:3,1:3) = Rodrigues(ttform_f(1:3,3),ang6)*ttform_f(1:3,1:3);
        robot_op.m_targ_f = ttform_f;
       
        for i = 1/n:1/n:1
%             ctform_t = robot_op.m_T;
%             error_scale = norm(ttform(1:3,4)-ctform_t(1:3,4));
%             if(norm(ttform(1:3,4)-ctform_t(1:3,4))< 1e-3)
%                 break
%             end

            q3 = j_space_tr(q2,q,1,1/n,i);
            [robot_op.m_T, robot_op.m_T_cam, robot_op.m_q, robot_op.m_q_ardu] = robot_op.update_pose(q3);
            if(visual)
                robot_op.robot_vis();
                robot_op.target_vis(1,1);
            end
        end
        robot_op.m_targ(1:3,1:3) = robot_op.m_T(1:3,1:3);
        robot_op.m_targ_f(1:3,1:3) =robot_op.m_T(1:3,1:3); 
        CT = robot_op.m_T
        targT = robot_op.m_targ
            %% trans to arduino
            commu.send_s(s,robot_op.m_status, robot_op.m_q);
%     end
    
end
