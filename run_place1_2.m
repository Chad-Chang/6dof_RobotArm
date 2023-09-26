function [robot_op] = run_place1_2(robot_op,commu, n,iteration,s,visual) % move to read label
%     if robot_op.m_status == 1 % run to see the label
        %% receiving data
        ttform = robot_op.m_T;
        ttform(3,4) = robot_op.m_T(3,4) + 0.6;
        robot_op.m_targ = ttform; % current target
        robot_op.m_targ_f = ttform; % final target 
        %% run
        [Nv,theta1,Rv,theta2,temp_Ori] = robot_op.calc_Rot();
        dtheta1 = 0;
        dtheta2 = 0;
        theta_sum = 0;
        theta_sum2 = 0;
        for i = 1/n: 1/n:1
            %% update via point
            ctform = robot_op.m_T;
            [temp_targP, ~] = robot_op.PathCheck(1,0,i);
            ctform_t = robot_op.m_T;
            error_scale = norm(ttform(1:3,4)-ctform_t(1:3,4));
            if(norm(ttform(1:3,4)-ctform_t(1:3,4))< 1e-3)
                break
            end

            old_theta1 = dtheta1;
            if i<0.5
                dtheta1 = robot_op.RotCheck(theta1, 1, 0, 2*i);
                delta_theta1 = dtheta1-old_theta1;  
                theta_sum = theta_sum+delta_theta1;
                rodri = Rodrigues(Nv,delta_theta1);
            else
            
                if i == 0.5
                    targ_O = temp_Ori;
                end
                 if i>0.5
                    old_theta2 = dtheta2;
                    dtheta2 = robot_op.RotCheck(theta2, 1, 0, 2*(i-0.5));
                    delta_theta2 = dtheta2 - old_theta2;
                    theta_sum2 = theta_sum2 + delta_theta2;
                    rodri = Rodrigues(Rv,delta_theta2);
                 end
            end
            targ_O = rodri * ctform(1:3,1:3);
            q = robot_op.m_q;
            ctform_t = robot_op.m_T;
            %% optimization
            for j = 0:iteration
                [e1,e2,e3] = robot_op.calc_PE(temp_targP,ctform_t(1:3,4));
                if ~isequal(robot_op.m_T(1:3,1:3),ttform(1:3,1:3))
                    [e4,e5,e6] = robot_op.calc_OE(targ_O,ctform_t(1:3,1:3));
                else
                    e4 = 0; e5 = 0; e6 = 0;
                end
                e1 = real(e1); e2 = real(e2); e3 = real(e3); e4 = real(e4); e5 = real(e5); e6 = real(e6);
                e_in = [e1 e2 e3 e4 e5 e6].';
                [q] = IK(e_in,q);
                ctform_t = FK(q);
            end
            [robot_op.m_T, robot_op.m_T_cam,robot_op.m_q,robot_op.m_q_ardu] = robot_op.update_pose(q);
            if(visual)
                robot_op.robot_vis();
                robot_op.target_vis(1,1);
            end
            %% trans to arduino
            commu.send_s(s,robot_op.m_status,robot_op.m_q);
            
        end     
end
