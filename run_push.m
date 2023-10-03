function [robot_op] = run_push(robot_op,commu, n,iteration,s, t, visual) % strech the arm to insert
%     if robot_op.m_status ==2 % run to see the label
       %% input data
        ttform = robot_op.m_targ_f; % this is the standard
        a_xy = ttform(1:2,3);
        a_xy = a_xy/norm(a_xy); % norm_axy = 1
        alpha = 1.2;
        
        ttform(1:2,4) = ttform(1:2,4) + alpha*a_xy;
%         ttform(1,4) = ttform(1,4) + 0.07;
        ttform(1,4) = ttform(1,4);
        robot_op.m_targ = ttform;
        robot_op.m_targ_f = ttform;
        CT_insert = robot_op.m_T;
        TT_insert = robot_op.m_targ_f;

        %% run
        [Nv,theta1,Rv,theta2,temp_Ori] = robot_op.calc_Rot();
        dtheta1 = 0;
        dtheta2 = 0;
        theta_sum = 0;
        theta_sum2 = 0;
        for i = 1/n: 1/n:1
%             tic
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
%                 dtheta1 = robot_op.RotCheck(theta1, 1, 0, 2*i);
                dtheta1 = robot_op.RotCheck(theta1, 1, 0, i);
                delta_theta1 = dtheta1-old_theta1;
                theta_sum = theta_sum+delta_theta1;
                rodri = Rodrigues(Nv,delta_theta1);
            else
            
                if i == 0.5
                    targ_O = temp_Ori;
                end
                 if i>0.5
                    old_theta2 = dtheta2;
%                     dtheta2 = robot_op.RotCheck(theta2, 1, 0, 2*(i-0.5));
                    dtheta2 = robot_op.RotCheck(theta2, 1, 0, (i-0.5));
                    delta_theta2 = dtheta2 - old_theta2;
                    theta_sum2 = theta_sum2 + delta_theta2;
    %                 if(tr_index==2) 
    %                     delta_theta2 = 0;
    %                 end
                    rodri = Rodrigues(Rv,delta_theta2);
                 end
            end
            targ_O = rodri * ctform(1:3,1:3);

            q = robot_op.m_q;
%             ctform_t = robot_op.m_T;
            [ex,ey,ez] = robot_op.calc_PE(temp_targP,ctform_t(1:3,4));
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
            [robot_op.m_T, robot_op.m_T_cam, robot_op.m_q, robot_op.m_q_ardu] = robot_op.update_pose(q);
            if(visual)
                robot_op.robot_vis();
                robot_op.target_vis(1,1);
            end
            %% trans to arduino
            commu.send_s(s,robot_op.m_status, robot_op.m_q);
%             toc
        end     
end