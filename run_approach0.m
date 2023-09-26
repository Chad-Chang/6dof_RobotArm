function [robot_op,data] = run_approach0(robot_op,commu, n,iteration,s, t,test,visual) % move to read label
    nothing = false;
        %% receiving data
        %update target
        if(test)
            ttform = eye(4);
            ttform(1:3,1:3) = [-1 0 0;0 0 -1; 0 -1 0;];
            ttform(1:3,4) = [0,-2,2.5];
            
            alpha = 1;
            a_xy = ttform(1:2,3);
            a_xy = a_xy/norm(a_xy);
            ttform(1:2,4) = ttform(1:2,4)-alpha*a_xy;
            ttform(1:3,1:3) = eul2rotm([pi/3,0,0])*ttform(1:3,1:3);
            ttform = [      -1,0,0,0;
                            0,0,-1,-1.6;
                            0,-1,0,2.8866;
                            0,0,0,1.0000;];
            data = 1;
        else
            data = commu.receive_t(t); % string data
            disp("approach")
            data
%             t.NumBytesAvailable
            if(data == "b")
                ttform = robot_op.m_T;
                ttform(1:3,4) = ttform(1:3,4) - 0.1*ttform(1:3,3);
            elseif(data == "r") % go right
                ttform = robot_op.m_T;
                ttform(1,4) = ttform(1,4) - 0.1;
            elseif(data == "l") % go left
                ttform = robot_op.m_T;
                ttform(1,4) = ttform(1,4) + 0.1;
            elseif(data == "2")
                nothing = true;
                robot_op = run_return(robot_op,commu,150,s,visual);
                commu.send_s(s,2,[0,0,0,0,0,0]); % when it ends
                commu.send_t(t,"/3.5/");
                robot_op.m_status = 3;
            else
                H_0 = eye(4,4);
                buff = str2double(strsplit(data,' '));
                %% not changed ..
                H_0(1,1:4) = buff(2:5); % pushed once
                H_0(2,1:4) = buff(6:9);
                H_0(3,1:4) = buff(10:13);
                H_0(1:3,4) = H_0(1:3,4);
                H_0(1:3,4) = H_0(1:3,4)/100;
            
                %% change - maintain horizontal orientation
                ttform = robot_op.m_T_cam*H_0;
                ttform(1:3,2) = [0,0,-1];
                ttform(3,3) = 0; % projection to x,y w.r.t base
                ttform(1:3,3) = ttform(1:3,3)/norm(ttform(1:3,3));
                ttform(1:3,1) = cross(ttform(1:3,2),ttform(1:3,3));
                robot_op.m_targ_f = ttform; 
                %% next     
                distance = 1.5;
                ttform(1,4) = 0;
                ttform(2,4) = ttform(2,4) + distance;
                ttform(3,4) = ttform(3,4);
                ttform(2,4) = ttform(2,4);
                ttform(1:3,1:3) = [-1,0,0;0,0,-1;0,-1,0];
                ttform
            end
        end
    if(~nothing)
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
            if(~test)
                commu.send_s(s,robot_op.m_status,robot_op.m_q);
            end
        end
    end
end
