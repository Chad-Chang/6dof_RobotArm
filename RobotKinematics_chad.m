classdef RobotKinematics_chad
    
    properties
        m_n_joints = []; % ex) 6joint
        m_joint_types = []; % ex) 6revolute : rrrrrr
        m_T_init = []; % ex) base frame
        m_DH_params = []; % DH parameter about tool
        m_DH_cam = []; % DH parameter about camera 
        m_q = []; % angle 6x1
        m_T_cam = []; % camera TF (4x4)
        m_tool_added = 0; % is tool added?
        m_T_sym = []; % total TF about th and DH parameters : function(a,alp,theta,r, theat1~thetaN), symbolic
        m_T_th_sym = []; % total TF about th :function(theat1~thetaN), symbolic
        m_T_cam_dh_sym = []; % camera equiped TF
        m_T_cam_th_sym = [] % 
        %derivative of oritnt (condition 2) wrt DH params
        m_orient_cond = []
        m_Jaco_th_sym = [] % jacobian(theta)
        m_Jaco_dh_sym = [] % jacobian(dh)
    end
     methods

        function obj = RobotKinematics_chad(n_joints, types,T_init,T_cam, DH_params)
            DH_params = reshape(DH_params.',4*n_joints,1);
            if isempty(T_cam) == 0
                obj.m_tool_added = 1;
                obj.m_T_cam = T_cam;
            end
            if isempty(T_init)
                obj.m_T_init = DH_mat([0.381,0,0,0]);
            else
                obj.m_T_init = T_init;
            end
            obj.m_DH_cam = DH_params(1:4*n_joints-4);
            obj.m_n_joints = n_joints;
            obj.m_joint_types = types;
            
            obj.m_DH_params = sym('m_DH_params',[4*n_joints,1]);
            obj.m_q = sym('m_q',[n_joints,1]);
            
            [obj.m_T_sym,obj.m_T_cam_dh_sym,obj.m_Jaco_dh_sym] = getKineSym(obj);
            [obj, obj.m_T_th_sym, obj.m_T_cam_th_sym,obj.m_Jaco_th_sym] = getPose(obj,DH_params);

        end

        %% Symbolic FWD Kine
        function [T,T_cam,Jaco] = getKineSym(obj)
            
            T = obj.m_T_init;
            T = sym(T);
            T_cam = T;
            F(:,:,1) = sym(obj.m_T_init);
            for i = 1:obj.m_n_joints
                v1 = 4*i-3;
                xi = obj.m_DH_params(v1:v1+3);
                
                if obj.m_joint_types(i) == 'r'
                    
                    xi(2) = xi(2)+obj.m_q(i);
                else
                    xi(1) = xi(1)+obj.m_q(i);
                end
                if i< 6 % only 5axis parameter 
                    Ti_cam = DH_mat(xi);
                    T_cam = T_cam*Ti_cam;
                end
                Ti = DH_mat(xi);
                T = T*Ti;
                F(:,:,i+1) = Ti;
            end
            T_cam = T_cam*obj.m_T_cam;

            Fb0 = F(:,:,1);
            Fb1 = Fb0*F(:,:,2);
            Fb2 = (Fb1*F(:,:,3));
            Fb3 = (Fb2*F(:,:,4));
            Fb4 = (Fb3*F(:,:,5));
            Fb5 = (Fb4*F(:,:,6));
            Fb6 = (Fb5*F(:,:,7));

            Jaco1 = [(cross(Fb0(1:3,3),([Fb6(1:3,4)-Fb0(1:3,4)])));Fb0(1:3,3)];%6x1
            Jaco2 = [(cross(Fb1(1:3,3),([Fb6(1:3,4)-Fb1(1:3,4)])));Fb1(1:3,3)];%6x1
            Jaco3 = [(cross(Fb2(1:3,3),([Fb6(1:3,4)-Fb2(1:3,4)])));Fb2(1:3,3)];%6x1
            Jaco4 = [(cross(Fb3(1:3,3),([Fb6(1:3,4)-Fb3(1:3,4)])));Fb3(1:3,3)];%6x1
            Jaco5 = [(cross(Fb4(1:3,3),([Fb6(1:3,4)-Fb4(1:3,4)])));Fb4(1:3,3)];%6x1
            Jaco6 = [(cross(Fb5(1:3,3),([Fb6(1:3,4)-Fb5(1:3,4)])));Fb5(1:3,3)];%6x1
            Jaco = [Jaco1 Jaco2 Jaco3 Jaco4 Jaco5 Jaco6];
        end


        %% substitute values in symbolic functions 
        function [obj,T_th,T_cam,Jv] = getPose(obj,DH_params) % T, Jacobian : func(q)
            T_th = subs(obj.m_T_sym,obj.m_DH_params,DH_params);
            T_cam = subs(obj.m_T_cam_dh_sym,obj.m_DH_params,DH_params);
            Jv = subs(obj.m_Jaco_dh_sym,obj.m_DH_params,DH_params);
        end


        function [T_val,T_cam] = FK(obj,q) % Fk: double
            T_val = subs(obj.m_T_th_sym,obj.m_q,q);
            T_val = double(T_val);
            T_cam = subs(obj.m_T_cam_th_sym,obj.m_q,q);
            T_cam = double(T_cam);
        end
        
        function [Jaco] = Jaco_th(obj,q) % Jaco : double
            Jaco = subs(obj.m_Jaco_th_sym, obj.m_q,q);
            Jaco = double(Jaco);
        end
        
        function [q_out] = IK_LM(obj, e_in, q)
            e = e_in;
            Jacobian = obj.Jaco_th(q)
            We = diag([1 1 1 1 1 1]);
            Eq = 0.5*e.'*We*e; 
            g = Jacobian.'*We*e;
            
            %Wn_bar = 0.001;
            Wn = Eq.*eye(6);                        %+ diag([Wn_bar Wn_bar Wn_bar Wn_bar Wn_bar Wn_bar]);
            H = Jacobian.'*We*Jacobian;
            
            % LM
            mul_LM = (H+Wn)\g;
            Q_LM_x = [q(1) q(2) q(3) q(4) q(5) q(6)].' + mul_LM;
            
            q1 = Q_LM_x(1);
            q2 = Q_LM_x(2);
            q3 = Q_LM_x(3);
            q4 = Q_LM_x(4);
            q5 = Q_LM_x(5);
            q6 = Q_LM_x(6);
            q_out = [q1;q2;q3;q4;q5;q6;];
        end

        

     end
end
        %% DH to homogeneus
function T = DH_mat(x)
            d = x(1);
            theta = x(2);
            a = x(3);
            alpha = x(4);
            
            T = zeros(4,4);
            T(4,4) = 1;
            
            T = sym(T);
            T(1,1) = cos(theta);    T(1,2) = -sin(theta)*cos(alpha);    T(1,3) = sin(theta)*sin(alpha); T(1,4) = a*cos(theta);
            T(2,1) = sin(theta);    T(2,2) = cos(theta)*cos(alpha); T(2,3) = -cos(theta)*sin(alpha);    T(2,4) = a*sin(theta);
            T(3,2) = sin(alpha);    T(3,3) = cos(alpha);    T(3,4) = d;

end

% quat = [x,y,z,w]
% if 1+R(1,1)-R(2,2)-R(3,3) > 0
function quat = Rot2Quat_1(R)
    %
    %https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternions
    t = R(1,1)-R(2,2)-R(3,3);
    
    quat(1,1) = 0.5*sqrt(1+t); %%qx
    s = 1/(4*quat(1));
    
    quat(2,1) = s*(R(1,2)+R(2,1)); %%qy
    quat(3,1) = s*(R(1,3)+R(3,1)); %%qz
    quat(4,1) = s*(R(3,2)-R(2,3)); %%qw

end

% quat = [x,y,z,w]
% if 1+R(1,1)+R(2,2)+R(3,3) > 0
function quat = Rot2Quat_2(R)
    %
    %https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternions
    t = R(1,1)+R(2,2)+R(3,3);
    
    quat(4,1) = 0.5*sqrt(1+t); %%qw
    s = 1/(4*quat(4));
    
    quat(1,1) = s*(R(3,2)-R(2,3)); %%qx
    quat(2,1) = s*(R(1,3)-R(3,1)); %%qy
    quat(3,1) = s*(R(2,1)-R(1,2)); %%qz

end

function quat = Rot2Quat_3(R)
    %
    %http://docs.ros.org/indigo/api/orocos_kdl/html/frames_8cpp_source.html#l00205t = R(1,1)+R(2,2)+R(3,3);
    
    t = R(2,2)-R(1,1)-R(3,3);
    
    quat(2,1) = 0.5*sqrt(1+t); %%qy
    s = 1/(4*quat(2));
    
    quat(1,1) = s*(R(1,2)+R(2,1)); %%qx
    quat(3,1) = s*(R(2,3)+R(3,2)); %%qz
    quat(4,1) = s*(R(1,3)-R(3,1)); %%qw

end

function quat = Rot2Quat_4(R)
%
    %http://docs.ros.org/indigo/api/orocos_kdl/html/frames_8cpp_source.html#l00205t = R(1,1)+R(2,2)+R(3,3);
    
    t = R(3,3)-R(2,2)-R(1,1);
    
    quat(3,1) = 0.5*sqrt(1+t); %%qz
    s = 1/(4*quat(3));
    
    quat(1,1) = s*(R(1,3)+R(3,1)); %%qx
    quat(2,1) = s*(R(2,3)+R(3,2)); %%qy
    quat(4,1) = s*(R(2,1)-R(1,2)); %%qw

end

function [quat,condition] = Rot2Quat(R)

    t = R(1,1)+R(2,2)+R(3,3);
    
    if (t > 1e-12)
        
        quat = Rot2Quat_2(R);
        condition = 2;
        
    else
        
        if (R(1,1) > R(2,2) && R(1,1) > R(3,3))
            
            quat = Rot2Quat_1(R);
            condition = 1;
            
        elseif (R(2,2) > R(3,3))
            
            quat = Rot2Quat_3(R);
            condition = 3;
            
        else
            quat = Rot2Quat_4(R);
            condition = 4;
        end
        
    end
end


function R = Quat2Rot(quat)
    x = quat(1);    y = quat(2);    z = quat(3); w = quat(4);
    
    R(1,1) = 1-2*y^2-2*z^2;
    R(1,2) = 2*(x*y-z*w);
    R(1,3) = 2*(x*z+y*w);
    
    R(2,1) = 2*(x*y+z*w);
    R(2,2) = 1-2*x^2-z^2;
    R(2,3) = 2*(y*z-x*w);
    
    R(3,1) = 2*(x*z-y*w);
    R(3,2) = 2*(y*z+x*w);
    R(3,3) = 1-x^2-y^2;
end
