function T_cam = cam_FK_test()
    syms th1 th2 th3 th4 th5 th6
    %%%%%%%%%%%%%%parameter%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    d0 = 0.381; d1 = 0.487;d2 = 0;d3= 0;d4 = 1.575;d5 = 0;d6 =1.524;
    d_cam0 = 0.506; 
    a1= 0;a2 = 1.0154;a3 = 0;a4 = 0;a5 = -0.218; a6 = 0;
    % a6 = 0;
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
    
%     T_total = tform*tform2*tform3*tform4*tform5*tform6*tform7;
    
    %% current cam

    tform_cam1 = dh(th6+th_off6,d_cam0,0,0);
    tform_cam2 = trvec2tform([0,-0.4547,0]);
    tform_cam3 = trvec2tform([-0.33,0,0]);
    
    T_cam = tform*tform2*tform3*tform4*tform5*tform6*tform_cam1*tform_cam2*tform_cam3;

end