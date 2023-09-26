function theta_out = j_space_tr(theta_F,theta_I,Tf,Ti,t)
    [N ,~] = size(theta_F);
    for i = 1:N
        theta_out(i) = theta_I(i) + (theta_I(i) - theta_F(i)) * (cos(pi / (Tf - Ti) * (t - Ti)) - 1) * 0.5; % path planning
    end
    theta_out = theta_out.';
end

