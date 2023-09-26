function [Homo_form] = dh(thi, di, ai, ali)
    Homo_theta = [cos(thi) -sin(thi) 0 0;sin(thi) cos(thi) 0 0;0 0 1 0;0 0 0 1];
    Homo_d = [eye(3) [0 0 di].';[0 0 0 1]];
    Homo_a = [eye(3) [ai 0 0].';[0 0 0 1]];
    Homo_alpha = [1 0 0 0;0 cos(ali) -sin(ali) 0;0 sin(ali) cos(ali) 0;0 0 0 1];
    Homo_form = Homo_theta*Homo_d*Homo_a*Homo_alpha;
end