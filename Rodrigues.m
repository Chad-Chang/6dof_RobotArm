function rod = Rodrigues(vector, theta)
vrr = [vector vector vector];
vrr = vrr.*(vrr.');
rv = [0 -vector(3) vector(2); vector(3) 0 -vector(1); -vector(2) vector(1) 0];

rod = cos(theta) * eye(3) + (1 - cos(theta)) * vrr + sin(theta)*rv; %로드리게스 회전공식
