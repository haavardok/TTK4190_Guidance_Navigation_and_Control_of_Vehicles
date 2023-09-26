function [alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)
% [alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)


% Add your code here

% Air speed expressed in BODY
v_a = v_g - v_w; % Vector

% Scalar speed expressed in BODY
Va = norm(v_a);
Vg = norm(v_g);
Vw = norm(v_w);

% Angle of attack
%alpha = rad2deg(atan2(v_a(3),v_a(1)));
alpha = atand(v_a(3)/v_a(1));

% Sideslip
beta = asind(v_a(2)/norm(v_a));

end