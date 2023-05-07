function dwdq = calc3DOFdwdq(q)
% syms q1 q2 q3
% 
% J = [- sin(q1 + q2 + q3)/2 - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3)/2 - sin(q1 + q2), -sin(q1 + q2 + q3)/2;
%      cos(q1 + q2 + q3)/2 + cos(q1 + q2) + cos(q1),   cos(q1 + q2 + q3)/2 + cos(q1 + q2),  cos(q1 + q2 + q3)/2];
%  
% w = sqrt(det(J*transpose(J)))
% 
% w =((cos(q1 + q2 + q3)^2*sin(q1)^2)/2 + (sin(q1 + q2 + q3)^2*cos(q1)^2)/2 + cos(q1 + q2)^2*sin(q1)^2 + sin(q1 + q2)^2*cos(q1)^2 + (cos(q1 + q2 + q3)^2*sin(q1 + q2)^2)/2 + (sin(q1 + q2 + q3)^2*cos(q1 + q2)^2)/2 + cos(q1 + q2 + q3)*cos(q1 + q2)*sin(q1)^2 + (sin(q1 + q2 + q3)^2*cos(q1 + q2)*cos(q1))/2 + sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)^2 + (cos(q1 + q2 + q3)^2*sin(q1 + q2)*sin(q1))/2 - (cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1))/2 - (cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1))/2 - cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1)*sin(q1) - 2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1) - cos(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)*sin(q1) - sin(q1 + q2 + q3)*cos(q1 + q2)*cos(q1)*sin(q1) - cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1 + q2))^(1/2)
% 
% dwdq_sym = simplify(jacobian(w, [q1, q2, q3]))

%%%%%%% NUMERICAL
q1 = q(1);
q2 = q(2);
q3 = q(3);

dwdq = [0, (sin(q2 + 2*q3) + 4*sin(2*q2 + q3) + 4*sin(2*q2) + 2*sin(2*q2 + 2*q3) - sin(q2))/(4*(cos(q2) - 2*cos(2*q2 + q3) - 2*cos(2*q2) - cos(2*q3) - cos(2*q2 + 2*q3) - cos(q2 + 2*q3) + 2*cos(q3) + 4)^(1/2)), (sin(q2 + 2*q3) + sin(2*q2 + q3) + sin(2*q3) + sin(2*q2 + 2*q3) - sin(q3))/(2*(cos(q2) - 2*cos(2*q2 + q3) - 2*cos(2*q2) - cos(2*q3) - cos(2*q2 + 2*q3) - cos(q2 + 2*q3) + 2*cos(q3) + 4)^(1/2))];


end