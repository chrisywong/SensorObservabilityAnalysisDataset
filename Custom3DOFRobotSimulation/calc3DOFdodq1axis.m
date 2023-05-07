function dodq = calc3DOFdodq1axis(q,axis)
q1 = q(1);
q2 = q(2);
q3 = q(3);

if (axis == 1) %x-axis
  dodq = [cos(q1 + q2 + q3)*sign(sin(q1 + q2 + q3)) - sign(cos(q1 + q2))*sin(q1 + q2) + sign(sin(q1))*cos(q1), cos(q1 + q2 + q3)*sign(sin(q1 + q2 + q3)) - sign(cos(q1 + q2))*sin(q1 + q2), cos(q1 + q2 + q3)*sign(sin(q1 + q2 + q3))];
elseif (axis == 2) %y-axis
  dodq = [sign(sin(q1 + q2))*cos(q1 + q2) - sin(q1 + q2 + q3)*sign(cos(q1 + q2 + q3)) - sign(cos(q1))*sin(q1), sign(sin(q1 + q2))*cos(q1 + q2) - sin(q1 + q2 + q3)*sign(cos(q1 + q2 + q3)), -sin(q1 + q2 + q3)*sign(cos(q1 + q2 + q3))];
end

% syms q1 q2 q3
% 
% s1tilde = [-sin(q1);
%             cos(q1)];
%  
% s2tilde = [cos(q1)*cos(q2) - sin(q1)*sin(q2);
%            cos(q1)*sin(q2) + cos(q2)*sin(q1)];
%  
% s3tilde = [- cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3));
%              cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))];
% 
% stilde = abs([s1tilde, s2tilde, s3tilde]);
% 
% %ONLY USE X AXIS
% o_x = prod(sum(stilde(1,:), 2)); %using sum because symbolic max doesn't work with jacobian() function
% 
% o_y = prod(sum(stilde(2,:), 2)); %using sum because symbolic max doesn't work with jacobian() function
% 
% dodq_sym_x = simplify(jacobian(o_x, [q1, q2, q3]));
% dodq_sym_y = simplify(jacobian(o_y, [q1, q2, q3]));

 
end