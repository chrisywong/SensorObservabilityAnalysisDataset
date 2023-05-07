function dodq = calc3DOFdodq(q)
q1 = q(1);
q2 = q(2);
q3 = q(3);

dodq = [(sign(sin(q1))*cos(q1) + sign(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))*(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - sign(cos(q1)*cos(q2) - sin(q1)*sin(q2))*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))*(abs(cos(q1)) + abs(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + abs(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - (sign(cos(q1))*sin(q1) + sign(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))*(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) - sign(cos(q1)*sin(q2) + cos(q2)*sin(q1))*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))*(abs(sin(q1)) + abs(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + abs(cos(q1)*cos(q2) - sin(q1)*sin(q2))), (sign(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))*(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - sign(cos(q1)*cos(q2) - sin(q1)*sin(q2))*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))*(abs(cos(q1)) + abs(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + abs(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - (sign(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))*(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) - sign(cos(q1)*sin(q2) + cos(q2)*sin(q1))*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))*(abs(sin(q1)) + abs(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + abs(cos(q1)*cos(q2) - sin(q1)*sin(q2))), sign(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))*(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))*(abs(cos(q1)) + abs(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + abs(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - sign(cos(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))*(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))*(abs(sin(q1)) + abs(cos(q1)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q1)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + abs(cos(q1)*cos(q2) - sin(q1)*sin(q2)))];

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
% o = prod(sum(stilde, 2)); %using sum because symbolic max doesn't work with jacobian() function
% 
% dodq_sym = jacobian(o, [q1, q2, q3]);

 
end