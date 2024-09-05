% q0 = 1;
% q1 = .1;
% q2 = .1;
% q3 = 1;
x = 0;
s = 10000; %number of iterations
% qpre = [q0,q1,q2,q3];
for i = 1:s
 qpre = 2*rand(1,4) -1;

q = normalize(quaternion(qpre));

[theta1, theta2, theta3] = quat2angle(q,'ZXY');

[p0, p1,p2,p3] = parts(q);

t1 = atan(2*(p0*p3 - p1*p2)/(1-2*(p3^2 +  p1^2)));
t2 = asin(2*(p0*p1 + p3*p2));
t3 = atan(2*(p0*p2 - p3*p1)/(1-2*(p1^2 + p2^2)));

diff1 = theta1-t1;
diff2 = theta2-t2;
diff3 = theta3-t3;
tol = 1E-13;



if ((abs(diff1)>tol) && (abs(diff1) - pi > tol))
    d1 = abs(diff1)
   x = x + 1;

elseif ((abs(diff2)>tol) && (abs(diff2) - pi >tol))
    d2 = abs(diff2)
      x = x + 1;

elseif ((abs(diff3)>tol) && (abs(diff3) - pi >tol))
    d3 = abs(diff3)
      x = x + 1;
end
end

fprintf("number of faults in checks: %u out of %u iterations\n",x,s)
