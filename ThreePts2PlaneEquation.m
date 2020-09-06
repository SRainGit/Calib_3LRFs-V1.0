
% Input 3 points, and output a plane equation based on them
% A*x+B*y+C*z+D=0;
function [a,b,c,d]=ThreePts2PlaneEquation(Pts)

if size(Pts,1)~=3 || size(Pts,2)~=3
    fprintf(2,'Warning: The "Pts" is wrong for input!');
    return;
end

% compute the norm vector
v12=Pts(2,:)-Pts(1,:);
v13=Pts(3,:)-Pts(1,:);

if norm(cross(v12,v13))/(norm(v12)*norm(v13))<sin(20*pi/180)
%     disp('Warning: The "Pts" are close to colinear!');
    a=0;b=0;c=0;d=0;
    return;
end

n=cross(v12,v13);
n=n/norm(n);

a=n(1);
b=n(2);
c=n(3);
d=-(n(1)*Pts(1,1)+n(2)*Pts(1,2)+n(3)*Pts(1,3));

end