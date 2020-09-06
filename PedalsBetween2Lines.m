
%% Calculate the pedals between two lines in 3D space
% p1 and p2 are on the line1 and line2 respectively
% v1 and v2 are the two lines's direction vector
% c1 and c2 are the two pedals
% p1+a1*v1=c1
% p2+a2*v2=c2
% c1-c2=d; (d is the distance vector)
% so we can get : a1*v1-a2*v2=d-(p1-p2), so that we can get a1 and a2
% and finnaly c1 and c2
function [c1,c2] = PedalsBetween2Lines(p1,v1,p2,v2)

v1=v1/norm(v1);
v2=v2/norm(v2);

distVector=cross(v1,v2);
distVector=distVector/norm(distVector);


% the shadow of p2 in the plane of which parralel to l2 and conclude l1
p2_=p2+dot((p1-p2),distVector)*distVector;

if (1-(dot(v1,v2))^2)<0.0000001
    a=dot((p1-p2_),v1);
else
    a=(dot((p1-p2_),v1)-dot(p1-p2_,v2)*dot(v1,v2))/(1-(dot(v1,v2))^2);
end

c1=p1-a*v1;
c2=c1+(p2-p2_);

end