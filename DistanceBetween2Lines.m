
% Calculate the distance between two lines in 3D space
% pt1 and pt2 are on the line1 and line2 respectively
% v1 and v2 are the two lines's direction vector
% distance = (pt1-pt2) * (v1 x v2) / norm((v1 x v2))
function distance=DistanceBetween2Lines(pt1,v1,pt2,v2)

distVector=cross(v1,v2);

if norm(distVector)<0.0001
    distance = 0;
else
    distance=abs(dot((pt1-pt2),distVector)/norm(distVector));
end

end