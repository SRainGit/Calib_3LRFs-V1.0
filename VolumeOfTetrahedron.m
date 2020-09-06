
function Volume=VolumeOfTetrahedron(Tetrahedron)
%% Solution 1 (may have some problems)
% A=[1,1,1,1;
%     Tetrahedron'];
% 
% Volume=1/6*det(A);


%% Solution 2
a=Tetrahedron(2,:)-Tetrahedron(1,:);
b=Tetrahedron(3,:)-Tetrahedron(1,:);
c=Tetrahedron(4,:)-Tetrahedron(1,:);

%|(a¡Áb)¡¤c|/6
Volume=abs(dot(cross(a,b),c))/6;

end