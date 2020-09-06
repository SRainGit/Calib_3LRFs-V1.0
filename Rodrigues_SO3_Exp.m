%% 
function R=Rodrigues_SO3_Exp(w,A,B)
R=zeros(3,3);

wx2=w(1)^2;
wy2=w(2)^2;
wz2=w(3)^2;

R(1,1) = 1.0 - B*(wy2 + wz2);
R(2,2) = 1.0 - B*(wx2 + wz2);
R(3,3) = 1.0 - B*(wx2 + wy2);

a = A*w(3);
b = B*(w(1)*w(2));
R(1,2) = b - a;
R(2,1) = b + a;

a = A*w(2);
b = B*(w(1)*w(3));
R(1,3) = b + a;
R(3,1) = b - a;

a = A*w(1);
b = B*(w(2)*w(3));
R(2,3) = b - a;
R(3,2) = b + a;

end