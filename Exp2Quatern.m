
function q = Exp2Quatern(w)  

q=zeros(1,4);

theta=norm(w);
if abs(theta)<0.00001
    q(1)=1;
    return;
end

w=w./theta;
sinHalfTheta=sin(theta/2);

q(1)=cos(theta/2);
q(2)=w(1)*sinHalfTheta;
q(3)=w(2)*sinHalfTheta;
q(4)=w(3)*sinHalfTheta;

end