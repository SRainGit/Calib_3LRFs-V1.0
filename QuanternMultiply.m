
% q=q0+i*q1+j*q2+k*q3;
% r=r0+i*r1+j*r2+k*r3;
% n=qxr=n0+i*n1+j*n2+k*n3;
function n=QuanternMultiply(q0,q1,q2,q3,r0,r1,r2,r3)
n0=r0*q0-r1*q1-r2*q2-r3*q3;
n1=r0*q1+r1*q0-r2*q3+r3*q2;
n2=r0*q2+r1*q3+r2*q0-r3*q1;
n3=r0*q3-r1*q2+r2*q1+r3*q0;
n=[n0 n1 n2 n3];
end
