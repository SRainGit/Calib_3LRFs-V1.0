
function AngleAndAxis = Quatern2AngleAndAxis(q)

if abs(norm(q)-1)>0.0001
    errordlg('输入的q模值不是1！')
end

halfTheta=acos(q(1));
w1=q(2)/sin(halfTheta);
w2=q(3)/sin(halfTheta);
w3=q(4)/sin(halfTheta);

AngleAndAxis=[2*halfTheta,w1,w2,w3];

end