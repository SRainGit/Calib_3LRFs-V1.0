%��ת����ŷ���ǡ�����ˣ�����*����תĿ�꣩������������ϵ˳ʱ����ת��
%����������תXYZ���˳����н���
function angles=RotateMat2EulerAngle_XYZ(R)
angles(1)=atan2(R(3,2),R(3,3))*180/pi;
angles(2)=atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2))*180/pi;
angles(3)=atan2(R(2,1),R(1,1))*180/pi;
end