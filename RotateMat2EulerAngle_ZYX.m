%��ת����ŷ���ǡ�����ˣ�����*����תĿ�꣩������������ϵ˳ʱ����ת��
%����������תZYX���˳����н���
function angles=RotateMat2EulerAngle_ZYX(R)
angles(1)=atan2(-R(2,3),R(3,3));
angles(2)=asin(R(1,3));
angles(3)=atan2(-R(1,2),R(1,1));
end