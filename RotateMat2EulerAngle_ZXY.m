%��ת����ŷ���ǡ�����ˣ�����*����תĿ�꣩������������ϵ˳ʱ����ת��
%����������תZXY���˳����н���
function [a,b,c]=RotateMat2EulerAngle_ZXY(R)
a=-asin(R(2,3))*180/pi;
b=atan2(-R(1,3),R(3,3))*180/pi;
c=atan2(R(2,1),R(2,2))*180/pi;
end