
%������ȫ������ϵ�µ�����ϵ������������, Firt roatation then translation
%��ת�����ÿһ�м�Ϊ��ת������ϵ��ÿ����������ԭ������ϵ�µ���������һ��ΪX�ᣩ�����ݴ˻���������
%�ο�http://blog.csdn.net/zhang11wu4/article/details/49761121
function PlotAxes_RT(Color,LineStyle,R,T,AxisLength,AxisLineWidth)

axisPoints=R.*AxisLength;

ZerosPoints=-1*[T; T; T];
ZerosPoints=(R'*ZerosPoints')';

%����3�������ᣬÿһ��Ϊһ��������
quiver3(ZerosPoints(:,1),ZerosPoints(:,2),ZerosPoints(:,3),...
    axisPoints(:,1),axisPoints(:,2),axisPoints(:,3),Color,'LineStyle',LineStyle,'LineWidth',AxisLineWidth);

%��ע������
text(axisPoints(1,1)+ZerosPoints(1,1), axisPoints(1,2)+ZerosPoints(1,2),...
    axisPoints(1,3)+ZerosPoints(1,3),'X');
text(axisPoints(2,1)+ZerosPoints(1,1), axisPoints(2,2)+ZerosPoints(1,2),...
    axisPoints(2,3)+ZerosPoints(1,3),'Y');
text(axisPoints(3,1)+ZerosPoints(1,1), axisPoints(3,2)+ZerosPoints(1,2),...
    axisPoints(3,3)+ZerosPoints(1,3),'Z');

end