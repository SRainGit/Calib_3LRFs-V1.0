
%������ȫ������ϵ�µ�����ϵ������������, Firt translation then rotation
%��ת�����ÿһ�м�Ϊ��ת������ϵ��ÿ����������ԭ������ϵ�µ���������һ��ΪX�ᣩ�����ݴ˻���������
%�ο�http://blog.csdn.net/zhang11wu4/article/details/49761121
function PlotAxes_TR(Color,LineStyle,ZerosPoint,R,AxisLength,AxisLineWidth)

axisPoints=R.*AxisLength;

ZerosPoints=[ZerosPoint; ZerosPoint; ZerosPoint];

%����3�������ᣬÿһ��Ϊһ��������
quiver3(ZerosPoints(:,1),ZerosPoints(:,2),ZerosPoints(:,3),...
    axisPoints(:,1),axisPoints(:,2),axisPoints(:,3),Color,'LineStyle',LineStyle,'LineWidth',AxisLineWidth);

%��ע������
text(axisPoints(1,1)+ZerosPoint(1), axisPoints(1,2)+ZerosPoint(2),...
    axisPoints(1,3)+ZerosPoint(3),'X');
text(axisPoints(2,1)+ZerosPoint(1), axisPoints(2,2)+ZerosPoint(2),...
    axisPoints(2,3)+ZerosPoint(3),'Y');
text(axisPoints(3,1)+ZerosPoint(1), axisPoints(3,2)+ZerosPoint(2),...
    axisPoints(3,3)+ZerosPoint(3),'Z');

end