
%绘制在全局坐标系下的坐标系的三个坐标轴, Firt translation then rotation
%旋转矩阵的每一行即为旋转后坐标系的每个坐标轴在原有坐标系下的向量（第一行为X轴），依据此绘制坐标轴
%参考http://blog.csdn.net/zhang11wu4/article/details/49761121
function PlotAxes_TR(Color,LineStyle,ZerosPoint,R,AxisLength,AxisLineWidth)

axisPoints=R.*AxisLength;

ZerosPoints=[ZerosPoint; ZerosPoint; ZerosPoint];

%绘制3个坐标轴，每一行为一个坐标轴
quiver3(ZerosPoints(:,1),ZerosPoints(:,2),ZerosPoints(:,3),...
    axisPoints(:,1),axisPoints(:,2),axisPoints(:,3),Color,'LineStyle',LineStyle,'LineWidth',AxisLineWidth);

%标注坐标轴
text(axisPoints(1,1)+ZerosPoint(1), axisPoints(1,2)+ZerosPoint(2),...
    axisPoints(1,3)+ZerosPoint(3),'X');
text(axisPoints(2,1)+ZerosPoint(1), axisPoints(2,2)+ZerosPoint(2),...
    axisPoints(2,3)+ZerosPoint(3),'Y');
text(axisPoints(3,1)+ZerosPoint(1), axisPoints(3,2)+ZerosPoint(2),...
    axisPoints(3,3)+ZerosPoint(3),'Z');

end