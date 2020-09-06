
%绘制在全局坐标系下的坐标系的三个坐标轴, Firt roatation then translation
%旋转矩阵的每一行即为旋转后坐标系的每个坐标轴在原有坐标系下的向量（第一行为X轴），依据此绘制坐标轴
%参考http://blog.csdn.net/zhang11wu4/article/details/49761121
function PlotAxes_RT(Color,LineStyle,R,T,AxisLength,AxisLineWidth)

axisPoints=R.*AxisLength;

ZerosPoints=-1*[T; T; T];
ZerosPoints=(R'*ZerosPoints')';

%绘制3个坐标轴，每一行为一个坐标轴
quiver3(ZerosPoints(:,1),ZerosPoints(:,2),ZerosPoints(:,3),...
    axisPoints(:,1),axisPoints(:,2),axisPoints(:,3),Color,'LineStyle',LineStyle,'LineWidth',AxisLineWidth);

%标注坐标轴
text(axisPoints(1,1)+ZerosPoints(1,1), axisPoints(1,2)+ZerosPoints(1,2),...
    axisPoints(1,3)+ZerosPoints(1,3),'X');
text(axisPoints(2,1)+ZerosPoints(1,1), axisPoints(2,2)+ZerosPoints(1,2),...
    axisPoints(2,3)+ZerosPoints(1,3),'Y');
text(axisPoints(3,1)+ZerosPoints(1,1), axisPoints(3,2)+ZerosPoints(1,2),...
    axisPoints(3,3)+ZerosPoints(1,3),'Z');

end