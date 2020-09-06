function PlotBoundingBox(color,lineStyle,direction,h,...
    leftEdge,rightEdge,frontEdge,rearEdge)

%第一个投影向量的角度
angle=direction;
%旋转角度对应的旋转矩阵
rotateMat=[cos(angle) -sin(angle);
    sin(angle) cos(angle)];

%绘制顶面的四个角点
corners=zeros(4,3);
corners(1,1:2)=[leftEdge frontEdge]*rotateMat;%左前
corners(2,1:2)=[leftEdge rearEdge]*rotateMat;%左后
corners(3,1:2)=[rightEdge frontEdge]*rotateMat;%右前
corners(4,1:2)=[rightEdge rearEdge]*rotateMat;%右后
corners(:,3)=h;
plot3(corners(:,1),corners(:,2),corners(:,3),[color 's']);
%绘制顶面的四条线
line1 = [corners(1,:); corners(2,:)];
line2 = [corners(3,:); corners(4,:)];
line3 = [corners(1,:); corners(3,:)];
line4 = [corners(2,:); corners(4,:)];
line(line1(:,1),line1(:,2),line1(:,3),'color',color,'LineStyle',lineStyle)
line(line2(:,1),line2(:,2),line2(:,3),'color',color,'LineStyle',lineStyle)
line(line3(:,1),line3(:,2),line3(:,3),'color',color,'LineStyle',lineStyle)
line(line4(:,1),line4(:,2),line4(:,3),'color',color,'LineStyle',lineStyle)
%绘制底面的四个角点
corners(:,3)=0;
plot3(corners(:,1),corners(:,2),corners(:,3),[color 's']);
%绘制底面的四条线
line1 = [corners(1,:); corners(2,:)];
line2 = [corners(3,:); corners(4,:)];
line3 = [corners(1,:); corners(3,:)];
line4 = [corners(2,:); corners(4,:)];
line(line1(:,1),line1(:,2),line1(:,3),'color',color,'LineStyle',lineStyle)
line(line2(:,1),line2(:,2),line2(:,3),'color',color,'LineStyle',lineStyle)
line(line3(:,1),line3(:,2),line3(:,3),'color',color,'LineStyle',lineStyle)
line(line4(:,1),line4(:,2),line4(:,3),'color',color,'LineStyle',lineStyle)
%绘制侧面的四条线
line1 = [[corners(1,1:2) 0]; [corners(1,1:2) h]];
line2 = [[corners(2,1:2) 0]; [corners(2,1:2) h]];
line3 = [[corners(3,1:2) 0]; [corners(3,1:2) h]];
line4 = [[corners(4,1:2) 0]; [corners(4,1:2) h]];
line(line1(:,1),line1(:,2),line1(:,3),'color',color,'LineStyle',lineStyle)
line(line2(:,1),line2(:,2),line2(:,3),'color',color,'LineStyle',lineStyle)
line(line3(:,1),line3(:,2),line3(:,3),'color',color,'LineStyle',lineStyle)
line(line4(:,1),line4(:,2),line4(:,3),'color',color,'LineStyle',lineStyle)

end