function PlotBoundingBox(color,lineStyle,direction,h,...
    leftEdge,rightEdge,frontEdge,rearEdge)

%��һ��ͶӰ�����ĽǶ�
angle=direction;
%��ת�Ƕȶ�Ӧ����ת����
rotateMat=[cos(angle) -sin(angle);
    sin(angle) cos(angle)];

%���ƶ�����ĸ��ǵ�
corners=zeros(4,3);
corners(1,1:2)=[leftEdge frontEdge]*rotateMat;%��ǰ
corners(2,1:2)=[leftEdge rearEdge]*rotateMat;%���
corners(3,1:2)=[rightEdge frontEdge]*rotateMat;%��ǰ
corners(4,1:2)=[rightEdge rearEdge]*rotateMat;%�Һ�
corners(:,3)=h;
plot3(corners(:,1),corners(:,2),corners(:,3),[color 's']);
%���ƶ����������
line1 = [corners(1,:); corners(2,:)];
line2 = [corners(3,:); corners(4,:)];
line3 = [corners(1,:); corners(3,:)];
line4 = [corners(2,:); corners(4,:)];
line(line1(:,1),line1(:,2),line1(:,3),'color',color,'LineStyle',lineStyle)
line(line2(:,1),line2(:,2),line2(:,3),'color',color,'LineStyle',lineStyle)
line(line3(:,1),line3(:,2),line3(:,3),'color',color,'LineStyle',lineStyle)
line(line4(:,1),line4(:,2),line4(:,3),'color',color,'LineStyle',lineStyle)
%���Ƶ�����ĸ��ǵ�
corners(:,3)=0;
plot3(corners(:,1),corners(:,2),corners(:,3),[color 's']);
%���Ƶ����������
line1 = [corners(1,:); corners(2,:)];
line2 = [corners(3,:); corners(4,:)];
line3 = [corners(1,:); corners(3,:)];
line4 = [corners(2,:); corners(4,:)];
line(line1(:,1),line1(:,2),line1(:,3),'color',color,'LineStyle',lineStyle)
line(line2(:,1),line2(:,2),line2(:,3),'color',color,'LineStyle',lineStyle)
line(line3(:,1),line3(:,2),line3(:,3),'color',color,'LineStyle',lineStyle)
line(line4(:,1),line4(:,2),line4(:,3),'color',color,'LineStyle',lineStyle)
%���Ʋ����������
line1 = [[corners(1,1:2) 0]; [corners(1,1:2) h]];
line2 = [[corners(2,1:2) 0]; [corners(2,1:2) h]];
line3 = [[corners(3,1:2) 0]; [corners(3,1:2) h]];
line4 = [[corners(4,1:2) 0]; [corners(4,1:2) h]];
line(line1(:,1),line1(:,2),line1(:,3),'color',color,'LineStyle',lineStyle)
line(line2(:,1),line2(:,2),line2(:,3),'color',color,'LineStyle',lineStyle)
line(line3(:,1),line3(:,2),line3(:,3),'color',color,'LineStyle',lineStyle)
line(line4(:,1),line4(:,2),line4(:,3),'color',color,'LineStyle',lineStyle)

end