clc
close all
clear

%用于展示2D激光扫描仪扫描长方体形走廊时推算LS位姿的示意图

%% 设置参数
%走廊的长和宽
galleryWidth=2400+12*2;%mm
galleryHeight=2290;%mm
galleryLength=7000;
%截面参数
sideLineLength=2500;
topLineLength=2500;
%传感器位置和旋转角
laserLocation=[800 1200 500];
rotateAng=20*pi/180;
%基准角点与其他角点
Corners=zeros(4,4,3);%存储四种情况下的四个角点，第一维代表四种情况，第二维表示四个角点
baseCornerPt=[galleryWidth 3500 galleryHeight];%四种情况下的基准点，即共点



%% 绘图
figure,hold on,axis equal,
axis off;  rotate3d on;
h = [-950 200 500 400];
set(gcf,'Position',h)
sectionColors='rgbc';
% for i=0:1
%     for j=0:1
%         Corners(i*2+j+1,3,:)=baseCornerPt;
%         vEdgeSide=[0,(i*2-1)*sqrt(sideLineLength^2-galleryHeight^2),-galleryHeight];
%         vEdgeUpDown=[-galleryWidth,(j*2-1)*sqrt((topLineLength^2-galleryWidth^2)),0];
%         Corners(i*2+j+1,4,:)=baseCornerPt+vEdgeSide;
%         Corners(i*2+j+1,2,:)=baseCornerPt+vEdgeUpDown;
%         Corners(i*2+j+1,1,:)=Corners(i*2+j+1,4,:)+(Corners(i*2+j+1,2,:)-Corners(i*2+j+1,3,:));
%         fill3(Corners(i*2+j+1,:,1),Corners(i*2+j+1,:,2),Corners(i*2+j+1,:,3),sectionColors(i*2+j+1),'linewidth',2);
% %         baseCornerPt(2)=baseCornerPt(2)+2*sqrt((topLineLength^2-galleryWidth^2));
%     end
% %         baseCornerPt(2)=baseCornerPt(2)+1*sqrt(sideLineLength^2-galleryHeight^2);
% end

%% for my paper
baseCornerPt=[galleryWidth 4000 galleryHeight];%四种情况下的基准点，即共点
% galleryLength=8000;
sectionColors='gbr';
for i=0:1
    for j=0:1
        Corners(i*2+j+1,3,:)=baseCornerPt;
        vEdgeSide=[0,(i*2-1)*sqrt(sideLineLength^2-galleryHeight^2),-galleryHeight];
        vEdgeUpDown=[-galleryWidth,(j*2-1)*sqrt((topLineLength^2-galleryWidth^2)),0];
        Corners(i*2+j+1,4,:)=baseCornerPt+vEdgeSide;
        Corners(i*2+j+1,2,:)=baseCornerPt+vEdgeUpDown;
        Corners(i*2+j+1,1,:)=Corners(i*2+j+1,4,:)+(Corners(i*2+j+1,2,:)-Corners(i*2+j+1,3,:));
%         Corners(i*2+j+1,:,2)=Corners(i*2+j+1,:,2)+1500*(i*2+j+1);
    end
end
for i=3:-1:1
%     fill3(Corners(i,:,1),Corners(i,:,2),Corners(i,:,3),sectionColors(i),'linewidth',2);
end

%绘制走廊
PlotBoundingBox('k','-',0,galleryHeight,0,galleryWidth,galleryLength,0);
PlotAxes_TR('k','-',[0 0 0],eye(3,3),1000,3);

%图例
% legend('Scanning Plane1','Scanning Plane2','Scanning Plane3','Scanning Plane4');
% legend('Scanning Plane1','Scanning Plane2','Scanning Plane3');

%% 绘制截面的平面图
Corners_2D=zeros(4,3);
Corners_2D(1,:)=[0 0 0];
vSideEdge_2D=[sqrt(sideLineLength^2-galleryHeight^2),galleryHeight,0];
vUpDownEdge_2D=[topLineLength,0,0];
Corners_2D(2,:)=Corners_2D(1,:)+vSideEdge_2D;
Corners_2D(4,:)=Corners_2D(1,:)+vUpDownEdge_2D;
Corners_2D(3,:)=Corners_2D(2,:)+(Corners_2D(4,:)-Corners_2D(1,:));
%进行一定角度的旋转
Corners_2D=(EularAngle2RotateMat(0,0,10*pi/180,'xyz')*Corners_2D')';
%平移
Corners_2D(:,1)=Corners_2D(:,1)-1300;
Corners_2D(:,2)=Corners_2D(:,2)-400;

figure,hold on,axis equal,
fill3(Corners_2D(:,1),Corners_2D(:,2),Corners_2D(:,3),'r');
text(Corners_2D(1,1), Corners_2D(1,2),Corners_2D(1,3),'A','FontSize',20);
text(Corners_2D(2,1), Corners_2D(2,2),Corners_2D(2,3),'B','FontSize',20);
text(Corners_2D(3,1), Corners_2D(3,2),Corners_2D(3,3),'C','FontSize',20);
text(Corners_2D(4,1), Corners_2D(4,2),Corners_2D(4,3),'D','FontSize',20);
xlabel('X/mm');
ylabel('Y/mm');
ylabel('Z/mm');
PlotAxes_TR('k','-',[0 0 0],eye(3,3)*EularAngle2RotateMat(0,pi,0,'xyz'),1000,3);
axis off;





