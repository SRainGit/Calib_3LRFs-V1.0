clc
close all
clear

%����չʾ2D����ɨ����ɨ�賤����������ʱ����LSλ�˵�ʾ��ͼ

%% ���ò���
%���ȵĳ��Ϳ�
galleryWidth=2400+12*2;%mm
galleryHeight=2290;%mm
galleryLength=7000;
%�������
sideLineLength=2500;
topLineLength=2500;
%������λ�ú���ת��
laserLocation=[800 1200 500];
rotateAng=20*pi/180;
%��׼�ǵ��������ǵ�
Corners=zeros(4,4,3);%�洢��������µ��ĸ��ǵ㣬��һά��������������ڶ�ά��ʾ�ĸ��ǵ�
baseCornerPt=[galleryWidth 3500 galleryHeight];%��������µĻ�׼�㣬������



%% ��ͼ
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
baseCornerPt=[galleryWidth 4000 galleryHeight];%��������µĻ�׼�㣬������
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

%��������
PlotBoundingBox('k','-',0,galleryHeight,0,galleryWidth,galleryLength,0);
PlotAxes_TR('k','-',[0 0 0],eye(3,3),1000,3);

%ͼ��
% legend('Scanning Plane1','Scanning Plane2','Scanning Plane3','Scanning Plane4');
% legend('Scanning Plane1','Scanning Plane2','Scanning Plane3');

%% ���ƽ����ƽ��ͼ
Corners_2D=zeros(4,3);
Corners_2D(1,:)=[0 0 0];
vSideEdge_2D=[sqrt(sideLineLength^2-galleryHeight^2),galleryHeight,0];
vUpDownEdge_2D=[topLineLength,0,0];
Corners_2D(2,:)=Corners_2D(1,:)+vSideEdge_2D;
Corners_2D(4,:)=Corners_2D(1,:)+vUpDownEdge_2D;
Corners_2D(3,:)=Corners_2D(2,:)+(Corners_2D(4,:)-Corners_2D(1,:));
%����һ���Ƕȵ���ת
Corners_2D=(EularAngle2RotateMat(0,0,10*pi/180,'xyz')*Corners_2D')';
%ƽ��
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





