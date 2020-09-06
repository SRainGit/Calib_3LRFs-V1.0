%%解算出激光的位姿
close all
clc
clear

%% 显示原始三维点云
startLaserIndex=0;
midLaserIndex=540;
endLaserIndex=1080;
angStep_degree=270/1080;
angStep_radian=angStep_degree*pi/180;
ScannerSpeed = 22;

startGroup=001;
endGroup=010;
groupDistance=100;
RawFilePath = 'D:\LaserData\20170712\1\';
Cutpoints=load([RawFilePath 'Cutpoints.txt']);%对应的墙面截取参数

rawData=zeros(endGroup-startGroup+1,endLaserIndex-startLaserIndex+1);
for groupIndex=startGroup:endGroup
    oneFile=[RawFilePath,'data_',num2str(groupIndex,'%03d'),'.csv'];
    rawData(groupIndex,:)=load(oneFile);
end
    
rawPC=zeros(size(rawData,1),size(rawData,2),3);
distance=0;
cntPs=0;
for groupIndex=startGroup:endGroup
    for laserIndex=1:size(rawData,2)
        angle=(midLaserIndex-laserIndex)*angStep_radian;
        distance=rawData(groupIndex,laserIndex);        
        rawPC(groupIndex,laserIndex,1)=sin(angle)*distance;
        rawPC(groupIndex,laserIndex,2)=cos(angle)*distance;
        rawPC(groupIndex,laserIndex,3)=groupIndex*groupDistance;        
        cntPs=cntPs+1;
    end
end

%% 截取通道的平直线段
%1、2、3、4片段分别对应走廊的底面、左侧面、顶面、右侧面
segment1=rawPC(:,[Cutpoints(1,1):Cutpoints(1,2) Cutpoints(6,1):Cutpoints(6,2)],:);
segment2=rawPC(:,Cutpoints(2,1):Cutpoints(2,2),:);
segment3=rawPC(:,[Cutpoints(3,1):Cutpoints(3,2) Cutpoints(4,1):Cutpoints(4,2)],:);
segment4=rawPC(:,Cutpoints(5,1):Cutpoints(5,2),:);

figure,hold on,
h = [ 290 950 400 300];
set(gcf,'Position',h)
plot3(rawPC(:,:,1),rawPC(:,:,2),rawPC(:,:,3),'b.');
plot3(segment1(:,:,1),segment1(:,:,2),segment1(:,:,3),'g.');
plot3(segment2(:,:,1),segment2(:,:,2),segment2(:,:,3),'g.');
plot3(segment3(:,:,1),segment3(:,:,2),segment3(:,:,3),'g.');
plot3(segment4(:,:,1),segment4(:,:,2),segment4(:,:,3),'g.');
view([0,0,1])
title('走廊截面');
xlabel('X/mm');
ylabel('Y/mm');
zlabel('Z/mm');
% save afile.txt -ascii rawPC

%% 将截取的片段拟合出对应的直线方程（二维）
line1=polyfit(segment1(:,:,1),segment1(:,:,2),1);
line2=polyfit(segment2(:,:,1),segment2(:,:,2),1);
line3=polyfit(segment3(:,:,1),segment3(:,:,2),1);
line4=polyfit(segment4(:,:,1),segment4(:,:,2),1);
minX=min(min(rawPC(:,:,1)))-200;
maxX=max(max(rawPC(:,:,1)))+200;
minY=min(min(rawPC(:,:,2)))-200;
maxY=max(max(rawPC(:,:,2)))+200;
%line1
x1=minX:maxX;
y1=line1(1)*x1+line1(2);
z1=0*x1;
%line2
y2=minY:maxY;
x2=(y2-line2(2))/line2(1);
z2=0*x2;
%line3
x3=minX:maxX;
y3=line3(1)*x3+line3(2);
z3=0*x3;
%line4
y4=minY:maxY;
x4=(y4-line4(2))/line4(1);
z4=0*x4;
plot3(x1,y1,z1,'r:','linewidth',2);
plot3(x2,y2,z2,'r:','linewidth',2);
plot3(x3,y3,z3,'r:','linewidth',2);
plot3(x4,y4,z4,'r:','linewidth',2);
axis equal,


%% 计算扫描平面（或扫描线）在世界坐标系下的相关参数
%计算出走廊的四个角点
corners=zeros(4,3);
corners(1,1:2)=GetCrossPtOf2Lines(line1(1),line1(2),line2(1),line2(2));
corners(2,1:2)=GetCrossPtOf2Lines(line2(1),line2(2),line3(1),line3(2));
corners(3,1:2)=GetCrossPtOf2Lines(line3(1),line3(2),line4(1),line4(2));
corners(4,1:2)=GetCrossPtOf2Lines(line4(1),line4(2),line1(1),line1(2));
plot3(corners(:,1),corners(:,2),corners(:,3),'rs','linewidth',3);




%% 将解算出的走廊角点保存到文件
dlmwrite([RawFilePath 'GalleryCorners.txt'],corners,...
    'precision', '%.8f','delimiter', '\t','newline', 'pc');













