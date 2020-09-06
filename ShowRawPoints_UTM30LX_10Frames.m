%% 显示激光点云
close all
clc
clear


startLaserIndex=0;
midLaserIndex=540;
endLaserIndex=1080;
angStep_degree=270/1080;
angStep_radian=angStep_degree*pi/180;
ScannerSpeed = 22;

startGroup=001;
endGroup=010;
groupDistance=100;
RawFilePath = 'D:\LaserData\20170723\1\';

rawData=zeros(endGroup-startGroup+1,endLaserIndex-startLaserIndex+1);

oneFile='';
for groupIndex=startGroup:endGroup
    oneFile=[RawFilePath,'data_',num2str(groupIndex,'%03d'),'.csv'];
    rawData(groupIndex,:)=load(oneFile);
end
    
rawPC=zeros(size(rawData,1)*size(rawData,2),3);
distance=0;
cntPs=0;
for groupIndex=startGroup:endGroup
    for laserIndex=1:size(rawData,2)
        angle=(laserIndex-midLaserIndex)*angStep_radian;
        distance=rawData(groupIndex,laserIndex);
        
        x=-sin(angle)*distance; % 注意激光光束扫描方向可能不是对的
        y=cos(angle)*distance;
        z=groupIndex*groupDistance;
        
        cntPs=cntPs+1;
        rawPC(cntPs,1)=x;
        rawPC(cntPs,2)=y;
        rawPC(cntPs,3)=z;
    end
end

figure,hold on,axis equal,
h = [ 60 150 500 400];
set(gcf,'Position',h)
plot3(rawPC(:,1),rawPC(:,2),rawPC(:,3),'.');
view([0,0,1])
title('RawPC');
xlabel('X/mm');
ylabel('Y/mm');
zlabel('Z/mm');hold off;
% save afile.txt -ascii rawPC



