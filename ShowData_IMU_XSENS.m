%% Read Xsens data, and plot
close all;clc;clear;

% load raw data
imuData=load('D:\IMUData\20171112\XSENS_20171112_165929.txt');
% get start time and end time
startTime=imuData(1,:);
endTime=imuData(size(imuData,1),:);
% remv the first row and last row (start time and end time)
imuData(size(imuData,1),:)=[];
imuData(1,:)=[];
nImuFrames=size(imuData,1)

timeLength=endTime(1)*60*60+endTime(2)*60+endTime(3)-(startTime(1)*60*60+startTime(2)*60+startTime(3))
timeOfPerFrame=timeLength*1000/nImuFrames

step=2;
for i=1:step:size(imuData,1)-step
    roll1=-imuData(i,1)*pi/180; pitch1=-imuData(i,2)*pi/180; yaw1=-imuData(i,3)*pi/180;
    roll2=-imuData(i+step,1)*pi/180; pitch2=-imuData(i+step,2)*pi/180; yaw2=-imuData(i+step,3)*pi/180;
    R1_IMU=EularAngle2RotateMat(roll1,pitch1,yaw1,'zyx');
    R2_IMU=EularAngle2RotateMat(roll2,pitch2,yaw2,'zyx');
    % get the deltaR_IMU
    deltaR_IMU=R2_IMU*R1_IMU';
    q_IMU=RotMat2Quatern(deltaR_IMU);
    w_ang_IMU=Quatern2AngleAndAxis(q_IMU);
    axes_IMU(i,:)=w_ang_IMU(2:4);
    rotAngs(i)=w_ang_IMU(1)*180/pi;
end


figure,
h = [ 10 50 500 400];
set(gcf,'Position',h)
subplot(1,3,1),hold on,
title('Roll');
plot(imuData(:,1),'.');
subplot(1,3,2),hold on,
title('Pitch');
plot(imuData(:,2),'.');
subplot(1,3,3),hold on,
title('Yaw');
plot(imuData(:,3),'.');
% ylim([0 360]);

figure,
h = [ 500 300 900 400];
set(gcf,'Position',h)
plot(rotAngs,'.');
title(['diff angles between 2 neighbor ', num2str(step), ' frames']);





