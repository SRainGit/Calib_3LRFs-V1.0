% Convert the calib result(from YinDeyu) to the needed result(to ChenShoubin)
close all;clc;clear

%% load data
LRFsFilePath = 'D:\LaserData\20171113\';
LRFsCalibPos=load([LRFsFilePath 'LRFsCalibPos.mat']);
ImuDataPath='D:\IMUData\20171113\';
ImuPose=load([ImuDataPath 'ImuPose.mat']);

%% extract LRFs Pose
ang_LRFs=LRFsCalibPos.ang_LRFs;
T_LRFs=LRFsCalibPos.T_LRFs;
ang_LRFs_radian=ang_LRFs.*pi/180;
R_LRFs=zeros(3,3,3);
for i=1:3
    R_LRFs(i,:,:)=EularAngle2RotateMat(ang_LRFs_radian(i,1),ang_LRFs_radian(i,2),ang_LRFs_radian(i,3),'xyz');
end

%% extract IMU Pose (TR, translation before rotation)
R_ImuToLRF1=ImuPose.R;
T_LRF1ToImu=[140,-115,-810];


%% Ori relationships
R_LRF2ToLRF1=squeeze( R_LRFs(2,:,:));
T_LRF1ToLRF2=T_LRFs(2,:);
R_LRF3ToLRF1=squeeze( R_LRFs(3,:,:));
T_LRF1ToLRF3=T_LRFs(3,:);

%% converted-TR, take IMU as base
R_Z__90=EularAngle2RotateMat(0,0,-90*pi/180,'xyz');
% Imu to LRF1
T_ImuToLRF1=-(R_ImuToLRF1'*T_LRF1ToImu');
R_ImuToLRF1=R_ImuToLRF1;
% Imu to LRF2
T_ImuToLRF2=R_ImuToLRF1'*(R_ImuToLRF1*T_ImuToLRF1+T_LRF1ToLRF2');
R_ImuToLRF2=R_LRF2ToLRF1'*R_ImuToLRF1;
% Imu to LRF3
T_ImuToLRF3=R_ImuToLRF1'*(R_ImuToLRF1*T_ImuToLRF1+T_LRF1ToLRF3');
R_ImuToLRF3=R_LRF3ToLRF1'*R_ImuToLRF1;

% LRF1
R_ImuToLRF1=R_Z__90*R_ImuToLRF1;
eulars_ImuToLRF1=RotateMat2EularAngle_ZYX(R_ImuToLRF1);
R_ImuToLRF1=EularAngle2RotateMat(eulars_ImuToLRF1(1),eulars_ImuToLRF1(2),eulars_ImuToLRF1(3),'zyx');
eulars_ImuToLRF1=-eulars_ImuToLRF1
T_ImuToLRF1=T_ImuToLRF1'*0.001
% LRF2
R_ImuToLRF2=R_Z__90*R_ImuToLRF2;
eulars_ImuToLRF2=RotateMat2EularAngle_ZYX(R_ImuToLRF2);
R_ImuToLRF2=EularAngle2RotateMat(eulars_ImuToLRF2(1),eulars_ImuToLRF2(2),eulars_ImuToLRF2(3),'zyx');
eulars_ImuToLRF2=-eulars_ImuToLRF2
T_ImuToLRF2=T_ImuToLRF2'*0.001
% LRF3
R_ImuToLRF3=R_Z__90*R_ImuToLRF3;
eulars_ImuToLRF3=RotateMat2EularAngle_ZYX(R_ImuToLRF3);
R_ImuToLRF3=EularAngle2RotateMat(eulars_ImuToLRF3(1),eulars_ImuToLRF3(2),eulars_ImuToLRF3(3),'zyx');
eulars_ImuToLRF3=-eulars_ImuToLRF3
T_ImuToLRF3=T_ImuToLRF3'*0.001

%% show frames
figure, hold on; rotate3d on;
h = [ -900 300 800 600];
set(gcf,'Position',h)
axisLength_Body=100;
PlotAxes_TR('k','-',[0,0,0],eye(3),axisLength_Body,3);
PlotAxes_TR('r','-',T_ImuToLRF1',R_ImuToLRF1,axisLength_Body,3);
PlotAxes_TR('g','-',T_ImuToLRF2',R_ImuToLRF2,axisLength_Body,3);
PlotAxes_TR('b','-',T_ImuToLRF3',R_ImuToLRF3,axisLength_Body,3);
axis equal;

xlabel('X/mm');
ylabel('Y/mm');
zlabel('Z/mm');
view([1 1 1]);

%放置调试设备的小平台
x=-20:380;
y=-20:380;
[x,y]=meshgrid(x,y);
z=x*0-20;
surf(x,y,z);
alpha(.5);%透明度
shading interp

%圆柱
R_Cylinder=eye(3,3);
CylinderCorePt=[-20 200 500];
CylinderHeight=800;
CylinderRadius=50;
% R_Cylinder=EularAngle2RotateMat(30,0,0,'xyz');
PlotCylinder(CylinderCorePt,CylinderRadius,CylinderHeight,R_Cylinder,'b',1);

axis off
legend('Body','IMU','LS1','LS2','LS3','Platform','Shaft')

return







