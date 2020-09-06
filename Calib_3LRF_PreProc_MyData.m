%% 显示3LRF点云
close all
clc
clear

NUMBER_LRFs=3;
PTS_PER_FRAME=1081;

RawFilePath = 'E:\SLAM\Data\LaserData\20180203\';
fileName0='UTM30LX_0_20180203_151541';
% RawFilePath = 'D:\LaserData\20171113\';
% fileName0='UTM30LX_0_20171113_210018';

% % the pose angle and translation of three LRFs
% ang_LRFsIni=[
%     0,0,0;
%     -87,-2.8,-29;
%     89,0.2,-138;
%     ];
% % From LRF0 to LRFi
% T_LRFsIni=[
%     0,0,0;
%     80,35,-270;
%     -120,35,-470;
%     ];

% % the pose angle and translation of three LRFs
% ang_LRFsIni=[
%     0,0,0;
%     -90,-10,-40;
%     90,0,-140;
%     ];
% % From LRF0 to LRFi
% T_LRFsIni=[
%     0,0,0;
%     100,50,-300;
%     -100,50,-400;
%     ];

ang_LRFsIni=[
    0,0,0;
    -80, 0.0, -30;
    90,   0.0,-135;
    ];
% From LRF0 to LRFi
T_LRFsIni=[
    0,0,0;
    0.1, 0.05,-0.4;
    -0.1,0.05,-0.6;
    ];
% % calibed - the pose angle and translation of three LRFs
% ang_LRFsIni=[
%     0,0,0;
%     -80.5, 0.45, -30;
%     88,1.45,-136.8;
%     ];
% % From LRF0 to LRFi
% T_LRFsIni=[
%     0,0,0;
%     0.087, 0.069,-0.394;
%     -0.134,0.025,-0.622;
%     ];


%% 解算3LRF扫描得到的点云
%文件路径
fileName1=fileName0; fileName2=fileName1;
fileName1(9)='1';
fileName2(9)='2';
RawFileFullPath0=[RawFilePath fileName0 '.txt'];
RawFileFullPath1=[RawFilePath fileName1 '.txt'];
RawFileFullPath2=[RawFilePath fileName2 '.txt'];

%提取原始数据
tic
rawData0=load(RawFileFullPath0);
rawData1=load(RawFileFullPath1);
rawData2=load(RawFileFullPath2);
toc

%为了方便，去最小组数作为最终组数，并截去多余的组
tic
dataGroups=min(min(size(rawData0,1),size(rawData1,1)),size(rawData2,1));
nRemvGroups0=size(rawData0,1)-dataGroups
nRemvGroups1=size(rawData1,1)-dataGroups
nRemvGroups2=size(rawData2,1)-dataGroups
rawData0(dataGroups+1:size(rawData0,1),:)=[];
rawData1(dataGroups+1:size(rawData1,1),:)=[];
rawData2(dataGroups+1:size(rawData2,1),:)=[];

%将三组数据合并到同一变量数组下
rawData(1,:,:)=rawData0;
rawData(2,:,:)=rawData1;
rawData(3,:,:)=rawData2;


%% 制作背包SLAM中各个传感器的初始坐标系，都以LRF0为参考
%设备坐标系，轴向等于LRF0
R_D=eye(3,3);
T_D=[0,-130,-1000];

%IMU坐标系
ang_Z_D2IMU=-pi/2;
ang_X_D2IMU=-pi;
R_D2IMU=EulerAngle2RotateMat(ang_X_D2IMU,0,ang_Z_D2IMU,'zxy');
T_IMU=[100 20 -1050];

ang_LRFsIni_radian=ang_LRFsIni.*pi/180;
R_LRFsIni=zeros(3,3,3);
for i=1:3
    R_LRFsIni(i,:,:)=EulerAngle2RotateMat(ang_LRFsIni_radian(i,1),ang_LRFsIni_radian(i,2),ang_LRFsIni_radian(i,3),'xyz');
end

%解算原始三维点云，并将传感器坐标系下的点云变换到Device坐标系下，注意旋转变换矩阵的逆和平移矩阵直接使用
PC_Raw=zeros(NUMBER_LRFs,dataGroups,PTS_PER_FRAME,3);
PC_IniPos=zeros(NUMBER_LRFs,dataGroups,PTS_PER_FRAME,3);
for iLRFs=1:NUMBER_LRFs
    for row=1:dataGroups
        PC_Raw(iLRFs,row,:,1:2)=OneFrameRawData2Pts_2D_UTM30LX(rawData(iLRFs,row,:));
        PC_IniPos(iLRFs,row,:,:)=...
            (squeeze(R_LRFsIni(iLRFs,:,:))*squeeze(PC_Raw(1,row,:,:))')'+repmat(squeeze(T_LRFsIni(iLRFs,:)),size(PC_Raw,3),1);
    end
end
toc


%% 保存解算好的数据到本地文件
tic
%LRF的数据
fileName_PC=fileName0;
fileFullPath_RawPC=[RawFilePath 'RawPC_' fileName_PC '.mat'];
fileFullPath_IniPosPC=[RawFilePath 'IniPosPC_' fileName_PC '.mat'];
save(fileFullPath_RawPC,'PC_Raw');
%旋转平移矩阵
fileFullPath_LRFsIniPos=[RawFilePath 'LRFsIniPos.mat'];
save(fileFullPath_LRFsIniPos,'ang_LRFsIni','T_LRFsIni');
toc





