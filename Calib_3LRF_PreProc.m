%% Preprocess data
% 1.load data
% 2.frames alignment (including IMU data)
% 3.generate 2D LRFs point clouds
% 4.save the LRFs data, IMU data and ini Poses to '.mat' file
close all;clc;clear;

NUMBER_LRFs=3;

bMyData=0;
% bMyData=1;
bHaveIMUData=1;


% RawFilePath = 'E:\SLAM\Data\LaserData\20180203\';
% fileName0='UTM30LX_0_20180203_151541';
% RawFilePath = 'D:\LaserData\20180629\data\';
% fileName0='4.bag_0';
% imuDataPath='D:\LaserData\20180629\data\';
% imuDataFileName='4.bag_imu';
% RawFilePath = 'E:\SLAM\Data\20180802\';
% RawFilePath = 'E:\SLAM\Matlab\Calib_3LRFs-V1.0\Data\20180802\';
RawFilePath = 'E:\SLAM\Matlab\Calib_3LRFs-V1.0\Data\20180928\';
fileName0='1_0';
imuDataFileName='1_imu';
imuDataPath=RawFilePath;

% guess - the pose angle and translation of three LRFs
ang_LRFsIni=[
    0,0,0;
    90,0,-155;
    90, 0, 155;   
    ];
% From LRF0 to LRFi
T_LRFsIni=[
    0,0,0;
    -0.1,0.15,-1.5;
    0.1, 0.15,-1.2;
    ];
% ang_LRFsIni=[
%     0,0,0;
%     -80, 0.0, -30;
%     90,   1.0,-135;
%     ];
% % From LRF0 to LRFi
% T_LRFsIni=[
%     0,0,0;
%     0.1, 0.05,-0.4;
%     -0.1,0.05,-0.6;
%     ];
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


if bMyData
    PTS_PER_FRAME=1081;
else
    PTS_PER_FRAME=1080;
end
timePerFrame_LRF=25;  % 25ms per frame
timePerFrame_IMU=100;


%% 解算3LRF扫描得到的点云
%文件路径
fileName1=fileName0; fileName2=fileName1;
if bMyData
    fileName1(9)='1';
    fileName2(9)='2';
% else
%     fileName1(7)='1';
%     fileName2(7)='2';
else
    fileName1(3)='1';
    fileName2(3)='2';
end
RawFileFullPath0=[RawFilePath fileName0 '.txt'];
RawFileFullPath1=[RawFilePath fileName1 '.txt'];
RawFileFullPath2=[RawFilePath fileName2 '.txt'];
TimestampFileFullPath0=[RawFilePath fileName0 '.timestamp'];
TimestampFileFullPath1=[RawFilePath fileName1 '.timestamp'];
TimestampFileFullPath2=[RawFilePath fileName2 '.timestamp'];
if bHaveIMUData==1
    % load IMU data
    imuData=load([imuDataPath imuDataFileName '.txt']);
    % get start time and end time
    startTime=imuData(1,:);
    endTime=imuData(size(imuData,1),:);
    % remv the first row and last row (start time and end time)
    timestamps_IMU=imuData(:,1);  % save the timestamp
    nImuFrames=size(imuData,1);
    imuData=[imuData(:,5),imuData(:,2),imuData(:,3),imuData(:,4)];  % remv the timestamp
    % imuData=[imuData(:,2),imuData(:,3),imuData(:,4),imuData(:,5)];  % remv the timestamp
end

% load raw data
tic
rawData0=load(RawFileFullPath0);
rawData1=load(RawFileFullPath1);
rawData2=load(RawFileFullPath2);
timestamps0=load(TimestampFileFullPath0);
timestamps1=load(TimestampFileFullPath1);
timestamps2=load(TimestampFileFullPath2);
toc

%% frame alignment according to timestamp
maxTimestampDiff=25/2;  % 25ms/2
iSearchRadius_LRF=40;
iSearchRadius_IMU=1000;
cntValidFrame=0;
alpha=timePerFrame_IMU/timePerFrame_LRF;
lastIMUFrame=1;
for iFrame0=1:size(rawData0,1)
    t0=timestamps0(iFrame0);
    
    % search the neighbor frame of LRF1
    iSuccess=0;
    for iOffset=0:iSearchRadius_LRF
        if iSuccess==1
            break;
        end
        for iDirection=-1:2:1
            if iSuccess==1
                break;
            end
            iFrame1=iDirection*iOffset+iFrame0;
            if iFrame1<1 || iFrame1>size(rawData1,1)
                continue;
            end
            t1=timestamps1(iFrame1);
            if abs(t1-t0)<maxTimestampDiff
                iSuccess=1;
            end
        end
    end
    if iSuccess==0
        continue;
    end
    
    % search the neighbor frame of LRF1
    iSuccess=0;
    for iOffset=0:iSearchRadius_LRF
        if iSuccess==1
            break;
        end
        for iDirection=-1:2:1
            if iSuccess==1
                break;
            end
            iFrame2=iDirection*iOffset+iFrame0;
            if iFrame2<1 || iFrame2>size(rawData2,1)
                continue;
            end
            t2=timestamps2(iFrame2);
            if abs(t2-t0)<maxTimestampDiff
                iSuccess=1;
                break;
            end
        end
    end
    if iSuccess==0
        continue;
    end
    
    if bHaveIMUData==1
        % search the neighbor frame of IMU
        iSuccess=0;
        for iOffset=0:iSearchRadius_IMU
            if iSuccess==1
                break;
            end
            for iDirection=-1:2:1
                if iSuccess==1
                    break;
                end
                iFrame_IMU=iDirection*iOffset+lastIMUFrame+alpha;
                if iFrame_IMU<1 || iFrame_IMU>size(imuData,1)
                    continue;
                end
                t3=timestamps_IMU(iFrame_IMU);
                if abs(t3-t0)<maxTimestampDiff
                    iSuccess=1;
                    lastIMUFrame=iFrame_IMU;
                    break;
                end
            end
        end
        if iSuccess==0
            continue;
        end
    end
    
    % recording
    cntValidFrame=cntValidFrame+1;
    rawData0_(cntValidFrame,:)=rawData0(iFrame0,:);
    rawData1_(cntValidFrame,:)=rawData1(iFrame1,:);
    rawData2_(cntValidFrame,:)=rawData2(iFrame2,:);  
    if bHaveIMUData==1
        imuData_(cntValidFrame,:)=imuData(iFrame_IMU,:);
    end
end
if size(rawData0,1)-cntValidFrame>50
    disp('Warning, please check.');
    return;
end
if bHaveIMUData==1
    IMU=imuData_;
end


% fusion 3 Datas to one matrix
rawData(1,:,:)=rawData0_;
rawData(2,:,:)=rawData1_;
rawData(3,:,:)=rawData2_;
rawData(isnan(rawData)==1)=0;
rawData(rawData==inf)=0;


%% Device frame
% the orientation equals to the orientation of LRF0
R_D=eye(3,3);
T_D=[0,-0.130,-1.000];

% IMU frame
ang_Z_D2IMU=-pi/2;
ang_X_D2IMU=-pi;
R_D2IMU=EulerAngle2RotateMat(ang_X_D2IMU,0,ang_Z_D2IMU,'zxy');
T_IMU=[0.100 0.020 -1.050];

ang_LRFsIni_radian=ang_LRFsIni.*pi/180;
R_LRFsIni=zeros(3,3,3);
for i=1:3
    R_LRFsIni(i,:,:)=EulerAngle2RotateMat(ang_LRFsIni_radian(i,1),ang_LRFsIni_radian(i,2),ang_LRFsIni_radian(i,3),'xyz');
end


%% generate 2D point clouds
PC_Raw=zeros(NUMBER_LRFs,cntValidFrame,PTS_PER_FRAME,3);
% PC_IniPos=zeros(NUMBER_LRFs,cntValidFrame,PTS_PER_FRAME,3);
for iLRFs=1:NUMBER_LRFs
    for row=1:cntValidFrame
        PC_Raw(iLRFs,row,:,1:2)=OneFrameRawData2Pts_2D_UTM30LX(rawData(iLRFs,row,:));
%         PC_IniPos(iLRFs,row,:,:)=...
%             (squeeze(R_LRFsIni(iLRFs,:,:))*squeeze(PC_Raw(1,row,:,:))')'+repmat(squeeze(T_LRFsIni(iLRFs,:)),size(PC_Raw,3),1);
    end
end
toc


%% save data to file
if bMyData
    PC_Raw=PC_Raw.*0.001; % change the unit to milimeter
end
tic
% LRF datas
fileName_PC=fileName0;
fileFullPath_Data=[RawFilePath 'Data_' fileName_PC '.mat'];
fileFullPath_IniPosPC=[RawFilePath 'IniPosPC_' fileName_PC '.mat'];
if bHaveIMUData==1
    save(fileFullPath_Data,'PC_Raw','IMU');
else
    save(fileFullPath_Data,'PC_Raw');
end
    
% angles and Ts
fileFullPath_LRFsIniPos=[RawFilePath 'LRFsIniPos.mat'];
save(fileFullPath_LRFsIniPos,'ang_LRFsIni','T_LRFsIni');
toc





