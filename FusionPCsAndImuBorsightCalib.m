%% 显示所有帧的3LRF点云
close all; clc; clear;

NUMBER_LRFs=3;
PTS_PER_FRAME=1081;

%% load data
% RawFilePath = 'D:\LaserData\20170830\';
% fileName_RawPC=['RawPC_' 'UTM30LX_0_20170830_144433'];
RawFilePath = 'D:\LaserData\20171113\';
fileName_RawPC0='UTM30LX_0_20171113_221637';
fileName_RawPC=['RawPC_' fileName_RawPC0];
fileName_LRFsIniPos='LRFsIniPos';
fileName_LRFsCalibPos='LRFsCalibPos';
% fileName_AllFramesPose=['AllFramesPose_' 'UTM30LX_0_20170830_144433'];
fileName_AllFramesPose=['AllFramesPose_' fileName_RawPC0];
fileFullPath_RawPC=[RawFilePath fileName_RawPC '.mat'];
fileFullPath_LRFsIniPos=[RawFilePath fileName_LRFsIniPos '.mat'];
fileFullPath_LRFsCalibPos=[RawFilePath fileName_LRFsCalibPos '.mat'];
fileFullPath_AllFramesPose=[RawFilePath fileName_AllFramesPose '.mat'];
% load IMU data
imuDataPath='D:\IMUData\20171113\';
imuDataFileName='XSENS_20171113_210006';
imuData=load([imuDataPath imuDataFileName '.txt']);
% get start time and end time
startTime=imuData(1,:);
endTime=imuData(size(imuData,1),:);
% remv the first row and last row (start time and end time)
imuData(size(imuData,1),:)=[];
imuData(1,:)=[];
nImuFrames=size(imuData,1);

% load raw PC(point cloud) data
PC_Raw=load(fileFullPath_RawPC);
PC_Raw=PC_Raw.PC_Raw;
nDataGroups=size(PC_Raw,2);
% load iniPos parameters
LRFsIniPos=load(fileFullPath_LRFsIniPos);
LRFsCalibPos=load(fileFullPath_LRFsCalibPos);
% ang_LRFs=LRFsIniPos.ang_LRFsIni; % use the ini Pos
% T_LRFs=LRFsIniPos.T_LRFsIni;
ang_LRFs=LRFsCalibPos.ang_LRFs; % use the calib Pos
T_LRFs=LRFsCalibPos.T_LRFs;
ang_LRFs_radian=ang_LRFs.*pi/180;
R_LRFs=zeros(3,3,3);
for i=1:3
    R_LRFs(i,:,:)=EularAngle2RotateMat(ang_LRFs_radian(i,1),ang_LRFs_radian(i,2),ang_LRFs_radian(i,3),'xyz');
end
% load pos data
AllPoses=load(fileFullPath_AllFramesPose);

% extract variables
R_D2Ws_Ori=AllPoses.R_D2Ws;
T_D2Ws_Ori=AllPoses.T_D2Ws;
R_D2Ws_deNull=R_D2Ws_Ori;
T_D2Ws_deNull=T_D2Ws_Ori;
nAllGroups=size(T_D2Ws_Ori,1);

%% preprocess data
% initialize the index array
indexOfGroups=zeros(nAllGroups,1);
for i=1:nAllGroups
    indexOfGroups(i)=i;
end

% delete unavailiable elements, and get the number of available elements
for iGroup=nAllGroups:-1:1
    if norm(T_D2Ws_deNull(iGroup,:))==0
        R_D2Ws_deNull(iGroup,:,:)=[];
        T_D2Ws_deNull(iGroup,:)=[];
        indexOfGroups(iGroup)=[];
    end
end
R_D2Ws=R_D2Ws_deNull;
T_D2Ws=T_D2Ws_deNull;
nGroups=length(indexOfGroups); % nGroups = nAvailableGroups(to be shorter)

% get the changes between neibor Ts
diffT_deNull(2:nGroups,:)=T_D2Ws_deNull(2:nGroups,:)-T_D2Ws_deNull(1:nGroups-1,:);
% invert the R to Eular angles, ang get the changes between beibor Rs
eular_D2W_deNull=zeros(nGroups,3);
for iGroup=1:nGroups
    eular_D2W_deNull(iGroup,:)=RotateMat2EularAngle_XYZ(squeeze(R_D2Ws_deNull(iGroup,:,:)));
end
eular_D2W=eular_D2W_deNull;
diffEular_deNull(2:nGroups,:)=eular_D2W_deNull(2:nGroups,:)-eular_D2W_deNull(1:nGroups-1,:);

%% Smoothing the Eular angles between neighbor frames
% ini all deltaRs (prepare the all possible for deltaR)
deltaRs=zeros(3*2*4,3,3); % 3x3 matrix(frame), 3 is "3 possibles for x axis", 2 is for "2 possible for y axis", 4 is for 2(-1 or 1)*2(-1 or 1)
cntDeltaRs=0;
for iX=1:3
    for iiX=-1:2:1
        for iY=1:3
            if iX~=iY
                for iiY=-1:2:1
                    deltaR_X=zeros(1,3);
                    deltaR_Y=zeros(1,3);
                    deltaR_X(iX)=iiX;
                    deltaR_Y(iY)=iiY;
                    cntDeltaRs=cntDeltaRs+1;
                    deltaR_Z=cross(deltaR_X,deltaR_Y);
                    deltaRs(cntDeltaRs,:,:)=[deltaR_X;deltaR_Y;deltaR_Z];
                end
            end
        end
    end
end
% smoothing
Threshold_close=60*pi/180;
cntNeedToChange=0; cntChangePos=0;
for iGroup=2:nGroups
    sumAng0=0;
    for i=1:3
        sumAng0=sumAng0+acos(dot(squeeze(R_D2Ws(iGroup,i,:)),squeeze(R_D2Ws(iGroup-1,i,:))));
    end
    if sumAng0>Threshold_close
        %     if sum(abs(abs(eular_D2W(iGroup,:))-abs(eular_D2W(iGroup-1,:))))>Threshold_ang
        cntNeedToChange=cntNeedToChange+1;
        isChangeSuccess=0;
        for iDeltaR=1:cntDeltaRs
            R=squeeze(deltaRs(iDeltaR,:,:))*squeeze(R_D2Ws(iGroup,:,:));
            sumAng1=0;
            for i=1:3
                sumAng1=sumAng1+acos(dot(R(i,:),squeeze(R_D2Ws(iGroup-1,i,:))));
            end
            if sumAng1<Threshold_close
                cntChangePos=cntChangePos+1;
                R_D2Ws(iGroup,:,:)=R;
                eular_D2W(iGroup,:)=RotateMat2EularAngle_XYZ(R);
                T_D2Ws(iGroup,:)=squeeze(deltaRs(iDeltaR,:,:))*T_D2Ws(iGroup,:)';
                isChangeSuccess=1;
                break;
            end
        end
        if isChangeSuccess~=1
            disp('one smooth failure!!!');
        end
    end
end
% update diffT & diffAng
diffT(2:nGroups,:)=T_D2Ws(2:nGroups,:)-T_D2Ws(1:nGroups-1,:);
diffAng(2:nGroups,:)=eular_D2W(2:nGroups,:)-eular_D2W(1:nGroups-1,:);



%% IMU boresight calibration
% 1. Get all the axes
sampleStep=40;
sampleCount=200;
cntDeltas=0;
iGroups1=randi(nGroups-2*sampleStep-1,1,sampleCount)+sampleStep+1;
for iGroup1=iGroups1
    aRandNum=rand*2-1;
    iGroup2=iGroup1+floor(sampleStep*aRandNum);
    if iGroup2==iGroup1
        continue;
    end
    cntDeltas=cntDeltas+1;
    % ori indexes for LRFs
    oriIndex1=indexOfGroups(iGroup1);
    oriIndex2=indexOfGroups(iGroup2);
    % get the deltaR_LRFs
    R1_LRFs=squeeze(R_D2Ws(iGroup1,:,:));
    R2_LRFs=squeeze(R_D2Ws(iGroup2,:,:));
    deltaR_LRFs=R2_LRFs'*R1_LRFs;
    deltaR_LRFs=deltaR_LRFs'; % I don't know WHY do this!!!!!!!!!!!!!!!!!!
    q_LRF=RotMat2Quatern(deltaR_LRFs');
    axis_LRF=Quatern2AngleAndAxis(q_LRF);
    axes_LRF(cntDeltas,:)=axis_LRF(2:4);
    rotAngs(cntDeltas,1)=axis_LRF(1);
    
    % find the closet imuIndex for index1 and index2
    imuIndex1=floor(oriIndex1*nImuFrames/nAllGroups);
    imuIndex2=floor(oriIndex2*nImuFrames/nAllGroups);
    roll1=-imuData(imuIndex1,1)*pi/180;        pitch1=-imuData(imuIndex1,2)*pi/180;        yaw1=-imuData(imuIndex1,3)*pi/180;
    roll2=-imuData(imuIndex2,1)*pi/180;        pitch2=-imuData(imuIndex2,2)*pi/180;        yaw2=-imuData(imuIndex2,3)*pi/180;
    R1_IMU=EularAngle2RotateMat(roll1,pitch1,yaw1,'zyx');
    R2_IMU=EularAngle2RotateMat(roll2,pitch2,yaw2,'zyx');
    % get the deltaR_IMU
    deltaR_IMU=R2_IMU*R1_IMU';
    q_IMU=RotMat2Quatern(deltaR_IMU);
    axis_IMU=Quatern2AngleAndAxis(q_IMU);
    axes_IMU(cntDeltas,:)=axis_IMU(2:4);
    rotAngs(cntDeltas,2)=axis_IMU(1);
    rotAngs(cntDeltas,3)=axis_LRF(1)-axis_IMU(1);
end

% 2. The first registration
axes_LRF=axes_LRF';
axes_IMU=axes_IMU';
R=RotationRegistrationBySVD(axes_LRF,axes_IMU,cntDeltas);
axes_IMU_=R*axes_IMU;
% 3. Remove the "outliers"
distThreshold=0.15;
cntDeltas=0;
for i=1:size(axes_LRF,2)
    if norm(axes_LRF(:,i)-axes_IMU_(:,i))<distThreshold
        cntDeltas=cntDeltas+1;
        axes_LRF__(:,cntDeltas)=axes_LRF(:,i);
        axes_IMU__(:,cntDeltas)=axes_IMU(:,i);
    end
end
% 4. The second registration
R=RotationRegistrationBySVD(axes_LRF__,axes_IMU__,cntDeltas);
axes_IMU__=R*axes_IMU__;

% 5. show the registration result
figure,
h = [0 0 1500 980];
set(gcf,'Position',h)
% Ori pairs
subplot(1,3,1)
hold on, rotate3d on;
axis equal;
for i=1:size(axes_LRF,2)
    oneMatch=[axes_LRF(:,i) axes_IMU(:,i)];
    line(oneMatch(1,:),oneMatch(2,:),oneMatch(3,:),'color','k','Marker','.','LineStyle','-')
end
plot3(axes_LRF(1,:),axes_LRF(2,:),axes_LRF(3,:),'ro');
plot3(axes_IMU(1,:),axes_IMU(2,:),axes_IMU(3,:),'go');
% The first registration result
subplot(1,3,2)
hold on, rotate3d on;
axis equal;
for i=1:size(axes_IMU_,2)
    oneMatch=[axes_LRF(:,i) axes_IMU_(:,i)];
    line(oneMatch(1,:),oneMatch(2,:),oneMatch(3,:),'color','k','Marker','.','LineStyle','-')
end
plot3(axes_LRF(1,:),axes_LRF(2,:),axes_LRF(3,:),'ro');
plot3(axes_IMU_(1,:),axes_IMU_(2,:),axes_IMU_(3,:),'go');
% The second registration result
subplot(1,3,3)
hold on, rotate3d on;
axis equal;
for i=1:size(axes_LRF__,2)
    oneMatch=[axes_LRF__(:,i) axes_IMU__(:,i)];
    line(oneMatch(1,:),oneMatch(2,:),oneMatch(3,:),'color','k','Marker','.','LineStyle','-')
end
plot3(axes_LRF__(1,:),axes_LRF__(2,:),axes_LRF__(3,:),'ro');
plot3(axes_IMU__(1,:),axes_IMU__(2,:),axes_IMU__(3,:),'go');

% return;

%% show device frame and IMU frame after boresight calib
RotateMat2EularAngle_XYZ(R)
f_finalPoses=figure;
h = [-900 700 800 600];
% set(gcf,'Position',h)
hold on, rotate3d on;
% xlim([-20  20]);    ylim([-20  20]);    zlim([-20  20]);
PlotAxes_RT('r','-',[0,0,0],eye(3),10,3);
% PlotAxes_TR('g','-',[0,0,0],R,10,3);
PlotAxes_TR('b','-',[0,0,0],R',10,3);
axis equal;

% save the IMU calib result
fileFullPath_IMUPose=[imuDataPath 'ImuPose.mat'];
save(fileFullPath_IMUPose,'R');

% return

%% show all the poses by translation and eular angles
% show the changes of T
figure,
h = [-1500 100 1500 980];
set(gcf,'Position',h);

subplot(2,2,1), hold on, view([0 1 0]);
axis equal;    rotate3d on;
plot3(T_D2Ws_deNull(:,1),T_D2Ws_deNull(:,2),T_D2Ws_deNull(:,3),'.');
quiver3(T_D2Ws_deNull(1:nGroups-1,1),0*T_D2Ws_deNull(1:nGroups-1,2),T_D2Ws_deNull(1:nGroups-1,3),...
    diffT_deNull(2:nGroups,1),diffT_deNull(2:nGroups,2),diffT_deNull(2:nGroups,3),15,'-');
xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
title('Pos-T: Before Smoothing');

% show the changes of R
subplot(2,2,2), hold on;
axis equal;    rotate3d on;
plot3(eular_D2W_deNull(:,1),eular_D2W_deNull(:,2),eular_D2W_deNull(:,3),'.');
quiver3(eular_D2W_deNull(1:nGroups-1,1),eular_D2W_deNull(1:nGroups-1,2),eular_D2W_deNull(1:nGroups-1,3),...
    diffEular_deNull(2:nGroups,1),diffEular_deNull(2:nGroups,2),diffEular_deNull(2:nGroups,3),15,'-');
xlabel('X/degree'); ylabel('Y/degree'); zlabel('Z/degree');
title('Pos-Ang: Before Smoothing');

subplot(2,2,3), hold on, view([0 1 0]);
axis equal;    rotate3d on;
plot3(T_D2Ws(:,1),T_D2Ws(:,2),T_D2Ws(:,3),'.');
quiver3(T_D2Ws(1:nGroups-1,1),0*T_D2Ws(1:nGroups-1,2),T_D2Ws(1:nGroups-1,3),...
    diffT(2:nGroups,1),diffT(2:nGroups,2),diffT(2:nGroups,3),5,'-');
xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
title('Pos-T: After Smoothing');

% show the changes of R
subplot(2,2,4), hold on;
axis equal;    rotate3d on;
plot3(eular_D2W(:,1),eular_D2W(:,2),eular_D2W(:,3),'.');
quiver3(eular_D2W(1:nGroups-1,1),eular_D2W(1:nGroups-1,2),eular_D2W(1:nGroups-1,3),...
    diffAng(2:nGroups,1),diffAng(2:nGroups,2),diffAng(2:nGroups,3),15,'-');
xlabel('X/degree'); ylabel('Y/degree'); zlabel('Z/degree');
title('Pos-Ang: After Smoothing');


%% show the fusion PC
figure,
h = [-1500 50 1500 980];
set(gcf,'Position',h)
hold on, rotate3d on;
lineColors=['r','g','b','m','c'];
LRFColors=['c','g','b'];
axis equal;
xlim([-5000  5000]);    ylim([-5000  5000]);    zlim([-5000  5000]);
for iGroup=1:nGroups
    oriIndex=indexOfGroups(iGroup);
    for iLRF=1:NUMBER_LRFs
        data=squeeze(PC_Raw(iLRF,oriIndex,:,:));
        
        R=squeeze(R_D2Ws(iGroup,:,:))*squeeze(R_LRFs(iLRF,:,:));
        T=(squeeze(R_D2Ws(iGroup,:,:))*T_LRFs(iLRF,:)'+T_D2Ws(iGroup,:)')';
        PC=(R*data')'+repmat(T,size(data,1),1);
        plot3(PC(:,1),PC(:,2),PC(:,3),[LRFColors(iLRF),'.'],'LineWidth',0.1);
    end
end
xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
xlim([-5000  5000]);    ylim([-5000  5000]);    zlim([-5000  5000]);
axis off;













