
function SyntheticDataGenerator_Master(iRotationOperation,iFrameStep,iCorridorSize)
%% Simulate the function of "Calib_3LRF_PreProc"
% close all;clc;clear;

NUMBER_LRFs=3;
bMyData=0;
if bMyData
    PTS_PER_FRAME=1081;
else
    PTS_PER_FRAME=1080;
end
timePerFrame_LRF=25;  % 25ms per frame
timePerFrame_IMU=100;
RawFilePath = 'E:\SLAM\Data\20180828_sim\';
fileName0='7_0';

%% Set the size of Corridor
% assume the corridor has enough length
if iCorridorSize==1
    corridorWidth=2.0;
    corridorHeight=2.0;
elseif iCorridorSize==2
    corridorWidth=2;
    corridorHeight=4;
elseif iCorridorSize==3
    corridorWidth=4;
    corridorHeight=2;
end

%% Set trajactories (assume that the device axis is parrallel to the world axis when angs equals 0)
t=1:360;
T_traj(:,1)=t.*0 + 1;
T_traj(:,2)=t.*0 + 0;
T_traj(:,3)=t.*0 + 1;
if iCorridorSize==3
    T_traj(:,1)=t.*0 + 2;
end
if iRotationOperation==1
    % Z360 A
    ang_traj(:,1)=0*t;
    ang_traj(:,2)=0*t;
    ang_traj(:,3)=t;
elseif iRotationOperation==2
    % X45Z360 B
    ang_traj(:,1)=0*t+45;
    ang_traj(:,2)=0*t;
    ang_traj(:,3)=t;
elseif iRotationOperation==3
    % Xd45Z360 C
    ang_traj(:,1)=sin(t*4*pi/180)*45+45;
    ang_traj(:,2)=0*t;
    ang_traj(:,3)=t;
elseif iRotationOperation==4
    % Xd60Yd60Z360 D
    ang_traj(:,1)=sin(t*4*pi/180)*45+45;
    ang_traj(:,2)=sin(t*4*pi/180)*45+45;
    ang_traj(:,3)=t;
elseif iRotationOperation==5
    % Xd60Zd120  E
    ang_traj(:,1)=(360-t)*45/360;
    ang_traj(:,2)=t.*0;
    ang_traj(:,3)=sin(t*2*pi/180)*90;
elseif iRotationOperation==6
    % RandomPose F
    ang_traj=rand(length(t),3)*360;
end

ang_traj=ang_traj.*pi/180;
nGroups=size(ang_traj,1);
% figure,
% subplot(1,2,1),
% plot3(ang_traj(:,1),ang_traj(:,2),ang_traj(:,3),'.');axis equal;
% subplot(1,2,2),
% plot3(T_traj(:,1),T_traj(:,2),T_traj(:,3),'.');axis equal;


%% Set simulation poses and ini poses
ang_LRFs_sim=zeros(3,3);
ang_LRFsIni=zeros(3,3);
ang_LRFs_sim=[
    0,0,0;
    -80, 0, -35;
    80, 0,  -150;
    ];
ang_LRFsIni=[
    0,0,0;
    -90, 0, -30;
    90, 0,  -145;
    ];
T_LRFs_sim=zeros(3,3);
T_LRFsIni=zeros(3,3);
T_LRFs_sim=[
    0,0,0;
    -0.15, 0.15, -0.2;
    0.15,  0.15, -0.5;
    ];
T_LRFsIni=[
    0,0,0;
    -0.1, 0.1, -0.2;
    0.1,  0.1, -0.5;
    ];
% ang_LRFs_sim=zeros(3,3);
% ang_LRFsIni=zeros(3,3);
% ang_LRFs_sim=[
%     0,0,0;
%     -78, 0.0, -30;
%     85,  0,  -137;
%     ];
% ang_LRFsIni=[
%     0,0,0;
%     -90, 0.0, -30;
%     90,  0,  -140;
%     ];
% T_LRFs_sim=zeros(3,3);
% T_LRFsIni=zeros(3,3);
% T_LRFs_sim=[
%     0,0,0;
%     0.09,  0.1  -0.39;
%     -0.17, 0.06, -0.62;
%     ];
% T_LRFsIni=[
%     0,0,0;
%     0.1,  0.05, -0.3;
%     -0.1, 0.05, -0.6;
%     ];
ang_LRFs_sim_radian=ang_LRFs_sim.*pi/180;
R_LRFs_Simulation=zeros(3,3,3);
for i=1:3
    R_LRFs_Simulation(i,:,:)=EulerAngle2RotateMat(ang_LRFs_sim_radian(i,1),ang_LRFs_sim_radian(i,2),ang_LRFs_sim_radian(i,3),'xyz');
end
% IMU
ang_IMU=[0,0,90];
ang_IMU_radian=ang_IMU.*pi/180;
R_IMU_sim=EulerAngle2RotateMat(ang_IMU_radian(1),ang_IMU_radian(2),ang_IMU_radian(3),'xyz');


%% Generate data
poses_LRFs_R=zeros(3,3,3);
poses_LRFs_T=zeros(3,3);
PC_Raw=zeros(NUMBER_LRFs,nGroups,PTS_PER_FRAME,3);
IMU=zeros(nGroups,3,3);
for iGroup=1:iFrameStep:nGroups
    % get poses of all the sensors
    R_iGroup=EulerAngle2RotateMat(ang_traj(iGroup,1),ang_traj(iGroup,2),ang_traj(iGroup,3),'zxy');
    T_iGroup=squeeze(T_traj(iGroup,:));
    for iLRF=1:3
        [poses_LRFs_R(iLRF,:,:),poses_LRFs_T(:,iLRF)]=...
            PoseAddition_RT(squeeze(R_LRFs_Simulation(iLRF,:,:)),squeeze(T_LRFs_sim(iLRF,:))',R_iGroup,T_iGroup');
    end
    poses_LRFs_T=poses_LRFs_T';
    
    % get synthetic data (LRF)
    for iLRF=1:3
        PC_Raw(iLRF,iGroup,:,1:2)=SyntheticDataGenerator_LRF(...
            inv(squeeze(poses_LRFs_R(iLRF,:,:))),squeeze(poses_LRFs_T(iLRF,:)),corridorWidth,corridorHeight);
    end
        
    % get synthetic data (IMU)
    IMU(iGroup,:,:)=R_IMU_sim*R_iGroup;
    
    disp(['已生成第' num2str(iGroup) '组']);
end


%% Save data
% PC_Raw & IMU
fileName_PC=fileName0;
fileFullPath_Data=[RawFilePath 'Data_' fileName_PC '.mat'];
save(fileFullPath_Data,'PC_Raw','IMU');
fileFullPath_LRFsIniPos=[RawFilePath 'LRFsIniPos.mat'];
save(fileFullPath_LRFsIniPos,'ang_LRFsIni','T_LRFsIni');














