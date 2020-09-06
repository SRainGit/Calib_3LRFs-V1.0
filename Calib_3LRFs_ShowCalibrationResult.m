% Show calibration result
close all;clc;clear

NUMBER_LRFs=3;
PTS_PER_FRAME=1081;

%% load data
tic
% set file path
disp('Loading data ...')
% RawFilePath = 'E:\SLAM\Data\LaserData\20180203\';
% fileName_RawPC=['Data_' 'UTM30LX_0_20180203_151541'];
% fileName_RawPC=['RawPC_' 'UTM30LX_0_20180203_151541'];
% fileName_RawPC=['RawPC_' '4.bag_0'];
% RawFilePath = 'E:\SLAM\Data\20180828_sim\';
% fileName_RawPC=['Data_' '7_0'];
% RawFilePath = 'E:\SLAM\Data\20180802\';
RawFilePath = 'E:\SLAM\Data\20180928\';
fileName_RawPC=['Data_' '1_0'];
iGroup = 191;

fileName_LRFsIniPos='LRFsIniPos';
fileName_LRFsCalibPos='LRFsCalibPos';
fileFullPath_RawPC=[RawFilePath fileName_RawPC '.mat'];
fileFullPath_LRFsIniPos=[RawFilePath fileName_LRFsIniPos '.mat'];
fileFullPath_LRFsCalibPos=[RawFilePath fileName_LRFsCalibPos '.mat'];

% load raw PC(point cloud) data
Data=load(fileFullPath_RawPC);
PC_Raw=Data.PC_Raw;
dataGroups=size(PC_Raw,2);

% load iniPos parameters
LRFsIniPos=load(fileFullPath_LRFsIniPos);
ang_LRFsIni=LRFsIniPos.ang_LRFsIni;
T_LRFsIni=LRFsIniPos.T_LRFsIni;

% load CalibPos parameters
LRFsCalibPos=load(fileFullPath_LRFsCalibPos);
ang_LRFs=LRFsCalibPos.ang_LRFs;
T_LRFs=LRFsCalibPos.T_LRFs;

ang_LRFsIni_radain=ang_LRFsIni.*pi/180;
ang_LRFs_radian=ang_LRFs.*pi/180;
R_LRFsIni=zeros(3,3,3);
R_LRFs=zeros(3,3,3);
for i=1:3
    R_LRFsIni(i,:,:)=EulerAngle2RotateMat(ang_LRFsIni_radain(i,1),ang_LRFsIni_radain(i,2),ang_LRFsIni_radain(i,3),'xyz');
    R_LRFs(i,:,:)=EulerAngle2RotateMat(ang_LRFs_radian(i,1),ang_LRFs_radian(i,2),ang_LRFs_radian(i,3),'xyz');
end
if isfield(LRFsCalibPos,'R_LRFs')
    R_LRFs=LRFsCalibPos.R_LRFs;
end
toc


%% choose one group to show the calibration result
fig1=figure; hold on,axis equal;
% h = [-1200 200 1100 800];
h = [80 0 1500 980];
set(gcf,'Position',h)
rotate3d on
lineColors=['r','g','b','m','c'];
LRFColors=['r','g','b'];
lineNorm=[0,0,0]; vector=[0,0,0]; corePt=[0,0,0]; basePt=[0,0,0];%basePt is for plot vector
vectorLineLength=2.000;
farthestDist=3.000;
minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
for iGroup=iGroup
% for iGroup=300:20:dataGroups
    disp(iGroup);
    cla(fig1);

    % iniPos
    subplot1=subplot(1,2,1);hold on,axis equal;
    % for each LRF
    for iLRF=1:NUMBER_LRFs
        data=squeeze(PC_Raw(iLRF,iGroup,:,:));
        PC_IniPos=(squeeze(R_LRFsIni(iLRF,:,:))*data')'+repmat(T_LRFsIni(iLRF,:),size(data,1),1);
        plot3(PC_IniPos(:,1),PC_IniPos(:,2),PC_IniPos(:,3),[LRFColors(iLRF),'.'],'LineWidth',0.1);
        minX=min(minX,min(PC_IniPos(:,1)));maxX=max(maxX,max(PC_IniPos(:,1)));
        minY=min(minY,min(PC_IniPos(:,2)));maxY=max(maxY,max(PC_IniPos(:,2)));
        minZ=min(minZ,min(PC_IniPos(:,3)));maxZ=max(maxZ,max(PC_IniPos(:,3)));
    end
    xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
    minX=max(minX,-farthestDist);maxX=min(maxX,farthestDist);
    minY=max(minY,-farthestDist);maxY=min(maxY,farthestDist);
    minZ=max(minZ,-farthestDist);maxZ=min(maxZ,farthestDist);
    xlim([minX  maxX]);    ylim([minY  maxY]);    zlim([minZ  maxZ]);
    set(subplot1,'position',[0.0 0.05 0.5 1]);
%     view([1,1,1]);
    
    
    % calibPos
    subplot2=subplot(1,2,2);hold on,axis equal;
    % for each LRF
    for iLRF=1:NUMBER_LRFs
        data=squeeze(PC_Raw(iLRF,iGroup,:,:));
        PC_CalibPos=(squeeze(R_LRFs(iLRF,:,:))*data')'+repmat(T_LRFs(iLRF,:),size(data,1),1);
        plot3(PC_CalibPos(:,1),PC_CalibPos(:,2),PC_CalibPos(:,3),[LRFColors(iLRF),'.'],'LineWidth',0.1);
        minX=min(minX,min(PC_CalibPos(:,1)));maxX=max(maxX,max(PC_CalibPos(:,1)));
        minY=min(minY,min(PC_CalibPos(:,2)));maxY=max(maxY,max(PC_CalibPos(:,2)));
        minZ=min(minZ,min(PC_CalibPos(:,3)));maxZ=max(maxZ,max(PC_CalibPos(:,3)));
    end
    xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
    minX=max(minX,-farthestDist);maxX=min(maxX,farthestDist);
    minY=max(minY,-farthestDist);maxY=min(maxY,farthestDist);
    minZ=max(minZ,-farthestDist);maxZ=min(maxZ,farthestDist);
    xlim([minX  maxX]);    ylim([minY  maxY]);    zlim([minZ  maxZ]);
%     set(subplot1,'position',[0.0 0.05 0.5 1]);
end



















