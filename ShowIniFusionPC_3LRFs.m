% Show calibration result
close all;clc;clear

NUMBER_LRFs=3;
PTS_PER_FRAME=1081;

%% load data
tic
% set file path
disp('Loading data ...')
RawFilePath = 'D:\LaserData\20180130\';
fileName_RawPC='UTM30LX_0_20180130_145106';
% RawFilePath = 'D:\LaserData\20180203\';
% fileName_RawPC='UTM30LX_0_20180203_151541';
iGroup = 350;

fileName_RawPC=[fileName_RawPC '.txt'];
fileName_LRFsIniPos='LRFsIniPos';
fileName_LRFsCalibPos='LRFsCalibPos';
rawData0





fileFullPath_RawPC=[RawFilePath fileName_RawPC '.mat'];
fileFullPath_LRFsIniPos=[RawFilePath fileName_LRFsIniPos '.mat'];
fileFullPath_LRFsCalibPos=[RawFilePath fileName_LRFsCalibPos '.mat'];

% load raw PC(point cloud) data
PC_Raw=load(fileFullPath_RawPC);
PC_Raw=PC_Raw.PC_Raw;
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
    R_LRFsIni(i,:,:)=EularAngle2RotateMat(ang_LRFsIni_radain(i,1),ang_LRFsIni_radain(i,2),ang_LRFsIni_radain(i,3),'xyz');
    R_LRFs(i,:,:)=EularAngle2RotateMat(ang_LRFs_radian(i,1),ang_LRFs_radian(i,2),ang_LRFs_radian(i,3),'xyz');
end
if isfield(LRFsCalibPos,'R_LRFs')
    R_LRFs=LRFsCalibPos.R_LRFs;
end
toc


%% choose one group to show the calibration result
fig1=figure; hold on,axis equal;
h = [-1500 100 1500 980];
% h = [80 0 1500 980];
set(gcf,'Position',h)
rotate3d on
lineColors=['r','g','b','m','c'];
LRFColors=['r','g','b'];
lineNorm=[0,0,0]; vector=[0,0,0]; corePt=[0,0,0]; basePt=[0,0,0];%basePt is for plot vector
vectorLineLength=2000;%用于绘制箭头时箭头的长度
farthestDist=9000;
minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
for iGroup=iGroup
% for iGroup=1:80:dataGroups
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



















