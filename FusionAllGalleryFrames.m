% Show calibration result
close all;clc;clear

NUMBER_LRFs=3;
PTS_PER_FRAME=1081;

%% load data
tic
% set file path
disp('Loading data ...')
RawFilePath = 'D:\LaserData\20170830\';
fileName_RawPC='UTM30LX_0_20170830_144433';
fileName_RawPC=['RawPC_' fileName_RawPC];
fileName_LRFsCalibPos='LRFsCalibPos';
fileFullPath_RawPC=[RawFilePath fileName_RawPC '.mat'];
fileFullPath_LRFsCalibPos=[RawFilePath fileName_LRFsCalibPos '.mat'];

% load raw PC(point cloud) data
PC_Raw=load(fileFullPath_RawPC);
PC_Raw=PC_Raw.PC_Raw;
dataGroups=size(PC_Raw,2);

% load CalibPos parameters
LRFsCalibPos=load(fileFullPath_LRFsCalibPos);
ang_LRFs=LRFsCalibPos.ang_LRFs;
T_LRFs=LRFsCalibPos.T_LRFs;

ang_LRFs_radian=ang_LRFs.*pi/180;
for i=1:3
    R_LRFs(i,:,:)=EularAngle2RotateMat(ang_LRFs_radian(i,1),ang_LRFs_radian(i,2),ang_LRFs_radian(i,3),'xyz');
end
toc