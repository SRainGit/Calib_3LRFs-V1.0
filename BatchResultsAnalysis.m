
% Show batch results
close all;clc;clear

RawFilePath = 'D:\LaserData\20180203\';
fileName_BatchResults='BatchResults';
fileFullPath_BatchResults=[RawFilePath fileName_BatchResults '.mat'];
BatchResults=load(fileFullPath_BatchResults);
Results=BatchResults.Results;
nGroups=size(Results,2);


angs_LRFs = cell2mat({Results.ang_LRFs});
Ts_LRFs = cell2mat({Results.T_LRFs});
ang_LRF2=angs_LRFs(2,:)';
ang_LRF2=reshape(ang_LRF2,3,size(ang_LRF2,1)/3)';
ang_LRF3=angs_LRFs(3,:)';
ang_LRF3=reshape(ang_LRF3,3,size(ang_LRF3,1)/3)';
T_LRF2=Ts_LRFs(2,:)';
T_LRF2=reshape(T_LRF2,3,size(T_LRF2,1)/3)';
T_LRF3=Ts_LRFs(3,:)';
T_LRF3=reshape(T_LRF3,3,size(T_LRF3,1)/3)';

%% Figure
fig1=figure; hold on, axis equal;
rotate3d on;
h = [-1500 100 1500 980];
set(gcf,'Position',h)
subplot(2,2,1);
plot3(ang_LRF2(:,1),ang_LRF2(:,2),ang_LRF2(:,3),'.');
title('ang-LRF2');
subplot(2,2,2);
plot3(T_LRF2(:,1),T_LRF2(:,2),T_LRF2(:,3),'.');
title('T-LRF2');
subplot(2,2,3);
plot3(ang_LRF3(:,1),ang_LRF3(:,2),ang_LRF3(:,3),'.');
title('ang-LRF3');
subplot(2,2,4);
plot3(T_LRF3(:,1),T_LRF3(:,2),T_LRF3(:,3),'.');
title('T-LRF3');

fig2=figure; hold on, axis equal;
rotate3d on;
h = [-800 100 500 800];
set(gcf,'Position',h)
subplot(2,3,1);
boxplot(ang_LRF2(:,1),'Labels',{'LRF2-Pitch/бу'});
subplot(2,3,2);
boxplot(ang_LRF2(:,2),'Labels',{'LRF2-Roll/бу'});
subplot(2,3,3);
boxplot(ang_LRF2(:,3),'Labels',{'LRF2-Yaw/бу'});
subplot(2,3,4);
boxplot(T_LRF2(:,1),'Labels',{'LRF2-T_X/mm'});
subplot(2,3,5);
boxplot(T_LRF2(:,2),'Labels',{'LRF2-T_Y/mm'});
subplot(2,3,6);
boxplot(T_LRF2(:,3),'Labels',{'LRF2-T_Z/mm'});
% h1=boxplot([ang_LRF2(:,1) ang_LRF2(:,2) ang_LRF2(:,3)],...
%     'Labels',{'0' '8' '0'},'colors','ggb');

a=0;












