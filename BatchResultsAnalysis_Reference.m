
% Show batch results
close all;clc;clear

RawFilePath = 'E:\SLAM\LRF\';
fileName_Angles_2='Angles_LRF2';
fileName_Translations_2='Translations_LRF2';
fileName_Angles_3='Angles_LRF3';
fileName_Translations_3='Translations_LRF3';
fileFullPath_Angles_2=[RawFilePath fileName_Angles_2 '.txt'];
fileFullPath_Translations_2=[RawFilePath fileName_Translations_2 '.txt'];
fileFullPath_Angles_3=[RawFilePath fileName_Angles_3 '.txt'];
fileFullPath_Translations_3=[RawFilePath fileName_Translations_3 '.txt'];
ang_LRF2=load(fileFullPath_Angles_2);
T_LRF2=load(fileFullPath_Translations_2);
ang_LRF3=load(fileFullPath_Angles_3);
T_LRF3=load(fileFullPath_Translations_3);
nGroups=size(ang_LRF2,1);


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
h = [-800 100 700 800];
set(gcf,'Position',h)
subplot(2,3,1);
boxplot(ang_LRF2(:,1),'Labels',{'LRF2-Yaw'});
subplot(2,3,2);
boxplot(ang_LRF2(:,2),'Labels',{'LRF2-Pitch'});
subplot(2,3,3);
boxplot(ang_LRF2(:,3),'Labels',{'LRF2-Roll'});
subplot(2,3,4);
boxplot(T_LRF2(:,1),'Labels',{'LRF2-T_X'});
subplot(2,3,5);
boxplot(T_LRF2(:,2),'Labels',{'LRF2-T_Y'});
subplot(2,3,6);
boxplot(T_LRF2(:,3),'Labels',{'LRF2-T_Z'});


% fig3=figure; hold on, axis equal;
% rotate3d on;
% h = [-800 100 700 800];
% set(gcf,'Position',h)
% subplot(2,3,1);
% boxplot(ang_LRF3(:,1),'Labels',{'LRF3-Yaw'});
% subplot(2,3,2);
% boxplot(ang_LRF3(:,2),'Labels',{'LRF3-Pitch'});
% subplot(2,3,3);
% boxplot(ang_LRF3(:,3),'Labels',{'LRF3-Roll'});
% subplot(2,3,4);
% boxplot(T_LRF3(:,1),'Labels',{'LRF3-T_X'});
% subplot(2,3,5);
% boxplot(T_LRF3(:,2),'Labels',{'LRF3-T_Y'});
% subplot(2,3,6);
% boxplot(T_LRF3(:,3),'Labels',{'LRF3-T_Z'});
% % h1=boxplot([ang_LRF2(:,1) ang_LRF2(:,2) ang_LRF2(:,3)],...
% %     'Labels',{'0' '8' '0'},'colors','ggb');

a=0;












