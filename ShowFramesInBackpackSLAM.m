%����չʾSLAMƽ̨�е�����ϵ
clc;close all;clear

NUMBER_LRFs=3;

%% ��������ϵ�Ĳ���
%�豸����ϵ���������LRF0
R_D=eye(3,3);
T_D=[0,-130,-1000];

%IMU����ϵ
ang_Z_D2IMU=-pi/2;
ang_X_D2IMU=-pi;
R_D2IMU=EularAngle2RotateMat(ang_X_D2IMU,0,ang_Z_D2IMU,'zxy');
T_IMU=[100 20 -1050];

% the pose angle and translation of three LRFs
% From the iLRF to LRF0!!! (care for also line54)
% ang_LRFsIni=[
%     0,0,0;
%     -80.5, 0.45, -30;
%     88,1.45,-136.8;
%     ];
% % From LRF0 to LRFi
% T_LRFsIni=[
%     0,0,0;
%     87, 69,-394;
%     -134,25,-622;
%     ];
ang_LRFsIni=[
    0,0,0;
    90,0,-160;
    90, 0, 160;
    ];
% From LRF0 to LRFi
T_LRFsIni=[
    0,0,0;
    -150,150,-1500;
    150, 150,-1200;
    ];

ang_LRFsIni=ang_LRFsIni.*pi/180;
R_LRFsIni=zeros(3,3,3);
for i=1:3
    R_LRFsIni(i,:,:)=EularAngle2RotateMat(ang_LRFsIni(i,1),ang_LRFsIni(i,2),ang_LRFsIni(i,3),'xyz');
end


%% �����ϱ�־��ʵ�����ز���
CylinderCorePt=[0 -130 -350];
CylinderHeight=1000;
CylinderRadius=50;


%% ��ͼ
figure,hold on,axis equal,
% h = [ -650 30 600 900];
h = [  150 300 100 100];
set(gcf,'Position',h)
rotate3d on

%��������ϵ
axisLength_Body=100;
axisLength_Device=100;
% PlotAxes_TR('k','-',T_D,R_D,axisLength_Body,3);%�豸����ϵ
% PlotAxes_TR('r','-',T_IMU,R_D2IMU,axisLength_Device,3);%IMU����ϵ
PlotAxes_TR('c','-',squeeze(T_LRFsIni(1,:)),squeeze(R_LRFsIni(1,:,:))',axisLength_Device,3);%LS1����ϵ (From 0LRF to iLRF!11)
PlotAxes_TR('g','-',squeeze(T_LRFsIni(2,:)),squeeze(R_LRFsIni(2,:,:))',axisLength_Device,3);%LS2����ϵ
PlotAxes_TR('m','-',squeeze(T_LRFsIni(3,:)),squeeze(R_LRFsIni(3,:,:))',axisLength_Device,3);%LS3����ϵ
xlabel('X/mm');
ylabel('Y/mm');
zlabel('Z/mm');
view([1 1 1]);

%���õ����豸��Сƽ̨
x=-200:200;
y=-100:300;
[x,y]=meshgrid(x,y);
% z=x*0-1000;
z=x*0-100;
surf(x,y,z);
alpha(.5);%͸����
shading interp

%Բ��
R_Cylinder=eye(3,3);
% R_Cylinder=EularAngle2RotateMat(30,0,0,'xyz');
PlotCylinder(CylinderCorePt,CylinderRadius,CylinderHeight,R_Cylinder,'b',1);

axis off
% legend('Body','IMU','LS1','LS2','LS3','Platform','Shaft')


% 
% PlotAxes_TR('r','-',T_D,R_D,100,3);%�豸����ϵ
% axis off;
% view([1 -1 1]);
% % print(gcf,'-dpng','E:\SLAM\Documents\3LRFs+IMU����궨����\���ͼƬ\axis_g.jpg')
% % saveas(gcf,'E:\SLAM\Documents\3LRFs+IMU����궨����\���ͼƬ\axis_g.jpg')



