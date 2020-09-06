
% Show batch results
close all;clc;clear

% RawFilePath = 'E:\SLAM\Data\LaserData\20180203\';



NumBatches=3;
PlotType=0; % 0:boxplot; 1: plot; 2:barweb;
% RawFilePath = 'E:\SLAM\Data\20180828_sim\';
RawFilePath = 'E:\SLAM\Data\20180928\';
Batch(1)=load([RawFilePath 'BatchResults-1_0.mat']);
Batch(2)=load([RawFilePath 'BatchResults-2_0.mat']);
Batch(3)=load([RawFilePath 'BatchResults-3_0.mat']);
groupnames={['W2H2'],['W4H2'],['W2H4']};
legend_Rotations={['Pitch'],['Roll'],['Yaw']};
legend_Translations={['X'],['Y'],['Z']};
Name_Rotation_Title='Rotation errors';
Name_Rotation_X='Corridor size';
Name_Rotation_Y='Degree';
Name_Translation_Title='Translation errors';
Name_Translation_X=Name_Rotation_X;
Name_Translation_Y='mm';


% NumBatches=4;
% RawFilePath = 'E:\SLAM\Data\20180828_sim\';
% Batch(1)=load([RawFilePath 'BatchResults-2-WithoutSelection.mat']);
% Batch(2)=load([RawFilePath 'BatchResults-2-step5-WithoutSelection.mat']);
% Batch(3)=load([RawFilePath 'BatchResults-2-step10-WithoutSelection.mat']);
% Batch(4)=load([RawFilePath 'BatchResults-2-step40-WithoutSelection.mat']);
% groupnames={['360'],['72'],['36'],['9']};
% legend_Rotations={['Pitch'],['Roll'],['Yaw']};
% legend_Translations={['X'],['Y'],['Z']};
% Name_Rotation_Title='Rotation errors';
% Name_Rotation_X='Number of Observations';
% Name_Rotation_Y='Degree';
% Name_Translation_Title='Translation errors';
% Name_Translation_X=Name_Rotation_X;
% Name_Translation_Y='mm';


% NumBatches=5;
% RawFilePath = 'E:\SLAM\Data\20180828_sim\';
% % Batch(1)=load([RawFilePath 'BatchResults-2.mat']);
% % Batch(2)=load([RawFilePath 'BatchResults-3.mat']);
% % Batch(3)=load([RawFilePath 'BatchResults-4.mat']);
% % Batch(4)=load([RawFilePath 'BatchResults-5.mat']);
% % Batch(5)=load([RawFilePath 'BatchResults-6.mat']);
% Batch(1)=load([RawFilePath 'BatchResults-2-WithoutSelection.mat']);
% Batch(2)=load([RawFilePath 'BatchResults-3-WithoutSelection.mat']);
% Batch(3)=load([RawFilePath 'BatchResults-4-WithoutSelection.mat']);
% Batch(4)=load([RawFilePath 'BatchResults-5-WithoutSelection.mat']);
% Batch(5)=load([RawFilePath 'BatchResults-6-WithoutSelection.mat']);
% groupnames={['B'],['C'],['D'],['E'],['F']};
% legend_Rotations={['Pitch'],['Roll'],['Yaw']};
% legend_Translations={['X'],['Y'],['Z']};
% legend_Rotations={['Pitch'],['Roll'],['Yaw']};
% legend_Translations={['X'],['Y'],['Z']};
% Name_Rotation_Title='Rotation errors';
% Name_Rotation_X='Rotation operation';
% Name_Rotation_Y='Degree';
% Name_Translation_Title='Translation errors';
% Name_Translation_X=Name_Rotation_X;
% Name_Translation_Y='mm';



% NumBatches=6;
% RawFilePath = 'E:\SLAM\Data\20180828_sim\';
% % Batch(1)=load([RawFilePath 'BatchResults-1.mat']);
% % Batch(2)=load([RawFilePath 'BatchResults-2.mat']);
% % Batch(3)=load([RawFilePath 'BatchResults-3.mat']);
% % Batch(4)=load([RawFilePath 'BatchResults-4.mat']);
% % Batch(5)=load([RawFilePath 'BatchResults-5.mat']);
% % Batch(6)=load([RawFilePath 'BatchResults-6.mat']);
% Batch(1)=load([RawFilePath 'BatchResults-1-WithoutSelection.mat']);
% Batch(2)=load([RawFilePath 'BatchResults-2-WithoutSelection.mat']);
% Batch(3)=load([RawFilePath 'BatchResults-3-WithoutSelection.mat']);
% Batch(4)=load([RawFilePath 'BatchResults-4-WithoutSelection.mat']);
% Batch(5)=load([RawFilePath 'BatchResults-5-WithoutSelection.mat']);
% Batch(6)=load([RawFilePath 'BatchResults-6-WithoutSelection.mat']);
% groupnames={['A'],['B'],['C'],['D'],['E'],['F']};
% legend_Rotations={['Pitch'],['Roll'],['Yaw']};
% legend_Translations={['X'],['Y'],['Z']};
% Name_Rotation_Title='Rotation errors';
% Name_Rotation_X='Rotation operation';
% Name_Rotation_Y='Degree';
% Name_Translation_Title='Translation errors';
% Name_Translation_X=Name_Rotation_X;
% Name_Translation_Y='mm';



ang_LRFs_sim=[
    0,0,0;
    -80, 0, -35;
    80, 0,  -150;
    ];
T_LRFs_sim=[
    0,0,0;
    -0.15, 0.15, -0.2;
    0.15,  0.15, -0.5;
    ];

for i=1:NumBatches
    Results=Batch(i).Results;
    angs_LRFs = cell2mat({Results.ang_LRFs});
    Ts_LRFs = cell2mat({Results.T_LRFs});
    angs_LRF2=angs_LRFs(2,:)';
    angs_LRF2=reshape(angs_LRF2,3,size(angs_LRF2,1)/3)';
    Ts_LRF2=Ts_LRFs(2,:)';
    Ts_LRF2=reshape(Ts_LRF2,3,size(Ts_LRF2,1)/3)';
    angs_LRF3=angs_LRFs(3,:)';
    angs_LRF3=reshape(angs_LRF3,3,size(angs_LRF3,1)/3)';
    Ts_LRF3=Ts_LRFs(3,:)';
    Ts_LRF3=reshape(Ts_LRF3,3,size(Ts_LRF3,1)/3)';
    
    Ts_LRF2=Ts_LRF2.*100;
    Ts_LRF3=Ts_LRF3.*100;
    
    % record the result of this batch
    batches_angs_LRF2(i,:,:)=angs_LRF2;
    batches_Ts_LRF2(i,:,:)=Ts_LRF2;
    batches_angs_LRF3(i,:,:)=angs_LRF3;
    batches_Ts_LRF3(i,:,:)=Ts_LRF3;
    
    mean_ang_2(i,:)=mean(angs_LRF2);
    std_ang_2(i,:)=std(angs_LRF2);
    mean_t_2(i,:)=mean(Ts_LRF2);
    std_t_2(i,:)=std(Ts_LRF2);
    mean_ang_3(i,:)=mean(angs_LRF3);
    std_ang_3(i,:)=std(angs_LRF3);
    mean_t_3(i,:)=mean(Ts_LRF3);
    std_t_3(i,:)=std(Ts_LRF3);
    
    errors_ang_LRF2=angs_LRF2-repmat(ang_LRFs_sim(2,:),size(angs_LRF2,1),1);
    errors_t_LRF2=Ts_LRF2-repmat(T_LRFs_sim(2,:)*1000,size(Ts_LRF2,1),1);
    
    errors_ang_LRF2=abs(errors_ang_LRF2);
    errors_t_LRF2=abs(errors_t_LRF2);
    
    mean_ang_error(i,:)=mean(errors_ang_LRF2);
    std_ang_error(i,:)=std(errors_ang_LRF2);
    mean_t_error(i,:)=mean(errors_t_LRF2);
    std_t_error(i,:)=std(errors_t_LRF2);
end

mean_ang_2
std_ang_2
mean_t_2
std_t_2
mean_ang_3
std_ang_3
mean_t_3
std_t_3

% return



%% Figure, result distributions
fig1=figure; hold on;
h = [-900 200 800 900];
set(gcf,'Position',h)
if PlotType==0
    if NumBatches==3
        subplot(4,3,1);
        boxplot([batches_angs_LRF2(1,:,1);batches_angs_LRF2(2,:,1);batches_angs_LRF2(3,:,1)]);
        title('Pitch');
        subplot(4,3,2);
        boxplot([batches_angs_LRF2(1,:,2);batches_angs_LRF2(2,:,2);batches_angs_LRF2(3,:,2)]);
        title('Roll');
        subplot(4,3,3);
        boxplot([batches_angs_LRF2(1,:,3);batches_angs_LRF2(2,:,3);batches_angs_LRF2(3,:,3)]);
        title('Yaw');
        subplot(4,3,4);
        boxplot([batches_Ts_LRF2(1,:,1);batches_Ts_LRF2(2,:,1);batches_Ts_LRF2(3,:,1)]);
        title('X');
        subplot(4,3,5);
        boxplot([batches_Ts_LRF2(1,:,2);batches_Ts_LRF2(2,:,2);batches_Ts_LRF2(3,:,2)]);
        title('Y');
        subplot(4,3,6);
        boxplot([batches_Ts_LRF2(1,:,3);batches_Ts_LRF2(2,:,3);batches_Ts_LRF2(3,:,3)]);
        title('Z');
        subplot(4,3,7);
        boxplot([batches_angs_LRF3(1,:,1);batches_angs_LRF3(2,:,1);batches_angs_LRF3(3,:,1)]);
        title('Pitch');
        subplot(4,3,8);
        boxplot([batches_angs_LRF3(1,:,2);batches_angs_LRF3(2,:,2);batches_angs_LRF3(3,:,2)]);
        title('Roll');
        subplot(4,3,9);
        boxplot([batches_angs_LRF3(1,:,3);batches_angs_LRF3(2,:,3);batches_angs_LRF3(3,:,3)]);
        title('Yaw');
        subplot(4,3,10);
        boxplot([batches_Ts_LRF3(1,:,1);batches_Ts_LRF3(2,:,1);batches_Ts_LRF3(3,:,1)]);
        title('X');
        subplot(4,3,11);
        boxplot([batches_Ts_LRF3(1,:,2);batches_Ts_LRF3(2,:,2);batches_Ts_LRF3(3,:,2)]);
        title('Y');
        subplot(4,3,12);
        boxplot([batches_Ts_LRF3(1,:,3);batches_Ts_LRF3(2,:,3);batches_Ts_LRF3(3,:,3)]);
        title('Z');
    elseif NumBatches==4
        boxplot([mean_ang_error(1,:);mean_ang_error(2,:);mean_ang_error(3,:);mean_ang_error(4,:)],...
            [std_ang_2(1,:);std_ang_2(2,:);std_ang_2(3,:);std_ang_2(4,:)])
    elseif NumBatches==5
        boxplot([mean_ang_error(1,:);mean_ang_error(2,:);mean_ang_error(3,:);mean_ang_error(4,:);mean_ang_error(5,:)],...
            [std_ang_2(1,:);std_ang_2(2,:);std_ang_2(3,:);std_ang_2(4,:);std_ang_2(5,:)])
    elseif NumBatches==6
        boxplot([mean_ang_error(1,:);mean_ang_error(2,:);mean_ang_error(3,:);mean_ang_error(4,:);mean_ang_error(5,:);mean_ang_error(6,:)],...
            [std_ang_2(1,:);std_ang_2(2,:);std_ang_2(3,:);std_ang_2(4,:);std_ang_2(5,:);std_ang_2(6,:)])
    end
elseif PlotType==1
    if NumBatches==3
        subplot(4,3,1);
        plot([batches_angs_LRF2(1,:,1);batches_angs_LRF2(2,:,1);batches_angs_LRF2(3,:,1)],'*');
        title('Pitch');
        subplot(4,3,2);
        plot([batches_angs_LRF2(1,:,2);batches_angs_LRF2(2,:,2);batches_angs_LRF2(3,:,2)],'*');
        title('Roll');
        subplot(4,3,3);
        plot([batches_angs_LRF2(1,:,3);batches_angs_LRF2(2,:,3);batches_angs_LRF2(3,:,3)],'*');
        title('Yaw');
        subplot(4,3,4);
        plot([batches_Ts_LRF2(1,:,1);batches_Ts_LRF2(2,:,1);batches_Ts_LRF2(3,:,1)],'*');
        title('X');
        subplot(4,3,5);
        plot([batches_Ts_LRF2(1,:,2);batches_Ts_LRF2(2,:,2);batches_Ts_LRF2(3,:,2)],'*');
        title('Y');
        subplot(4,3,6);
        plot([batches_Ts_LRF2(1,:,3);batches_Ts_LRF2(2,:,3);batches_Ts_LRF2(3,:,3)],'*');
        title('Z');
        subplot(4,3,7);
        plot([batches_angs_LRF3(1,:,1);batches_angs_LRF3(2,:,1);batches_angs_LRF3(3,:,1)],'*');
        title('Pitch');
        subplot(4,3,8);
        plot([batches_angs_LRF3(1,:,2);batches_angs_LRF3(2,:,2);batches_angs_LRF3(3,:,2)],'*');
        title('Roll');
        subplot(4,3,9);
        plot([batches_angs_LRF3(1,:,3);batches_angs_LRF3(2,:,3);batches_angs_LRF3(3,:,3)],'*');
        title('Yaw');
        subplot(4,3,10);
        plot([batches_Ts_LRF3(1,:,1);batches_Ts_LRF3(2,:,1);batches_Ts_LRF3(3,:,1)],'*');
        title('X');
        subplot(4,3,11);
        plot([batches_Ts_LRF3(1,:,2);batches_Ts_LRF3(2,:,2);batches_Ts_LRF3(3,:,2)],'*');
        title('Y');
        subplot(4,3,12);
        plot([batches_Ts_LRF3(1,:,3);batches_Ts_LRF3(2,:,3);batches_Ts_LRF3(3,:,3)],'*');
        title('Z');
    end
elseif PlotType==2
    subplot(1,2,1);
    if NumBatches==3
        barweb([mean_ang_error(1,:);mean_ang_error(2,:);mean_ang_error(3,:)],...
            [std_ang_2(1,:);std_ang_2(2,:);std_ang_2(3,:)],...
            1,groupnames,Name_Rotation_Title,Name_Rotation_X,Name_Rotation_Y,jet,'none',legend_Rotations,2,'plot')
    elseif NumBatches==4
        barweb([mean_ang_error(1,:);mean_ang_error(2,:);mean_ang_error(3,:);mean_ang_error(4,:)],...
            [std_ang_2(1,:);std_ang_2(2,:);std_ang_2(3,:);std_ang_2(4,:)],...
            1,groupnames,Name_Rotation_Title,Name_Rotation_X,Name_Rotation_Y,jet,'none',legend_Rotations,2,'plot')
    elseif NumBatches==5
        barweb([mean_ang_error(1,:);mean_ang_error(2,:);mean_ang_error(3,:);mean_ang_error(4,:);mean_ang_error(5,:)],...
            [std_ang_2(1,:);std_ang_2(2,:);std_ang_2(3,:);std_ang_2(4,:);std_ang_2(5,:)],...
            1,groupnames,Name_Rotation_Title,Name_Rotation_X,Name_Rotation_Y,jet,'none',legend_Rotations,2,'plot')
    elseif NumBatches==6
        barweb([mean_ang_error(1,:);mean_ang_error(2,:);mean_ang_error(3,:);mean_ang_error(4,:);mean_ang_error(5,:);mean_ang_error(6,:)],...
            [std_ang_2(1,:);std_ang_2(2,:);std_ang_2(3,:);std_ang_2(4,:);std_ang_2(5,:);std_ang_2(6,:)],...
            1,groupnames,Name_Rotation_Title,Name_Rotation_X,Name_Rotation_Y,jet,'none',legend_Rotations,2,'plot')
    end
    subplot(1,2,2);
    if NumBatches==3
        barweb([mean_t_error(1,:);mean_t_error(2,:);mean_t_error(3,:)],...
            [std_t_2(1,:);std_t_2(2,:);std_t_2(3,:)],...
            1,groupnames,Name_Translation_Title,Name_Translation_X,Name_Translation_Y,jet,'none',legend_Translations,2,'plot')
    elseif NumBatches==4
        barweb([mean_t_error(1,:);mean_t_error(2,:);mean_t_error(3,:);mean_t_error(4,:)],...
            [std_t_2(1,:);std_t_2(2,:);std_t_2(3,:);std_t_2(4,:)],...
            1,groupnames,Name_Translation_Title,Name_Translation_X,Name_Translation_Y,jet,'none',legend_Translations,2,'plot')
    elseif NumBatches==5
        barweb([mean_t_error(1,:);mean_t_error(2,:);mean_t_error(3,:);mean_t_error(4,:);mean_t_error(5,:)],...
            [std_t_2(1,:);std_t_2(2,:);std_t_2(3,:);std_t_2(4,:);std_t_2(5,:)],...
            1,groupnames,Name_Translation_Title,Name_Translation_X,Name_Translation_Y,jet,'none',legend_Translations,2,'plot')
    elseif NumBatches==6
        barweb([mean_t_error(1,:);mean_t_error(2,:);mean_t_error(3,:);mean_t_error(4,:);mean_t_error(5,:);mean_t_error(6,:)],...
            [std_t_2(1,:);std_t_2(2,:);std_t_2(3,:);std_t_2(4,:);std_t_2(5,:);std_t_2(6,:)],...
            1,groupnames,Name_Translation_Title,Name_Translation_X,Name_Translation_Y,jet,'none',legend_Translations,2,'plot')
    end
end

return

fig2=figure; hold on, axis equal;
rotate3d on;
h = [-800 100 550 800];
set(gcf,'Position',h)
subplot(2,3,1);
boxplot(ang_LRF2(:,1),'Labels',{'LRF2-Yaw'});
ylabel('Degree');
subplot(2,3,2);
boxplot(ang_LRF2(:,2),'Labels',{'LRF2-Pitch'});
ylabel('Degree');
subplot(2,3,3);
boxplot(ang_LRF2(:,3),'Labels',{'LRF2-Roll'});
ylabel('Degree');
subplot(2,3,4);
boxplot(T_LRF2(:,1),'Labels',{'LRF2-T_X'});
ylabel('mm');
subplot(2,3,5);
boxplot(T_LRF2(:,2),'Labels',{'LRF2-T_Y'});
ylabel('mm');
subplot(2,3,6);
boxplot(T_LRF2(:,3),'Labels',{'LRF2-T_Z'});
ylabel('mm');
% h1=boxplot([ang_LRF2(:,1) ang_LRF2(:,2) ang_LRF2(:,3)],...
%     'Labels',{'0' '8' '0'},'colors','ggb');


% fig3=figure; hold on, axis equal;
% rotate3d on;
% h = [-1000 300 900 400];
% set(gcf,'Position',h)
% subplot(2,3,1);
% plot(ang_LRF2(:,1),'r*-');
% xlabel('ang-LRF2-Pitch/бу');
% subplot(2,3,2);
% plot(ang_LRF2(:,2),'g*-');
% xlabel('ang-LRF2-Roll/бу');
% subplot(2,3,3);
% plot(ang_LRF2(:,3),'b*-');
% xlabel('ang-LRF2-Yaw/бу');
% subplot(2,3,4);
% plot(T_LRF2(:,1),'r*-');
% xlabel('T-LRF2-X/mm');
% subplot(2,3,5);
% plot(T_LRF2(:,2),'g*-');
% xlabel('T-LRF2-Y/mm');
% subplot(2,3,6);
% plot(T_LRF2(:,3),'b*-');
% xlabel('T-LRF2-Z/mm');













