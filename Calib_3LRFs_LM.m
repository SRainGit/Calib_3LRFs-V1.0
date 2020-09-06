% function Calib_3LRFs_LM(DataPath,FileName0,iStartFrame,iFrameStep,iEndFrame)
close all;clc;clear;close all;
NUMBER_LRFs=3;
PTS_PER_FRAME=1081;
half_PI=pi/2;

tic
% set file path
disp('Loading data ...')
% RawFilePath = DataPath;
% fileName_RawPC=['RawPC_' FileName0];

% RawFilePath = 'D:\LaserData\20171113\';
% fileName_RawPC=['RawPC_' 'UTM30LX_0_20171113_210018'];

RawFilePath = 'E:\SLAM\Data\LaserData\20180203\';
fileName_RawPC=['RawPC_' 'UTM30LX_0_20180203_151541'];
iStartFrame=239;
iFrameStep=3;
iEndFrame=340;
% iStartFrame=1;
% iFrameStep=3;
% iEndFrame=-1;
% % % fileName_RawPC=['RawPC_' 'UTM30LX_0_20180203_151439']; % 2553-2567,15个有临边的GOs
% % % fileName_RawPC=['RawPC_' 'UTM30LX_0_20180203_150300']; 
% % % fileName_RawPC=['RawPC_' 'UTM30LX_0_20180203_150114'];

% RawFilePath = 'E:\SLAM\Data\LaserData\20180201\';
% % fileName_RawPC=['RawPC_' 'UTM30LX_0_20180201_164158'];
% fileName_RawPC=['RawPC_' 'UTM30LX_0_20180201_164535'];


% RawFilePath = 'E:\SLAM\Data\20180828_sim\';
% fileName_RawPC=['Data_' '7_0'];
% iStartFrame=1;
% iFrameStep=1;
% iEndFrame=-1;

% % RawFilePath = 'E:\SLAM\Data\20180802\';
% RawFilePath = 'E:\SLAM\Data\20180928\';
% % fileName_RawPC=['Data_' '1_0'];
% % iStartFrame=1;
% % iFrameStep=20;
% % iEndFrame=-1;
% fileName_RawPC=['Data_' '2_0'];
% iStartFrame=700;
% iFrameStep=20;
% iEndFrame=-1;
% % fileName_RawPC=['Data_' '3_0'];
% % iStartFrame=100;
% % iFrameStep=100;
% % iEndFrame=700;
% % iStartFrame=100;
% % iFrameStep=120;
% % iEndFrame=1450;


fileName_LRFsIniPos='LRFsIniPos';
fileFullPath_RawPC=[RawFilePath fileName_RawPC '.mat'];
fileFullPath_LRFsIniPos=[RawFilePath fileName_LRFsIniPos '.mat'];

% iFrameStepList=[5,10,40,100];
iFrameStepList=iFrameStep;
for WithoutSelction=0
for iFrameStep=iFrameStepList
for iRotationOperation=2:2
for iCorridorSize=1:1
    
cntTries=0;
for iRepeat=1:1:1
% SyntheticDataGenerator_Master(iRotationOperation,iFrameStep,iCorridorSize)


% load raw PC(point cloud) data
PC_Raw=load(fileFullPath_RawPC);
PC_Raw=PC_Raw.PC_Raw;
% PC_Raw=PC_Raw*1000;
dataGroups=size(PC_Raw,2);

% load iniPos parameters
LRFsIniPos=load(fileFullPath_LRFsIniPos);

ang_LRFsIni_Ori=LRFsIniPos.ang_LRFsIni;
T_LRFsIni_Ori=LRFsIniPos.T_LRFsIni;
% T_LRFsIni_Ori=T_LRFsIni_Ori.*1000;
   
tic
% for deltaAng_21=1:1:10
% for deltaAng_22=-10:10:10
% for deltaAng_23=-10:10:10
% % for deltaAng_31=-10:5:10
% % for deltaAng_32=-10:5:10
% % for deltaAng_33=-10:5:10
% for deltaT_21=-100:100:100
% for deltaT_22=-100:100:100
% for deltaT_23=-100:100:100
% % for deltaT_31=-200:100:200
% % for deltaT_32=-200:100:200
% % for deltaT_33=-200:100:200

    deltaAng=[
        0,0,0;
        0,0,0;
        0,0,0;];
    deltaT=[
        0,0,0;
        0,0,0;
        0,0,0;];
    
%     deltaAng=[
%         0,0,0;
%         deltaAng_21,deltaAng_22,deltaAng_23;
%         0,0,0;];
%     deltaT=[
%         0,0,0;
%         deltaT_21,deltaT_22,deltaT_23;
%         0,0,0;];
    
%     deltaAng=[
%         0,0,0;
%         deltaAng_21,deltaAng_22,deltaAng_23;
%         deltaAng_31,deltaAng_32,deltaAng_33;];
%     deltaT=[
%         0,0,0;
%         deltaT_21,deltaT_22,deltaT_23;
%         deltaT_31,deltaT_32,deltaT_33;];
    
    ang_LRFsIni=ang_LRFsIni_Ori+deltaAng;
    T_LRFsIni=T_LRFsIni_Ori+deltaT;
    
    cntTries=cntTries+1;
    disp(['cntTries=',num2str(cntTries),'...']);

ang_LRFsIni_radian=ang_LRFsIni.*pi/180;
R_LRFsIni=zeros(3,3,3);
for i=1:3
    R_LRFsIni(i,:,:)=EularAngle2RotateMat(ang_LRFsIni_radian(i,1),ang_LRFsIni_radian(i,2),ang_LRFsIni_radian(i,3),'xyz');
end

% device poses
R_DevicePoses=zeros(dataGroups,3,3);
T_DevicePoses=zeros(dataGroups,3);


%% ============= RANSAC for fitting 2D lines and find coplanar line pairs ===========================
% tic
% parameters for lnie detection based on RANSAC
nearestDist=500;%设定最近的视野范围，mm
farthestDist=6*1000;%设定最远的视野范围，mm
distThreshold=10;%距离阈值，超出该阈值认为是outliers，单位mm
minLineLength=600;%提取出线段的最短距离，实质用作采集两点的时候两点间的最短距离，用于提升检测精度
minInliersPts=150;
maxTotalIterTimes=1000;
fixedInerIterTimes=100;%固定的内循环次数，用于寻找尽量最优的model

% parameters and variables for detecting coplanar line segments
linesDistThreshold=200; % the distance between coplanar lines should smaller than this value ( for first step detection)
maxLinesDist=1000; % the distance between coplanar lines should smaller than this value ( for second step detection)
allLines_cnt=zeros(1,3); % record the number of allt the lines detected in the all groups; 3 number for 3LRFs
allLines_centerPt=zeros(3,100,3); % (iLRF,iLine,:)
allLines_vector=zeros(3,100,3); % the number of the second dimension will changes as the line detection going on

% coplanar line pairs
cntLinePairs(3,3)=0;
linePairsIndex=zeros(3,3,10,3);
linePairs=zeros(3,3,10,3,3);
% orthogonality line pairs
cntOrthogonalityPairs=0;
% total line pairs in all data frames
cntLinePairs(3,3)=0;

bShowFigure = 0 ; % It is used to control whether the drawing is opened.
% bShowFigure = 1 ; % It is used to control whether the drawing is opened.
if bShowFigure
    fig1=figure; h = [-1500 110 1500 980]; set(gcf,'Position',h)
%         figure, h = [150 50 900 600]; set(gcf,'Position',h)
    h1=zeros(1,4);
    rotate3d on 
    lineColors=['r','g','b','m','c'];
    LRFColors=['r','g','b'];
    lineNorm=[0,0,0]; vector=[0,0,0]; centerPt=[0,0,0]; basePt=[0,0,0];%basePt is for plot vector
    vectorLineLength=2000;%用于绘制箭头时箭头的长度
end
centerPt1=[0,0,0];centerPt2=[0,0,0];vector1=[0,0,0];vector2=[0,0,0]; distance=0;
cntSelectedGroups=0;
% for each group data; each group data include 3LRF's data
if iEndFrame<=0
    iEndFrame=dataGroups;
end
for iGroup=iStartFrame:iFrameStep:iEndFrame
% for iGroup=100:1:dataGroups-100
% for iGroup=240:2:340
% for iGroup=300:1:350
% for iGroup=841
    disp(['第' num2str(iGroup) '组']);
    minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
    for iLRF=1:NUMBER_LRFs
        %% Detecting lines by RANSAC， for each LRF
        data=squeeze(PC_Raw(iLRF,iGroup,:,:));
        % detecting
        [cntLines,lines,nInliers,inliers,outliers] = ...
            RANSAC_DetectLines_2D(data(:,1:2),size(data,1),nearestDist,farthestDist, ...
            distThreshold, minLineLength, minInliersPts, maxTotalIterTimes,fixedInerIterTimes);
        % 预处理检测出的直线：法向量、中点等信息
        iLine=1;
        while iLine<=cntLines
            %解算直线信息：重心和法向量；并根据初试位姿将其变换到Device坐标系下
            centerPt(1)=mean(inliers(iLine,1:nInliers(iLine),1));%每条线的重心，根据inliers计算
            centerPt(2)=mean(inliers(iLine,1:nInliers(iLine),2));
            centerPt(3)=0;            
            lineNorm=[-lines(iLine,2),lines(iLine,1),0];
            vector=lineNorm/norm(lineNorm);%每条线的法向量            
            % save the line information to the struct data (for display and coplanar line detection)
            LRF_Lines(iLRF,iGroup).lines(iLine).idLRF=iLRF; % the index of this line in the all lines
            LRF_Lines(iLRF,iGroup).lines(iLine).coefficients=squeeze(lines(iLine,:)); %
            LRF_Lines(iLRF,iGroup).lines(iLine).nInliers=nInliers(iLine);%每条线的inliers数量
            LRF_Lines(iLRF,iGroup).lines(iLine).inliers=squeeze(inliers(iLine,1:nInliers(iLine),:));%每条线的inliers数量
            LRF_Lines(iLRF,iGroup).lines(iLine).centerPt=centerPt;
            LRF_Lines(iLRF,iGroup).lines(iLine).vector=vector;%每条线的法向量
            % endPts
            projection=squeeze(inliers(iLine,1:nInliers(iLine),:))*vector(1,1:2)';
            sortProjection=sort(projection);
            if abs(sortProjection(10)-sortProjection(length(sortProjection)-10))<minLineLength
                LRF_Lines(iLRF,iGroup).lines(iLine)=[];
                lines(iLine,:)=[];
                outliers(size(outliers,1)+1:size(outliers,1)+nInliers(iLine),:)=...
                    inliers(iLine,1:nInliers(iLine),:);
                cntLines=cntLines-1;
                nInliers(iLine)=[];
                inliers(iLine,:,:)=[];
                continue;
            end
            [maxValue,iMax]=max(projection);
            [minValue,iMin]=min(projection);
            LRF_Lines(iLRF,iGroup).lines(iLine).endPts(1,:)=inliers(iLine,iMin,:);
            LRF_Lines(iLRF,iGroup).lines(iLine).endPts(2,:)=inliers(iLine,iMax,:);
            iLine=iLine+1;
        end
        % remove the double line at the top of gallery
        if cntLines>=2
            iLine1=1;
            while iLine1<=cntLines-1
                iLine2=iLine1+1;
                while iLine2<=cntLines
                    centerPt1=LRF_Lines(iLRF,iGroup).lines(iLine1).centerPt;
                    vector1=LRF_Lines(iLRF,iGroup).lines(iLine1).vector;
                    centerPt2=LRF_Lines(iLRF,iGroup).lines(iLine2).centerPt;
                    vector2=LRF_Lines(iLRF,iGroup).lines(iLine2).vector;
                    if norm(cross(vector1,vector2))<sin(10*pi/180) % if their direction are close to the same
                        if norm(cross(vector1,(centerPt1-centerPt2)))<200 % if their distance(approximation) is small enough
                            dist2ZeroPt1=abs(dot(centerPt1,[vector1(2),-vector1(1),0]));
                            dist2ZeroPt2=abs(dot(centerPt2,[vector2(2),-vector2(1),0]));
                            if dist2ZeroPt1<dist2ZeroPt2 % remv the further one, and take its inliers as outliers
                                LRF_Lines(iLRF,iGroup).lines(iLine2)=[];
                                lines(iLine2,:)=[];
                                outliers(size(outliers,1)+1:size(outliers,1)+nInliers(iLine2),:)=...
                                    inliers(iLine2,1:nInliers(iLine2),:);
                                cntLines=cntLines-1;
                                nInliers(iLine2)=[];
                                inliers(iLine2,:,:)=[];
                                continue;
                            else
                                LRF_Lines(iLRF,iGroup).lines(iLine1)=[];
                                lines(iLine1,:)=[];
                                outliers(size(outliers,1)+1:size(outliers,1)+nInliers(iLine1),:)=...
                                    inliers(iLine1,1:nInliers(iLine1),:);
                                nInliers(iLine1)=[];
                                inliers(iLine1,:,:)=[];
                                iLine1=iLine1-1;
                                iLine1=max(iLine1,1); % ensure the smaller one's index is bigger than zero
                                iLine2=iLine2-1;
                                cntLines=cntLines-1;
                            end
                        end
                    end
                    iLine2=iLine2+1;
                end
                iLine1=iLine1+1;
            end
        end
        LRF_Lines(iLRF,iGroup).cntLines=cntLines; % the final number of lines detected
        
        %% Sorting the lines by the scanning sequnce(the clockwise angle to X axis) (this is for finding inliers)
        axisX=[1,0,0];
        for iter=1:cntLines-1
            index=iter;
            tempPt=LRF_Lines(iLRF,iGroup).lines(iter).centerPt;
            dotVal=dot(tempPt,axisX);
            tAng2X=acos(dotVal/norm(tempPt));
            if tempPt(2)>0
                tAng2X=2*pi-tAng2X;
            end
            for iLine=iter+1:cntLines
                centerPt=LRF_Lines(iLRF,iGroup).lines(iLine).centerPt;
                dotVal=dot(centerPt,axisX);
                ang2X=acos(dotVal/norm(centerPt));
                if centerPt(2)>0
                    ang2X=2*pi-ang2X;
                end
                if ang2X<tAng2X
                    index=iLine;
                    tAng2X=ang2X;
                end
            end
            % replace the minimal angle to X axis in current iteratioin
            if index~=iter
                tempLines=LRF_Lines(iLRF,iGroup).lines(iter);
                LRF_Lines(iLRF,iGroup).lines(iter)=LRF_Lines(iLRF,iGroup).lines(index);
                LRF_Lines(iLRF,iGroup).lines(index)=tempLines;
            end
        end
        % recomupte each angles to the X axis
        angles=zeros(1,cntLines);
        for iLine=1:cntLines
            tempPt=LRF_Lines(iLRF,iGroup).lines(iLine).centerPt;
            dotVal=dot(tempPt,axisX);
            tAng2X=acos(dotVal/norm(tempPt));
            if tempPt(2)>0
                tAng2X=2*pi-tAng2X;
            end
            angles(iLine)=tAng2X*180/pi;
        end
        % guess the gaps between lines
        gapsBetweenLines=zeros(1,cntLines); % it means: if there are l1,l2, there wall indexes are w1,w3, then the gap is 2
        for iLine=1:cntLines-1
            v1=LRF_Lines(iLRF,iGroup).lines(iLine).vector;
            v2=LRF_Lines(iLRF,iGroup).lines(iLine+1).vector;
            if abs(dot(v1,v2))>0.995 % the number is about acos(5.7*pi/180)
                gapsBetweenLines(iLine+1)=2; % the gap between parallel lines
            else
                % compute the intersection of the two lines
                coefs1=LRF_Lines(iLRF,iGroup).lines(iLine).coefficients;
                coefs2=LRF_Lines(iLRF,iGroup).lines(iLine+1).coefficients;
                k1=-coefs1(1)/coefs1(2); b1=-coefs1(3)/coefs1(2);
                k2=-coefs2(1)/coefs2(2); b2=-coefs2(3)/coefs2(2);
                intersectionPt=GetCrossPtOf2Lines(k1,b1,k2,b2); intersectionPt(3)=0;
                % get the two vectors between the two centerPt 2 intersectionPt
                c1=LRF_Lines(iLRF,iGroup).lines(iLine).centerPt; c1(3)=0;
                c2=LRF_Lines(iLRF,iGroup).lines(iLine+1).centerPt; c2(3)=0;
                v1=intersectionPt-c1;
                v2=c2-intersectionPt;
                crossV=cross(v1,v2);
                if crossV(3)>0
                    gapsBetweenLines(iLine+1)=3; % it means angle(iLine)<90 && angle(iLine+1)>270
                else
                    gapsBetweenLines(iLine+1)=1; % the default gap
                end
            end
        end
        % get the id of walls for the lines according to the gaps
        idWalls=zeros(1,cntLines);
        cntWalls=1;
        for iLine=1:cntLines
            cntWalls=cntWalls+gapsBetweenLines(iLine);
            idWalls(iLine)=cntWalls;
        end
        % ensure that we have got the right gaps
        if sum(gapsBetweenLines)+1>4
            disp('Wrong gaps!!!!!!!!!!!!!!!!!');
%             return;
        end
        % record the idWalls
        LRF_Lines(iLRF,iGroup).idWalls=idWalls;
        
        %% figure
        % 每个LRF的直线检测效果
        if bShowFigure
            h1(iLRF)=subplot(2,2,iLRF); hold on, axis equal;
            cla(h1(iLRF));
            offset=500;
            % outliers
            plot(outliers(:,1),outliers(:,2),'k.');
            xlim([min(data(:,1))-offset  max(data(:,1))+offset]);
            ylim([min(data(:,2))-offset  max(data(:,2))+offset]);
            % inliers and lines
            if cntLines>0
                title([num2str(cntLines), ' lines in LRF', num2str(iLRF)]);
                for iLine=1:cntLines
                    inliers=LRF_Lines(iLRF,iGroup).lines(iLine).inliers;
                    coefficients=LRF_Lines(iLRF,iGroup).lines(iLine).coefficients;
                    plot(inliers(:,1),inliers(:,2),[lineColors(iLine),'.']);
                    x=min(data(:,1)):max(data(:,1));
                    y=(coefficients(1).*x+coefficients(3))./(-coefficients(2));
                    plot(x,y,[lineColors(iLine),'-']);
                    centerPt=LRF_Lines(iLRF,iGroup).lines(iLine).centerPt;
                    plot(centerPt(1),centerPt(2),'ko','LineWidth',3);
                end
            end
            xlabel('X/mm'); ylabel('Y/mm');
            hold off;
            % 显示3LRF点云在初始位姿下的效果
            h1(4)=subplot(2,2,4);
            if iLRF==1
                cla(h1(4));
            end
            hold on, axis equal;
            %绘制初始位姿下的点云
            clear allInliers;
            cntAllInliers1=1; cntAllInliers2=0;
            for iLine=1:cntLines
                nOneInliers= LRF_Lines(iLRF,iGroup).lines(iLine).nInliers;
                oneInliers=LRF_Lines(iLRF,iGroup).lines(iLine).inliers;
                cntAllInliers2=cntAllInliers2+nOneInliers;
                allInliers(cntAllInliers1:cntAllInliers2,:)=squeeze(oneInliers(1:nOneInliers,:));
                cntAllInliers1=cntAllInliers2+1;
            end
            allInliers(:,3)=0;
            PC_IniPos=(squeeze(R_LRFsIni(iLRF,:,:))*allInliers')'+repmat(T_LRFsIni(iLRF,:),size(allInliers,1),1);
%             plot3(PC_IniPos(:,1),PC_IniPos(:,2),PC_IniPos(:,3),[LRFColors(iLRF),'.'],'LineWidth',0.1);
            %绘制检测出直线在初始位姿下的法向量和重心点
            if cntLines>0
                for iLine=1:cntLines
                    centerPt=LRF_Lines(iLRF,iGroup).lines(iLine).centerPt;
                    vector=8*LRF_Lines(iLRF,iGroup).lines(iLine).vector*LRF_Lines(iLRF,iGroup).lines(iLine).nInliers;
                    
                    centerPt(3)=0;
                    centerPt=(squeeze(R_LRFsIni(iLRF,:,:))*centerPt')'+T_LRFsIni(iLRF,:);
                    vector=(squeeze(R_LRFsIni(iLRF,:,:))*vector')';
                    
%                     plot3(centerPt(1),centerPt(2),centerPt(3),'ko','LineWidth',3);
                    
%                     basePt=centerPt-0.5.*vector;
%                     quiver3(basePt(:,1),basePt(:,2),basePt(:,3)...
%                         ,vector(:,1),vector(:,2),vector(:,3),'k','LineWidth',3);
                end
            end
            xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
            ylim([min(allInliers(:,2))-offset  farthestDist]);
            allInliers=[];
            hold off;
        end
    end
    
    % ensure that we have got enough lines
    cntSelectedGroups=cntSelectedGroups+1;
    cntAllLines=0; bBadLineDetection=0;
    for iLRF=1:NUMBER_LRFs
        if LRF_Lines(iLRF,iGroup).cntLines < 2 || LRF_Lines(iLRF,iGroup).cntLines > 4
            bBadLineDetection=1;
        end
%         allCntDetectedLines(cntSelectedGroups,iLRF)=LRF_Lines(iLRF,iGroup).cntLines;
        cntAllLines=cntAllLines+LRF_Lines(iLRF,iGroup).cntLines;
    end
    if cntAllLines<7 || bBadLineDetection==1
        disp('Not enough lines.');
%         selectedGroupsList(cntSelectedGroups)=[];
        cntSelectedGroups=cntSelectedGroups-1;
        continue;
    end
    
    %% Generate all possible GOs in this dataGroup, and find a best one
    cntGOs=0;
    clear GOsInOneFrame;
    % Fix the lines of LRF1, take the line number as the wall number
    % and arrange the lines of LRF2 & the lines of LRF3
    i2_step=2;
    i3_step=2;
    % if 2 lines lies opposite walls in iLRF1 or iLRF2, then no need reverse check for iLRF2
    if LRF_Lines(1,iGroup).cntLines==2
        if abs(LRF_Lines(1,iGroup).idWalls(1)-LRF_Lines(1,iGroup).idWalls(2))==2
            i2_step=3;
        end
    elseif LRF_Lines(2,iGroup).cntLines==2
        if abs(LRF_Lines(2,iGroup).idWalls(1)-LRF_Lines(2,iGroup).idWalls(2))==2
            i2_step=3;
        end
    end
    if LRF_Lines(3,iGroup).cntLines==2
        if abs(LRF_Lines(3,iGroup).idWalls(1)-LRF_Lines(3,iGroup).idWalls(2))==2
            i3_step=3;
        end
    end
    for i2=-1:i2_step:1 % Forward and reverse (LRF2)
        for offset2=0:1:3 % 4 possibles by offset
            for i3=-1:i3_step:1 % (LRF3)
                for offset3=0:1:3
                    % counter
                    cntGOs=cntGOs+1;
                    % initiate GOs(cntGOs)
                    for iWall=1:4
                        GOsInOneFrame(cntGOs,iWall).cntLines=0;
                    end
                    % Fix the lines of LRF1, take the line number as the wall number
                    for iLine1=1:LRF_Lines(1,iGroup).cntLines
                        idWalls=LRF_Lines(1,iGroup).idWalls;
                        iWall=idWalls(iLine1);
                        GOsInOneFrame(cntGOs,iWall).cntLines=GOsInOneFrame(cntGOs,iWall).cntLines+1;
                        GOsInOneFrame(cntGOs,iWall).lines(GOsInOneFrame(cntGOs,iWall).cntLines,:)=LRF_Lines(1,iGroup).lines(iLine1);
                    end
                    % arrange the lines of LRF2
                    for iLine2=1:LRF_Lines(2,iGroup).cntLines
                        idWalls=LRF_Lines(2,iGroup).idWalls;
                        iWall=mod(i2*(offset2+idWalls(iLine2))-1,4)+1;
                        GOsInOneFrame(cntGOs,iWall).cntLines=GOsInOneFrame(cntGOs,iWall).cntLines+1;
                        GOsInOneFrame(cntGOs,iWall).lines(GOsInOneFrame(cntGOs,iWall).cntLines,:)=LRF_Lines(2,iGroup).lines(iLine2);
                    end
                    % arrange the lines of LRF3
                    for iLine3=1:LRF_Lines(3,iGroup).cntLines
                        idWalls=LRF_Lines(3,iGroup).idWalls;
                        iWall=mod(i3*(offset3+idWalls(iLine3))-1,4)+1;
                        GOsInOneFrame(cntGOs,iWall).cntLines=GOsInOneFrame(cntGOs,iWall).cntLines+1;
                        GOsInOneFrame(cntGOs,iWall).lines(GOsInOneFrame(cntGOs,iWall).cntLines,:)=LRF_Lines(3,iGroup).lines(iLine3);
                    end
                end
            end
        end
    end
    % compute the errors in all the GOs
%     t1=clock;
    errorsGO=zeros(cntGOs,1);
    for iGO=1:cntGOs
        oneGO.indexDataGroup=iGroup;
        oneGO.GO=GOsInOneFrame(iGO,:);
        errorsGO(iGO)=ErrorInOneGO(oneGO,NUMBER_LRFs,R_LRFsIni,T_LRFsIni);
%         errorsGO(iGO)=ErrorInOneGO_AllLRFs(oneGO,R_LRFsIni,T_LRFsIni);
    end
%     t2=clock;
%     disp([num2str(etime(t2,t1)) ' seconds']);
    
    %% Input the GO with minimal error to the linePairs
    [minErrorGO,iMin]=min(errorsGO);
    GOs(cntSelectedGroups).indexDataGroup=iGroup;
    GOs(cntSelectedGroups).GO=GOsInOneFrame(iMin,:);
    
    
    %% Show the GO with minimal error
    if bShowFigure
        oneGO.indexDataGroup=iGroup;
        oneGO.GO=GOsInOneFrame(iMin,:);
        ErrorInOneGO(oneGO,NUMBER_LRFs,R_LRFsIni,T_LRFsIni);
%         figure, h = [-900 110 800 600]; set(gcf,'Position',h)
        subplot(2,2,4); %cla(h1(4));
        hold on; rotate3d on; axis equal;
        WallColors=['r','g','b','m','c'];
        minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
        title(['Sum of Volume = ', num2str(minErrorGO), '(m^3)']);
        for iWall=1:4
            % extract data
            cntLines=oneGO.GO(iWall).cntLines;
            lines=oneGO.GO(iWall).lines;
            % skip the wall without enough lines
            if cntLines>0
                clear allInliers;
                cntAllInliers1=1; cntAllInliers2=0;
                for iLine=1:cntLines
                    idLRF=lines(iLine).idLRF;
                    R_=squeeze(R_LRFsIni(idLRF,:,:));
                    T_=squeeze(T_LRFsIni(idLRF,:));
                    nOneInliers=lines(iLine).nInliers;
                    oneInliers=lines(iLine).inliers;
                    oneInliers(:,3)=0;
                    oneInliers=(R_*oneInliers')'+repmat(T_,nOneInliers,1);
                    cntAllInliers2=cntAllInliers2+nOneInliers;
                    allInliers(cntAllInliers1:cntAllInliers2,:)=squeeze(oneInliers(1:nOneInliers,:));
                    cntAllInliers1=cntAllInliers2+1;
                end
                % The gallery observation in this data frame based on ini poses
                plot3(allInliers(:,1),allInliers(:,2),allInliers(:,3),[WallColors(iWall),'.']);
                if cntLines>1
                    % main axis
                    covariance=cov(allInliers);
                    [d v]=eig(covariance);
                    center=mean(allInliers);
%                     PlotAxes_TR('k','-',center,d,1000,3);
                    minX=min(minX,min(allInliers(:,1)));
                    minY=min(minY,min(allInliers(:,2)));
                    minZ=min(minZ,min(allInliers(:,3)));
                    maxX=max(maxX,max(allInliers(:,1)));
                    maxY=max(maxY,max(allInliers(:,2)));
                end
            end
        end
    end
    ErrorInOneGO(oneGO,NUMBER_LRFs,R_LRFsIni,T_LRFsIni);
%     ErrorInOneGO_AllLRFs(oneGO,R_LRFsIni,T_LRFsIni);
    
end
% toc

% ======================== Calib ======================================================
% Selecting GOs for LRF1_LRF2
    cntGOs_LRF1_LRF2=0; cntGOs_LRF1_LRF3=0;
if WithoutSelction==0
    clear GOs_LRF1_LRF2 cntGOs_LRF1_LRF3=0;
    FixedLRF=1; CalibLRF2=2; CalibLRF3=3;
    for iGO=1:size(GOs,2)
        % Tag the walls with line pairs (have both LRF1 line and LRF2 line)
        bWithLinePairs2=zeros(1,4);
        bWithLinePairs3=zeros(1,4);
        for iWall=1:4
            cntLines=GOs(iGO).GO(iWall).cntLines;
            lines=GOs(iGO).GO(iWall).lines;
            bFindLine_LRF1=0; bFindLine_LRF2=0; bFindLine_LRF3=0;
            if cntLines>=3
                for iLine=1:cntLines
                    iLRF=lines(iLine).idLRF;
                    if iLRF==FixedLRF
                        bFindLine_LRF1=1;
                    elseif iLRF==CalibLRF2
                        bFindLine_LRF2=1;
                    elseif iLRF==CalibLRF3
                        bFindLine_LRF3=1;
                    end
                end
            else
                continue;
            end
            if bFindLine_LRF1==1 && bFindLine_LRF2==1
                bWithLinePairs2(iWall)=1;
            end
            if bFindLine_LRF1==1 && bFindLine_LRF3==1
                bWithLinePairs3(iWall)=1;
            end
        end
        
        if sum(bWithLinePairs2)>=3
            cntGOs_LRF1_LRF2=cntGOs_LRF1_LRF2+1;
            GOs_LRF1_LRF2(cntGOs_LRF1_LRF2)=GOs(iGO);
        end
        if sum(bWithLinePairs3)>=3
            cntGOs_LRF1_LRF3=cntGOs_LRF1_LRF3+1;
            GOs_LRF1_LRF3(cntGOs_LRF1_LRF3)=GOs(iGO);
        end
        
        %     iSelectGO2=0; iSelectGO3=0;
        %     for iWall=1:4
        %         if bWithLinePairs2(iWall)==1 && bWithLinePairs2(mod(iWall,4)+1)==1
        %             iSelectGO2=1;
        %         end
        %         if bWithLinePairs3(iWall)==1 && bWithLinePairs3(mod(iWall,4)+1)==1
        %             iSelectGO3=1;
        %         end
        %     end
        %     if iSelectGO2==1
        %         cntGOs_LRF1_LRF2=cntGOs_LRF1_LRF2+1;
        %         GOs_LRF1_LRF2(cntGOs_LRF1_LRF2)=GOs(iGO);
        %     end
        %     if iSelectGO3==1
        %         cntGOs_LRF1_LRF3=cntGOs_LRF1_LRF3+1;
        %         GOs_LRF1_LRF3(cntGOs_LRF1_LRF3)=GOs(iGO);
        %     end
        
    end
    cntGOs_LRF1_LRF2
    cntGOs_LRF1_LRF3
end

%% Loop for repeat calibration      
ang_LRFsIni=ang_LRFsIni_Ori+deltaAng;
T_LRFsIni=T_LRFsIni_Ori+deltaT;
ang_LRFsIni_radian=ang_LRFsIni.*pi/180;
R_LRFsIni=zeros(3,3,3);
for i=1:3
    R_LRFsIni(i,:,:)=EularAngle2RotateMat(ang_LRFsIni_radian(i,1),ang_LRFsIni_radian(i,2),ang_LRFsIni_radian(i,3),'xyz');
end
errors2=0;
R_Calib=R_LRFsIni;
T_Calib=T_LRFsIni;
if WithoutSelction==0 && cntGOs_LRF1_LRF2>0
    [R_Calib,T_Calib,errors2,updates_ang_t_2] = Calib_2LRFs_Core_LM(GOs_LRF1_LRF2,NUMBER_LRFs,1,2,R_LRFsIni,T_LRFsIni);
    [R_Calib,T_Calib,errors3,updates_ang_t_3] = Calib_2LRFs_Core_LM(GOs_LRF1_LRF3,NUMBER_LRFs,1,3,R_Calib,T_Calib);
elseif WithoutSelction==1
    [R_Calib,T_Calib,errors2,updates_ang_t_2] = Calib_2LRFs_Core_LM(GOs,NUMBER_LRFs,1,2,R_LRFsIni,T_LRFsIni);
    [R_Calib,T_Calib,errors3,updates_ang_t_3] = Calib_2LRFs_Core_LM(GOs,NUMBER_LRFs,1,3,R_Calib,T_Calib);
end

% % Plot Errors
% figure,
% rotate3d on;
% h = [90 60 900 500];
% set(gcf,'Position',h)
% subplot(1,2,1),hold on;title('Erros-LRF1&LRF2');
% plot(errors2(1,:),'*-');
% xlabel('Number of iterations'); ylabel('error');
% subplot(1,2,2),hold on;title('Erros-LRF1&LRF3');
% plot(errors3(1,:),'*-');
% xlabel('Number of iterations'); ylabel('error');


% % save calibration result to file
R_LRFs=R_Calib; T_LRFs=T_Calib;
for iLRF=1:NUMBER_LRFs
    ang_LRFs(iLRF,:)=RotateMat2EularAngle_XYZ(squeeze(R_LRFs(iLRF,:,:)));
end

fileName_LRFsCalibPos='LRFsCalibPos';
fileFullPath_LRFsCalibPos=[RawFilePath fileName_LRFsCalibPos '.mat'];
save(fileFullPath_LRFsCalibPos,'ang_LRFs','R_LRFs','T_LRFs');
ang_LRFs
T_LRFs
toc

Results(cntTries).ang_LRFsIni=ang_LRFsIni;
Results(cntTries).T_LRFsIni=T_LRFsIni;
Results(cntTries).deltaAng=deltaAng;
Results(cntTries).deltaT=deltaT;
Results(cntTries).ang_LRFs=ang_LRFs;
Results(cntTries).R_LRFs=R_LRFs;
Results(cntTries).T_LRFs=T_LRFs;
Results(cntTries).errors2=errors2;
% Results(cntTries).errors3=errors3;

end

if WithoutSelction==0
    fileName_BatchResults=['BatchResults-',num2str(iRotationOperation),'-step',num2str(iFrameStep)];
else
    fileName_BatchResults=['BatchResults-',num2str(iRotationOperation),'-step',num2str(iFrameStep),...
        '-WithoutSelection','-CorridorSize',num2str(iCorridorSize)];
end
fileFullPath_BatchResults=[RawFilePath fileName_BatchResults '.mat'];
save(fileFullPath_BatchResults,'Results');

end
end
end
end
% end
% end
% end
% end
% end
% end
% end







