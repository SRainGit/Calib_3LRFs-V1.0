%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% the master program of the extrinsic calibration of 3 LRFs, and it also
%%% do some preprocessing for the IMU boresight calibration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc; clear;
global COs

bMyData=0;
% bMyData=1;

NUMBER_LRFs=3;
if bMyData
    PTS_PER_FRAME=1081;
else
    PTS_PER_FRAME=1080;
end
half_PI=pi/2;

tic
% set file path
disp('Loading data ...')
% RawFilePath = 'E:\SLAM\Data\LaserData\20180203\';
% fileName_RawData='UTM30LX_0_20180203_151541';
% RawFilePath = 'D:\LaserData\20180629\data\';
% fileName_RawData='4.bag_0';
% RawFilePath = 'E:\SLAM\Data\20180802\';
% RawFilePath = 'Data\20180802\';
% RawFilePath = 'E:\SLAM\Data\20180928\';
RawFilePath = 'E:\SLAM\Matlab\Calib_3LRFs-V1.0\Data\20180928\';
% fileName_RawData='4_0';
% RawFilePath = 'E:\SLAM\Data\20180828_sim\';
% fileName_RawData='7_0';



iStartFrame=1; % set the start index of the group
iFrameStep=20; % generally, 12, 20 and 40 are recommended
iEndFrame=-1; % if indexOfEndFrame=-1, then it will be the number of all groups

bCOSelection=0; % take all the sampled COs as input
bCOSelection=1; % select the COs with adjacent surfaces as input

% fileName_RawData_list={'1_0','2_0','3_0'};
fileName_RawData_list={'1_0'};
for fileName_RawData=fileName_RawData_list
    
cntTries=0;
for iRepeat=1:1:1
    COs=[];
    
% iRotationOperation=5;
% iCorridorSize=1;
% SyntheticDataGenerator_Master(iRotationOperation,iFrameStep,iCorridorSize)

cntTries=cntTries+1;
fileName_RawData=char(fileName_RawData);

% fileName_Data=['RawPC_' fileName_RawData];
fileName_Data=['Data_' fileName_RawData];
fileName_LRFsIniPos='LRFsIniPos';
fileFullPath_Data=[RawFilePath fileName_Data '.mat'];
fileFullPath_LRFsIniPos=[RawFilePath fileName_LRFsIniPos '.mat'];

% load raw PC(point cloud) data
Data=load(fileFullPath_Data);
PC_Raw=Data.PC_Raw;
nDataGroups=size(PC_Raw,2);

% load iniPos parameters
LRFsIniPos=load(fileFullPath_LRFsIniPos);
ang_LRFsIni=LRFsIniPos.ang_LRFsIni;
T_LRFsIni=LRFsIniPos.T_LRFsIni;

ang_LRFsIni_Ori=LRFsIniPos.ang_LRFsIni;
T_LRFsIni_Ori=LRFsIniPos.T_LRFsIni;

ang_LRFsIni_radian=ang_LRFsIni.*pi/180;
R_LRFsIni=zeros(3,3,3);
for i=1:3
    R_LRFsIni(i,:,:)=EulerAngle2RotateMat(ang_LRFsIni_radian(i,1),ang_LRFsIni_radian(i,2),ang_LRFsIni_radian(i,3),'xyz');
end

% device poses
R_DevicePoses=zeros(nDataGroups,3,3);
T_DevicePoses=zeros(nDataGroups,3);
% toc


%% ============= RANSAC for fitting 2D lines and find coplanar line pairs ===========================
% tic
% parameters for lnie detection based on RANSAC
nearestDist=0.500; % the laser point too near are set to zeros point, meters
% farthestDist=4.5*1.000;  % the laser point too far are set to zeros point, meters
farthestDist=6*1.000; % the laser point too near are set to zeros point, meters
distThreshold=0.010; % threshold for line deteciton (outliers/inliers)
minLineLength=0.600; % the accacptable minimum line length of the lines detected
minInliersPts=150; % the accacptable minimum number of inliers of the lines detected
maxTotalIterTimes=1000; % maximum number of iterations for line detection
fixedInerIterTimes=100; % the fixed number of inner iterations for finding the better model
dist2DoubleLine=0.2; % if their distance(approximation) is small enough

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
bShowLineDetectionResult=0;
% bShowLineDetectionResult=1;
if bShowFigure
    fig1=figure; h = [-1500 110 1500 980]; set(gcf,'Position',h)
%         figure, h = [0 0 1500 980]; set(gcf,'Position',h)
    h1=zeros(1,4);
    rotate3d on 
    lineColors=['r','g','b','m','c'];
    LRFColors=['r','g','b'];
    lineNorm=[0,0,0]; vector=[0,0,0]; centerPt=[0,0,0]; basePt=[0,0,0];%basePt is for plot vector
    vectorLineLength=2; % the line lenth to plot arrows
end
centerPt1=[0,0,0];centerPt2=[0,0,0];vector1=[0,0,0];vector2=[0,0,0]; distance=0;
basePt_2D=[0,0];
cntSelectedGroups=0;
if iEndFrame==-1
    iEndFrame=nDataGroups;
end
% for each group data; each group data include 3LRF's data
for iGroup=iStartFrame:iFrameStep:iEndFrame
    disp(['iGroup=' num2str(iGroup)]);
    minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
    for iLRF=1:NUMBER_LRFs
        %% Detecting lines by RANSACгм for each LRF
        data=squeeze(PC_Raw(iLRF,iGroup,:,:));
        % detecting
        [cntLines,lines,nInliers,inliers,outliers] = ...
            RANSAC_DetectLines_2D(data(:,1:2),size(data,1),nearestDist,farthestDist, ...
            distThreshold, minLineLength, minInliersPts, maxTotalIterTimes,fixedInerIterTimes);
        % preprocess of vectos, center points etc.
        iLine=1;
        while iLine<=cntLines
            centerPt(1)=mean(inliers(iLine,1:nInliers(iLine),1));
            centerPt(2)=mean(inliers(iLine,1:nInliers(iLine),2));
            centerPt(3)=0;            
            lineNorm=[-lines(iLine,2),lines(iLine,1),0];
            vector=lineNorm/norm(lineNorm); % line vector     
            % save the line information to the struct data (for display and coplanar line detection)
            LRF_Lines(iLRF,iGroup).lines(iLine).idGroup=iGroup; % the index of the group
            LRF_Lines(iLRF,iGroup).lines(iLine).idLRF=iLRF; % the index of this line in the all lines
            LRF_Lines(iLRF,iGroup).lines(iLine).coefficients=squeeze(lines(iLine,:)); %
            LRF_Lines(iLRF,iGroup).lines(iLine).nInliers=nInliers(iLine);
            LRF_Lines(iLRF,iGroup).lines(iLine).inliers=squeeze(inliers(iLine,1:nInliers(iLine),:));
            LRF_Lines(iLRF,iGroup).lines(iLine).centerPt=centerPt;
            LRF_Lines(iLRF,iGroup).lines(iLine).vector=vector;
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
            [minValue,iMin]=min(projection);   
            [maxValue,iMax]=max(projection);
            if minValue<-farthestDist
                minValue=-farthestDist;
            end
            if maxValue>farthestDist
                maxValue=farthestDist;
            end
            basePt_2D=[-lines(iLine,1),-lines(iLine,2)].*lines(iLine,3);
            LRF_Lines(iLRF,iGroup).lines(iLine).endPts(1,:)=basePt_2D+minValue*vector(1,1:2);
            LRF_Lines(iLRF,iGroup).lines(iLine).endPts(2,:)=basePt_2D+maxValue*vector(1,1:2);
            LRF_Lines(iLRF,iGroup).lines(iLine).lineLength=maxValue-minValue;
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
                        if norm(cross(vector1,(centerPt1-centerPt2)))<dist2DoubleLine
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
            return;
        end
        % record the idWalls
        LRF_Lines(iLRF,iGroup).idWalls=idWalls;
        
        %% figure
        % line detection result of each LRF
        if bShowLineDetectionResult
            h1(iLRF)=subplot(2,2,iLRF); hold on, axis equal;
            cla(h1(iLRF));
            offset=0.500;
            % outliers
            plot(outliers(:,1),outliers(:,2),'k.');
            xlim([min(data(:,1))-offset  max(data(:,1))+offset]);
            ylim([min(data(:,2))-offset  max(data(:,2))+offset]);
            % inliers and lines
            if cntLines>0
                title([num2str(cntLines), ' lines in ','LRF', num2str(iLRF)]);
                for iLine=1:cntLines
                    inliers=LRF_Lines(iLRF,iGroup).lines(iLine).inliers;
                    coefficients=LRF_Lines(iLRF,iGroup).lines(iLine).coefficients;
                    plot(inliers(:,1),inliers(:,2),[lineColors(iLine),'.']);
                    x=min(data(:,1)):0.01:max(data(:,1));
                    y=(coefficients(1).*x+coefficients(3))./(-coefficients(2));
%                     plot(x,y,[lineColors(iLine),'-']);
                    centerPt=LRF_Lines(iLRF,iGroup).lines(iLine).centerPt;
                    plot(centerPt(1),centerPt(2),'ko','LineWidth',3);
                end
            end
            xlabel('X/mm'); ylabel('Y/mm');
            hold off;
            % show the result of fused point cloud
            h1(4)=subplot(2,2,4);
            if iLRF==1
                cla(h1(4));
            end
            hold on, axis equal;
            % point clouds with initial poses
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
            plot3(PC_IniPos(:,1),PC_IniPos(:,2),PC_IniPos(:,3),[LRFColors(iLRF),'.'],'LineWidth',0.1);
            % plot the center points and the vectors of the detected lines
            % in 3D space with initial poses
            if cntLines>0
                for iLine=1:cntLines
                    centerPt=LRF_Lines(iLRF,iGroup).lines(iLine).centerPt;
                    vector=0.01*LRF_Lines(iLRF,iGroup).lines(iLine).vector*LRF_Lines(iLRF,iGroup).lines(iLine).nInliers;
                    
                    centerPt(3)=0;
                    centerPt=(squeeze(R_LRFsIni(iLRF,:,:))*centerPt')'+T_LRFsIni(iLRF,:);
                    vector=(squeeze(R_LRFsIni(iLRF,:,:))*vector')';
                    
                    plot3(centerPt(1),centerPt(2),centerPt(3),'ko','LineWidth',3);
                    
                    basePt=centerPt-0.5.*vector;
                    quiver3(basePt(:,1),basePt(:,2),basePt(:,3)...
                        ,vector(:,1),vector(:,2),vector(:,3),'k','LineWidth',3);
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
        cntAllLines=cntAllLines+LRF_Lines(iLRF,iGroup).cntLines;
    end
    if cntAllLines<7 || bBadLineDetection==1
        disp('Not enough lines.');
        cntSelectedGroups=cntSelectedGroups-1;
        continue;
    end
    
    %% Generate all possible COs in this dataGroup, and find a best one
    cntCOs=0;
    clear COsInOneFrame;
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
                    cntCOs=cntCOs+1;
                    % initiate COs(cntCOs)
                    for iWall=1:4
                        COsInOneFrame(cntCOs,iWall).cntLines=0;
%                         a = rmfield(COsInOneFrame(cntCOs,iWall),'lines');
                    end
                    % Fix the lines of LRF1, take the line number as the wall number
                    for iLine1=1:LRF_Lines(1,iGroup).cntLines
                        idWalls=LRF_Lines(1,iGroup).idWalls;
                        iWall=idWalls(iLine1);
                        COsInOneFrame(cntCOs,iWall).cntLines=COsInOneFrame(cntCOs,iWall).cntLines+1;
                        COsInOneFrame(cntCOs,iWall).lines(COsInOneFrame(cntCOs,iWall).cntLines,:)=LRF_Lines(1,iGroup).lines(iLine1);
                    end
                    % arrange the lines of LRF2
                    for iLine2=1:LRF_Lines(2,iGroup).cntLines
                        idWalls=LRF_Lines(2,iGroup).idWalls;
                        iWall=mod(i2*(offset2+idWalls(iLine2))-1,4)+1;
                        COsInOneFrame(cntCOs,iWall).cntLines=COsInOneFrame(cntCOs,iWall).cntLines+1;
                        COsInOneFrame(cntCOs,iWall).lines(COsInOneFrame(cntCOs,iWall).cntLines,:)=LRF_Lines(2,iGroup).lines(iLine2);
                    end
                    % arrange the lines of LRF3
                    for iLine3=1:LRF_Lines(3,iGroup).cntLines
                        idWalls=LRF_Lines(3,iGroup).idWalls;
                        iWall=mod(i3*(offset3+idWalls(iLine3))-1,4)+1;
                        COsInOneFrame(cntCOs,iWall).cntLines=COsInOneFrame(cntCOs,iWall).cntLines+1;
                        COsInOneFrame(cntCOs,iWall).lines(COsInOneFrame(cntCOs,iWall).cntLines,:)=LRF_Lines(3,iGroup).lines(iLine3);
                    end
                end
            end
        end
    end
    % compute the errors in all the COs
%     t1=clock;
    errorsCO=zeros(cntCOs,1);
    for iCO=1:cntCOs
        oneCO.indexDataGroup=iGroup;
        oneCO.CO=COsInOneFrame(iCO,:);
        errorsCO(iCO)=ErrorInOneCO_AllLRFs(oneCO,R_LRFsIni,T_LRFsIni);
    end
%     t2=clock;
%     disp([num2str(etime(t2,t1)) ' seconds']);
    
    %% Input the CO with minimal error to the linePairs
    [minErrorCO,iMin]=min(errorsCO);
    COs(cntSelectedGroups).indexDataGroup=iGroup;
    COs(cntSelectedGroups).CO=COsInOneFrame(iMin,:);
    
    
    %% Show the CO with minimal error
    if bShowFigure
        if ~exist('h_miniError_CO')
            h_miniError_CO=figure, h = [-900 110 800 600]; set(gcf,'Position',h);
            hold on; rotate3d on; axis equal;
        else
            cla(h_miniError_CO);
        end
        WallColors=['r','g','b','m','c'];
        minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
        OneCO=COsInOneFrame(iMin,:);
        title(num2str(minErrorCO));
        for iWall=1:4
            % extract data
            cntLines=OneCO(iWall).cntLines;
            lines=OneCO(iWall).lines;
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
                    PlotAxes_TR('k','-',center,d,1,3);
                    minX=min(minX,min(allInliers(:,1)));
                    minY=min(minY,min(allInliers(:,2)));
                    minZ=min(minZ,min(allInliers(:,3)));
                    maxX=max(maxX,max(allInliers(:,1)));
                    maxY=max(maxY,max(allInliers(:,2)));
                end
            end
        end
        thisError=ErrorInOneCO_AllLRFs(COs(cntSelectedGroups),R_LRFsIni,T_LRFsIni);
        cla(h_miniError_CO);
    end
    
end
% toc


%% ======================== Calib ======================================================
R_Calib=R_LRFsIni; T_Calib=T_LRFsIni;
%% align the wall indexes in all COs
cnt0=0;
for iCO=2:cntSelectedGroups
    % import the previous group
    for iWall=1:4
        oneFusionCO.CO(iWall).cntLines=COs(iCO-1).CO(iWall).cntLines;
        oneFusionCO.CO(iWall).lines=COs(iCO-1).CO(iWall).lines;
    end
    oneFusionCO_bkp=oneFusionCO;
    % find out which wall index offset is to the minimum error
    minError=10^9;
    finalDirection=1; finalOffset=0;
    for iOffsetDirection=-1:2:1
        for iOffset=0:3
            oneFusionCO=oneFusionCO_bkp;
            for iWall=1:4
                cnt0=oneFusionCO.CO(iWall).cntLines;
                cnt1=COs(iCO).CO(mod(iOffsetDirection*(iWall+iOffset)-1,4)+1).cntLines;
                if cnt1>0
                    oneFusionCO.CO(iWall).cntLines = cnt0+cnt1;
                    if cnt0==0
                        oneFusionCO.CO(iWall).lines=...
                            COs(iCO).CO(mod(iOffsetDirection*(iWall+iOffset)-1,4)+1).lines;
                    else
                        oneFusionCO.CO(iWall).lines(cnt0+1:cnt0+cnt1)=...
                            COs(iCO).CO(mod(iOffsetDirection*(iWall+iOffset)-1,4)+1).lines;
                    end
                end
            end
            currentError=ErrorInOneCO_AllLRFs(oneFusionCO,R_Calib,T_Calib);
            if currentError<minError
                minError=currentError;
                finalDirection=iOffsetDirection;
                finalOffset=iOffset;
            end
        end
    end
    % set the right sequnce
    if finalDirection==1 && finalOffset==0
        continue;
    end
    oneCO.CO=COs(iCO).CO;
    for iWall=1:4
        COs(iCO).CO(iWall)=oneCO.CO(mod(finalDirection*(iWall+finalOffset)-1,4)+1);
    end
end
% get corner points in each CO
for iCO=1:cntSelectedGroups
    OneCO=COs(iCO);
    for iWall0=1:4
        cntCorners=0;
        COs(iCO).CO(iWall0).cntCorners=0;
        
        iWall1=mod(iWall0+1-1,4)+1;
        cntLines0=OneCO.CO(iWall0).cntLines;
        lines0=OneCO.CO(iWall0).lines;
        cntLines1=OneCO.CO(iWall1).cntLines;
        lines1=OneCO.CO(iWall1).lines;
        if cntLines0<1 && cntLines1<1
            continue;
        end
        
        for iLine0=1:cntLines0
            iLRF0=lines0(iLine0).idLRF;
            for iLine1=1:cntLines1
                iLRF1=lines1(iLine1).idLRF;
                if iLRF0==iLRF1
                    line0=lines0(iLine0).coefficients;
                    line1=lines1(iLine1).coefficients;
                    A=[line0(1:2);line1(1:2)];
                    B=[-line0(3);-line1(3)];
                    X=A\B;
                    cntCorners=cntCorners+1;
                    COs(iCO).CO(iWall0).corners(cntCorners).idLRF=iLRF0;
                    COs(iCO).CO(iWall0).corners(cntCorners).Pt=[X;0];
                    break;
                end
            end
        end
        COs(iCO).CO(iWall0).cntCorners=cntCorners;
    end
end 



% show the aligning result (before the pose estimation of each frame)
bShowFigure=0;
% bShowFigure=1;
if bShowFigure
    fig_alignResult=figure;
    h = [-1500 110 1500 980]; 
%     h = [100 10 1500 980]; 
    set(gcf,'Position',h)
    rotate3d on; hold on; axis equal;
    WallColors=['r','g','b','m','c'];
    LRFColors=['r','g','b'];
    for iCO=1:cntSelectedGroups  % to distinguish 'iGroup' & 'iiGroup'
        oneCO
        oneCO=COs(iCO);
        R_iGroup=eye(3,3);
        T_iGroup=[0,0,0]';
        for iWall=1:4
            for iLine=1:oneCO.CO(iWall).cntLines
                oneLine=oneCO.CO(iWall).lines(iLine);
                idGroup=oneLine.idGroup;
                idLRF=oneLine.idLRF;
                endPts=oneLine.endPts;
                endPts(:,3)=0;
                R_device=squeeze(R_Calib(idLRF,:,:));
                T_device=squeeze(T_Calib(idLRF,:))';
                [R,T]=PoseAddition_RT(R_iGroup,T_iGroup,R_device,T_device);
                endPts=(R*endPts')'+[T';T'];
                line(endPts(:,1),endPts(:,2),endPts(:,3),'color',WallColors(iWall),'LineStyle','-')
            end
        end
        % end of oneCO
        if rem(iCO,2)==0
            cla(fig_alignResult);
        end
    end
end


%% Calib LRFs
COs_bkp=COs;
% Selecting COs for LRF1_LRF2
    cntCOs_LRF1_LRF2=0; cntCOs_LRF1_LRF3=0;
if bCOSelection==1
    clear COs_LRF1_LRF2 cntCOs_LRF1_LRF3=0;
    FixedLRF=1; CalibLRF2=2; CalibLRF3=3;
    for iCO=1:size(COs,2)
        % Tag the walls with line pairs (have both LRF1 line and LRF2 line)
        bWithLinePairs2=zeros(1,4);
        bWithLinePairs3=zeros(1,4);
        for iWall=1:4
            cntLines=COs(iCO).CO(iWall).cntLines;
            lines=COs(iCO).CO(iWall).lines;
            bFindLine_LRF1=0; bFindLine_LRF2=0; bFindLine_LRF3=0;
            if cntLines>=2
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
        
%         if sum(bWithLinePairs2)>=3
%             cntCOs_LRF1_LRF2=cntCOs_LRF1_LRF2+1;
%             COs_LRF1_LRF2(cntCOs_LRF1_LRF2)=COs(iCO);
%         end
%         if sum(bWithLinePairs3)>=3
%             cntCOs_LRF1_LRF3=cntCOs_LRF1_LRF3+1;
%             COs_LRF1_LRF3(cntCOs_LRF1_LRF3)=COs(iCO);
%         end
        
            iSelectCO2=0; iSelectCO3=0;
            for iWall=1:4
                if bWithLinePairs2(iWall)==1 && bWithLinePairs2(mod(iWall,4)+1)==1
                    iSelectCO2=1;
                end
                if bWithLinePairs3(iWall)==1 && bWithLinePairs3(mod(iWall,4)+1)==1
                    iSelectCO3=1;
                end
            end
            if iSelectCO2==1
                cntCOs_LRF1_LRF2=cntCOs_LRF1_LRF2+1;
                GoodCOs2(cntCOs_LRF1_LRF2)=COs(iCO);
            end
            if iSelectCO3==1
                cntCOs_LRF1_LRF3=cntCOs_LRF1_LRF3+1;
                GoodCOs3(cntCOs_LRF1_LRF3)=COs(iCO);
            end
        
    end
end   
    
% [R_Calib,T_Calib,errors2] = Calib_2LRFs_Core_SimplexAlgorithm(COs,NUMBER_LRFs,1,2,R_LRFsIni,T_LRFsIni);
% [R_Calib,T_Calib,errors3] = Calib_2LRFs_Core_SimplexAlgorithm(COs,NUMBER_LRFs,1,3,R_Calib,T_Calib);
errors2=0;errors3=0;updates_ang_t_2=0;updates_ang_t_3=0;
R_Calib=R_LRFsIni;
T_Calib=T_LRFsIni;
if bCOSelection==0
%     [R_Calib,T_Calib,errors2,updates_ang_t_2] = Calib_2LRFs_Core_LM(NUMBER_LRFs,1,2,R_LRFsIni,T_LRFsIni);
%     [R_Calib,T_Calib,errors3,updates_ang_t_3] = Calib_2LRFs_Core_LM(NUMBER_LRFs,1,3,R_Calib,T_Calib);
    [R_Calib,T_Calib,errors2,updates_ang_t_2] = Calib_2LRFs_Core_SimplexAlgorithm(NUMBER_LRFs,1,2,R_LRFsIni,T_LRFsIni);
    [R_Calib,T_Calib,errors3,updates_ang_t_3] = Calib_2LRFs_Core_SimplexAlgorithm(NUMBER_LRFs,1,3,R_Calib,T_Calib);
elseif bCOSelection==1 && cntCOs_LRF1_LRF2>0
    cntCOs_LRF1_LRF2
    cntCOs_LRF1_LRF3
    COs=GoodCOs2;
%     [R_Calib,T_Calib,errors2,updates_ang_t_2] = Calib_2LRFs_Core_LM(NUMBER_LRFs,1,2,R_LRFsIni,T_LRFsIni);
    [R_Calib,T_Calib,errors2,updates_ang_t_2] = Calib_2LRFs_Core_SimplexAlgorithm(NUMBER_LRFs,1,2,R_LRFsIni,T_LRFsIni);
    COs=GoodCOs3;
%     [R_Calib,T_Calib,errors3,updates_ang_t_3] = Calib_2LRFs_Core_LM(NUMBER_LRFs,1,3,R_Calib,T_Calib);
    [R_Calib,T_Calib,errors3,updates_ang_t_3] = Calib_2LRFs_Core_SimplexAlgorithm(NUMBER_LRFs,1,3,R_Calib,T_Calib);
end
COs=COs_bkp;

% % Plot Errors
% figure,
% rotate3d on;
% h = [-1800 600 900 500];
% set(gcf,'Position',h)
% subplot(1,2,1),hold on;title('Erros-LRF1&LRF2');
% plot(errors2,'*-');
% xlabel('Number of iterations'); ylabel('error');
% subplot(1,2,2),hold on;title('Erros-LRF1&LRF3');
% plot(errors3,'*-');
% xlabel('Number of iterations'); ylabel('error');
% 
% figure,
% h = [-900 240 800 900];
% set(gcf,'Position',h)
% subplot(4,3,1),hold on;title('Pitch-Roll');
% plot(updates_ang_t_2(:,1),updates_ang_t_2(:,2),'*-');
% xlabel('Pitch(бу)'); ylabel('Roll(бу)'); axis equal;
% subplot(4,3,2),hold on;title('Roll-Yaw');
% plot(updates_ang_t_2(:,2),updates_ang_t_2(:,3),'*-');
% xlabel('Roll(бу)'); ylabel('Yaw(бу)'); axis equal;
% subplot(4,3,3),hold on;title('Pitch-Yaw');
% plot(updates_ang_t_2(:,1),updates_ang_t_2(:,3),'*-');
% xlabel('Pitch(бу)'); ylabel('Yaw(бу)'); axis equal;
% subplot(4,3,4),hold on;title('X-Y');
% plot(updates_ang_t_2(:,4),updates_ang_t_2(:,5),'*-');
% xlabel('X(mm)'); ylabel('Y(mm)'); axis equal;
% subplot(4,3,5),hold on;title('Y-Z');
% plot(updates_ang_t_2(:,5),updates_ang_t_2(:,6),'*-');
% xlabel('Y(mm)'); ylabel('Z(mm)'); axis equal;
% subplot(4,3,6),hold on;title('X-Z');
% plot(updates_ang_t_2(:,4),updates_ang_t_2(:,6),'*-');
% xlabel('X(mm)'); ylabel('Z(mm)'); axis equal;
% subplot(4,3,7),hold on;title('Pitch-Roll');
% plot(updates_ang_t_3(:,1),updates_ang_t_3(:,2),'*-');
% xlabel('Pitch(бу)'); ylabel('Roll(бу)'); axis equal;
% subplot(4,3,8),hold on;title('Roll-Yaw');
% plot(updates_ang_t_3(:,2),updates_ang_t_3(:,3),'*-');
% xlabel('Roll(бу)'); ylabel('Yaw(бу)'); axis equal;
% subplot(4,3,9),hold on;title('Pitch-Yaw');
% plot(updates_ang_t_3(:,1),updates_ang_t_3(:,3),'*-');
% xlabel('Pitch(бу)'); ylabel('Yaw(бу)'); axis equal;
% subplot(4,3,10),hold on;title('X-Y');
% plot(updates_ang_t_3(:,4),updates_ang_t_3(:,5),'*-');
% xlabel('X(mm)'); ylabel('Y(mm)'); axis equal;
% subplot(4,3,11),hold on;title('Y-Z');
% plot(updates_ang_t_3(:,5),updates_ang_t_3(:,6),'*-');
% xlabel('Y(mm)'); ylabel('Z(mm)'); axis equal;
% subplot(4,3,12),hold on;title('X-Z');
% plot(updates_ang_t_3(:,4),updates_ang_t_3(:,6),'*-');
% xlabel('X(mm)'); ylabel('Z(mm)'); axis equal;



% save calibration result to file
R_LRFs=R_Calib; T_LRFs=T_Calib;
for iLRF=1:NUMBER_LRFs
    ang_LRFs(iLRF,:)=RotateMat2EulerAngle_XYZ(squeeze(R_LRFs(iLRF,:,:)));
end

fileName_LRFsCalibPos='LRFsCalibPos';
fileFullPath_LRFsCalibPos=[RawFilePath fileName_LRFsCalibPos '.mat'];
save(fileFullPath_LRFsCalibPos,'ang_LRFs','R_LRFs','T_LRFs');
ang_LRFs
T_LRFs


fileFullPath_GOs=[RawFilePath, 'COs_', fileName_RawData, '.mat'];
save(fileFullPath_GOs,'COs');

toc

Results(cntTries).ang_LRFsIni=ang_LRFsIni;
Results(cntTries).T_LRFsIni=T_LRFsIni;
% Results(cntTries).deltaAng=deltaAng;
% Results(cntTries).deltaT=deltaT;
Results(cntTries).ang_LRFs=ang_LRFs;
Results(cntTries).R_LRFs=R_LRFs;
Results(cntTries).T_LRFs=T_LRFs;
Results(cntTries).errors2=errors2;
Results(cntTries).errors3=errors3;
Results(cntTries).updates_ang_t_2=updates_ang_t_2;
Results(cntTries).updates_ang_t_3=updates_ang_t_3;

end

fileName_BatchResults=['BatchResults-',fileName_RawData];
fileFullPath_BatchResults=[RawFilePath fileName_BatchResults '.mat'];
save(fileFullPath_BatchResults,'Results');

end








