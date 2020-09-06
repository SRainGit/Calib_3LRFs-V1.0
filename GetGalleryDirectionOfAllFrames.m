close all; clc; clear;

NUMBER_LRFs=3;
PTS_PER_FRAME=1081;
% galleryWidth=2400+12*2;%mm
% galleryHeight=2290;%mm
galleryWidth=2450;%mm
galleryHeight=2320;%mm

tic
% set file path
disp('Loading data ...')
% RawFilePath = 'D:\LaserData\20170830\';
% fileName_RawPC=['RawPC_' 'UTM30LX_0_20170830_144433'];
RawFilePath = 'D:\LaserData\20171113\';
fileName_RawPC0='UTM30LX_0_20171113_210018';
fileName_RawPC=['RawPC_' fileName_RawPC0];
fileName_LRFsIniPos='LRFsIniPos';
fileName_LRFsCalibPos='LRFsCalibPos';
fileFullPath_RawPC=[RawFilePath fileName_RawPC '.mat'];
fileFullPath_LRFsIniPos=[RawFilePath fileName_LRFsIniPos '.mat'];
fileFullPath_LRFsCalibPos=[RawFilePath fileName_LRFsCalibPos '.mat'];

% load raw PC(point cloud) data
PC_Raw=load(fileFullPath_RawPC);
PC_Raw=PC_Raw.PC_Raw;
nDataGroups=size(PC_Raw,2);

% load iniPos parameters
LRFsIniPos=load(fileFullPath_LRFsIniPos);
LRFsCalibPos=load(fileFullPath_LRFsCalibPos);
% ang_LRFs=LRFsIniPos.ang_LRFsIni; % use the ini Pos
% T_LRFs=LRFsIniPos.T_LRFsIni;
ang_LRFs=LRFsCalibPos.ang_LRFs; % use the calib Pos
T_LRFs=LRFsCalibPos.T_LRFs;
ang_LRFs_radian=ang_LRFs.*pi/180;
R_LRFs=zeros(3,3,3);
for i=1:3
    R_LRFs(i,:,:)=EularAngle2RotateMat(ang_LRFs_radian(i,1),ang_LRFs_radian(i,2),ang_LRFs_radian(i,3),'xyz');
end

% device poses
R_D2Ws=zeros(nDataGroups,3,3);
T_D2Ws=zeros(nDataGroups,3);
toc


%% ============= RANSAC for fitting 2D lines and find coplanar line pairs ===========================
tic
disp('RANSAC...')
% parameters for lnie detection based on RANSAC
nearestDist=500;%设定最近的视野范围，mm
farthestDist=6*1000;%设定最远的视野范围，mm
distThreshold=40;%距离阈值，超出该阈值认为是outliers，单位mm
minLineLength=200;%提取出线段的最短距离，实质用作采集两点的时候两点间的最短距离，用于提升检测精度
minInliersPts=120;
maxTotalIterTimes=1000;
fixedInerIterTimes=100;%固定的内循环次数，用于寻找尽量最优的model

% parameters and variables for detectong coplanar line segments
linesDistThreshold=200; % the distance between coplanar lines should smaller than this value ( for first step detection)
maxLinesDist=1000; % the distance between coplanar lines should smaller than this value ( for second step detection)
allLines_cnt=zeros(1,3); % record the number of allt the lines detected in the all groups; 3 number for 3LRFs
allLines_CorePt=zeros(3,100,3); % (iLRF,iLine,:)
allLines_Vector=zeros(3,100,3); % the number of the second dimension will changes as the line detection going on

% coplanar line pairs
cntLinePairs(3,3)=0;
linePairsIndex=zeros(3,3,10,3);
linePairs=zeros(3,3,10,3,3);
% orthogonality line pairs
cntOrthogonalityPairs=0;

isShowFigure = 0 ; % It is used to control whether the drawing is opened.
% isShowFigure = 1 ; % It is used to control whether the drawing is opened.
if isShowFigure
    figure, h = [-1500 110 1500 980]; set(gcf,'Position',h)
    %     figure, h = [0 0 1500 980]; set(gcf,'Position',h)
    rotate3d on
    lineColors=['r','g','b','m','c'];
    LRFColors=['c','g','b'];
    lineNorm=[0,0,0]; vector=[0,0,0]; corePt=[0,0,0]; basePt=[0,0,0];%basePt is for plot vector
    vectorLineLength=2000;%用于绘制箭头时箭头的长度
end

corePt1=[0,0,0];corePt2=[0,0,0];vector1=[0,0,0];vector2=[0,0,0]; distance=0;
syms x y z real
% for each group data; each group data include 3LRF's data
% for iGroup=90:1:360
% for iGroup=164
for iGroup=1:nDataGroups
    disp(['第' num2str(iGroup) '组']);
    
    %% Detecting lines by RANSAC， for each LRF
    minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
    for iLRF=1:NUMBER_LRFs
        data=squeeze(PC_Raw(iLRF,iGroup,:,:));
        %检测
        [cntLines,lines,nInliers,inliers,outliers] = ...
            RANSAC_DetectLines_2D(data(:,1:2),size(data,1),nearestDist,farthestDist, ...
            distThreshold, minLineLength, minInliersPts, maxTotalIterTimes,fixedInerIterTimes);
        
        %预处理检测出的直线：法向量、中点等信息
        for iLine=1:cntLines
            %解算直线信息：重心和法向量；并根据初试位姿将其变换到Device坐标系下
            corePt(1)=mean(inliers(iLine,1:nInliers(iLine),1));%每条线的重心，根据inliers计算
            corePt(2)=mean(inliers(iLine,1:nInliers(iLine),2));
            corePt(3)=0;
            lineNorm=[-lines(iLine,2),lines(iLine,1),0];
            vector=lineNorm/norm(lineNorm);%每条线的法向量
            
            % save the line information to the data array (preparing for batch process)
            allLines_cnt(iLRF)=allLines_cnt(iLRF)+1;
            allLines_CorePt(iLRF,allLines_cnt(iLRF),:)=corePt;
            allLines_Vector(iLRF,allLines_cnt(iLRF),:)=vector;
            
            % save the line information to the struct data (for display and coplanar line detection)
            LRF_Lines(iLRF,iGroup).lines(iLine).index=allLines_cnt(iLRF); % the index of this line in the all lines
            LRF_Lines(iLRF,iGroup).lines(iLine).nInliers=nInliers(iLine);%每条线的inliers数量
            LRF_Lines(iLRF,iGroup).lines(iLine).inliers=inliers(iLine,1:nInliers(iLine),:);%每条线的inliers数量
            LRF_Lines(iLRF,iGroup).lines(iLine).corePt=corePt;
            LRF_Lines(iLRF,iGroup).lines(iLine).vector=vector;%每条线的法向量
        end
        
        % remove the double line at the top of gallery
        if cntLines>=2
            iLine1=1;
            while iLine1<=cntLines-1
                iLine2=iLine1+1;
                while iLine2<=cntLines
                    corePt1=LRF_Lines(iLRF,iGroup).lines(iLine1).corePt;
                    vector1=LRF_Lines(iLRF,iGroup).lines(iLine1).vector;
                    corePt2=LRF_Lines(iLRF,iGroup).lines(iLine2).corePt;
                    vector2=LRF_Lines(iLRF,iGroup).lines(iLine2).vector;
                    if norm(cross(vector1,vector2))<sin(10*pi/180) % if their direction are close to the same
                        if norm(cross(vector1,(corePt1-corePt2)))<200 % if their distance(approximation) is small enough
                            dist2ZeroPt1=abs(dot(corePt1,[vector1(2),-vector1(1),0]));
                            dist2ZeroPt2=abs(dot(corePt2,[vector2(2),-vector2(1),0]));
                            if dist2ZeroPt1<dist2ZeroPt2 % remv the further one, and take its inliers as outliers
                                LRF_Lines(iLRF,iGroup).lines(iLine2)=[];
                                lines(iLine2,:)=[];
                                outliers(size(outliers,1)+1:size(outliers,1)+nInliers(iLine2),:)=...
                                    inliers(iLine2,1:nInliers(iLine2),:);
                                cntLines=cntLines-1;
                                continue;
                            else
                                LRF_Lines(iLRF,iGroup).lines(iLine1)=[];
                                lines(iLine1,:)=[];
                                outliers(size(outliers,1)+1:size(outliers,1)+nInliers(iLine1),:)=...
                                    inliers(iLine1,1:nInliers(iLine1),:);
                                if iLine1<cntLines
                                    inliers(iLine1,1:nInliers(iLine1+1),:)=inliers(iLine1+1,1:nInliers(iLine1+1),:);
                                end
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
        
        
        % figure,每个LRF的直线检测效果
        if isShowFigure
            offset=500;
            %             subplot(2,2,iLRF), hold on, axis equal;
            %             if cntLines>0
            %                 title([num2str(iLRF),'号LRF检测到',num2str(cntLines),'条直线']);
            %                 for i=1:cntLines
            %                     plot(inliers(i,1:nInliers(i),1),inliers(i,1:nInliers(i),2),[lineColors(i),'.']);
            %                     x=min(data(:,1)):max(data(:,1));
            %                     y=(lines(i,1).*x+lines(i,3))./(-lines(i,2));
            %                     plot(x,y,[lineColors(i),'-']);
            %                     corePt=LRF_Lines(iLRF,iGroup).lines(i).corePt;
            %                     plot(corePt(1),corePt(2),'ko','LineWidth',3);
            %                 end
            %             end
            %             plot(outliers(:,1),outliers(:,2),'k.');
            %             xlim([min(data(:,1))-offset  max(data(:,1))+offset]);
            %             ylim([min(data(:,2))-offset  max(data(:,2))+offset]);
            %             hold off;
            
            %显示3LRF点云在初始位姿下的效果
            %             subplot4=subplot(2,2,4);
            hold on, axis equal;
            %绘制初始位姿下的点云
            cntAllInliers1=1; cntAllInliers2=0;
            for i=1:cntLines
                cntAllInliers2=cntAllInliers2+nInliers(i);
                allInliers(cntAllInliers1:cntAllInliers2,:)=squeeze(inliers(i,1:nInliers(i),:));
                cntAllInliers1=cntAllInliers2+1;
            end
            allInliers(:,3)=0;
            PC_IniPos=(squeeze(R_LRFs(iLRF,:,:))*allInliers')'+repmat(T_LRFs(iLRF,:),size(allInliers,1),1);
            plot3(PC_IniPos(:,1),PC_IniPos(:,2),PC_IniPos(:,3),[LRFColors(iLRF),'.'],'LineWidth',0.1);
            %绘制检测出直线在初始位姿下的法向量和重心点
            if cntLines>0
                for iLine=1:cntLines
                    corePt=LRF_Lines(iLRF,iGroup).lines(iLine).corePt;
                    vector=8*LRF_Lines(iLRF,iGroup).lines(iLine).vector*LRF_Lines(iLRF,iGroup).lines(iLine).nInliers;
                    
                    corePt(3)=0;
                    corePt=(squeeze(R_LRFs(iLRF,:,:))*corePt')'+T_LRFs(iLRF,:);
                    vector=(squeeze(R_LRFs(iLRF,:,:))*vector')';
                    
                    plot3(corePt(1),corePt(2),corePt(3),'ko','LineWidth',3);
                    
                    basePt=corePt-0.5.*vector;
                    quiver3(basePt(:,1),basePt(:,2),basePt(:,3)...
                        ,vector(:,1),vector(:,2),vector(:,3),'k','LineWidth',3);
                end
            end
            xlabel('X/mm'); ylabel('Y/mm'); zlabel('Z/mm');
            ylim([min(allInliers(:,2))-offset  farthestDist]);
            allInliers=[];
        end
    end
    
    
    %% gallery walls plane detecting
    % detecting planes by colpanar lines
    cntTotaleLinePairs=0;
    cntDeletedPairs1=0;    cntDeletedPairs2=0;
    cntPlanes=0;
    Planes=[];
    for iLRF1=1:NUMBER_LRFs-1
        for iLRF2=iLRF1+1:NUMBER_LRFs
            for iLine1=1:LRF_Lines(iLRF1,iGroup).cntLines
                for iLine2=1:LRF_Lines(iLRF2,iGroup).cntLines
                    cntTotaleLinePairs=cntTotaleLinePairs+1;
                    inliers1=squeeze(LRF_Lines(iLRF1,iGroup).lines(iLine1).inliers);
                    inliers1(:,3)=0;
                    inliers2=squeeze(LRF_Lines(iLRF2,iGroup).lines(iLine2).inliers);
                    inliers2(:,3)=0;
                    inliers1=(squeeze(R_LRFs(iLRF1,:,:))*inliers1')'+repmat(T_LRFs(iLRF1,:),size(inliers1,1),1);
                    inliers2=(squeeze(R_LRFs(iLRF2,:,:))*inliers2')'+repmat(T_LRFs(iLRF2,:),size(inliers2,1),1);
                    
                    splicingInliers=[inliers1;inliers2];
                    X=[ones(size(splicingInliers,1),1),splicingInliers(:,1),splicingInliers(:,2)];
                    Z=splicingInliers(:,3); % z=a0+a1*x+a2*y
                    A=regress(Z,X);
                    plane=[A(2),A(3),-1,A(1)]./norm([A(2),A(3),-1]); % ax+by+cz+d=0;
                    f_plane=plane(1)*x+plane(2)*y+plane(3)*z+plane(4);
                    mask=abs(plane*[splicingInliers,ones(size(splicingInliers,1),1)]')<distThreshold;
                    
                    if sum(mask)<0.9*size(splicingInliers,1)
                        cntDeletedPairs1=cntDeletedPairs1+1;
                        continue;
                    end
                    cntPlanes=cntPlanes+1;
                    Planes(cntPlanes,:)=plane;
                    
                    if isShowFigure
                        minX=min(minX,min(splicingInliers(:,1)));
                        minY=min(minY,min(splicingInliers(:,2)));
                        minZ=min(minZ,min(splicingInliers(:,3)));
                        maxX=max(maxX,max(splicingInliers(:,1)));
                        maxY=max(maxY,max(splicingInliers(:,2)));
                        maxZ=max(maxZ,max(splicingInliers(:,3)));
                        
                        fig11=subplot(1,2,1);hold on,axis equal;
                        plot3(inliers1(:,1),inliers1(:,2),inliers1(:,3),[LRFColors(iLRF1),'o'],'LineWidth',0.1);
                        plot3(inliers2(:,1),inliers2(:,2),inliers2(:,3),[LRFColors(iLRF2),'o'],'LineWidth',0.1);
                        ezimplot3(f_plane,[min(min(minX,minY),minZ)  max(max(maxX,maxY),maxZ)],'r');
                    end
                end
            end
        end
    end
    if cntPlanes<=0
        disp('第',num2str(iGroup),'组未检测出足够符合条件的平面！');
        continue;
    end
    
    %     BlunderDetection_GelleryWalls(Planes);
    %     break;
    
    % removing planes which unmatch to the gallery
    parallelThreshold=5*pi/180; % the least angle between parallel planes
    iPlane1=1;
    while iPlane1<=cntPlanes
        plane1=Planes(iPlane1,:);
        n1=[plane1(1) plane1(2) plane1(3)];
        % to find is there a plane parallel to it or perpendicular to it
        matchedPlanes=0;
        iPlane2=1;
        while iPlane2<=cntPlanes
            if iPlane1==iPlane2
                iPlane2=iPlane2+1;
                continue;
            end
            plane2=Planes(iPlane2,:);
            n2=[plane2(1) plane2(2) plane2(3)];
            if norm(cross(n1,n2))<sin(parallelThreshold) || norm(cross(n1,n2))>sin(pi/2-parallelThreshold)
                matchedPlanes=matchedPlanes+1;
            end
            iPlane2=iPlane2+1;
        end
        % if there is no plane ....
        if matchedPlanes<0.6*cntPlanes
            cntDeletedPairs2=cntDeletedPairs2+1;
            Planes(iPlane1,:)=[];
            cntPlanes=cntPlanes-1;
            continue;
        end
        iPlane1=iPlane1+1;
    end
    if cntPlanes<=0
        disp(['第',num2str(iGroup),'组未检测出足够符合条件的平面！']);
        continue;
    end
    
    % show the planes (after removed unmatched planes)
    if isShowFigure
        for iPlane=1:cntPlanes
            plane=Planes(iPlane,:);
            f_plane=plane(1)*x+plane(2)*y+plane(3)*z+plane(4);
            ezimplot3(f_plane,[min(min(minX,minY),minZ)  max(max(maxX,maxY),maxZ)],'r');
        end
    end
    % disp the planes number and the removed planes number
    disp(['Total ' num2str(cntTotaleLinePairs) ' line pair(s)']);
    disp(['deleted ' num2str(cntDeletedPairs1) ' not coplanar line pair(s)']);
    disp(['deleted ' num2str(cntDeletedPairs2) ' not on-wall line pair(s)']);
    
    Planes
    
    %% get the direction of the world frame (gallery frame)
    % dividing the norm vectors of planes to two goups of approximately parallel groups
    cntV1=0; cntV2=0;
    clear V1 V2 sumV;
    meanV1=[0,0,0]; meanV2=[0,0,0];
    for iPlane=1:cntPlanes
        v=Planes(iPlane,1:3);
        if abs(cross(v,meanV1))<sin(parallelThreshold)
            cntV1=cntV1+1;
            V1(cntV1,:)=v;
            meanV1(1)=mean(V1(:,1));meanV1(2)=mean(V1(:,2));meanV1(3)=mean(V1(:,3));
        elseif abs(cross(v,meanV2))<sin(parallelThreshold)
            cntV2=cntV2+1;
            V2(cntV2,:)=v;
            meanV2(1)=mean(V2(:,1));meanV2(2)=mean(V2(:,2));meanV2(3)=mean(V2(:,3));
        else
            disp('There are more than 2 parallel plane groups!');
            continue;
        end
    end
    if cntV1*cntV2<=1
        disp('There is less than 2 parallel plane groups!');
        continue;
    end
    
    % get the total pairs between V1 and V2, and record the cross product
    cntSumV=0;
    for iV1=1:cntV1
        for iV2=1:cntV2
            v1=V1(iV1,:);
            v2=V2(iV2,:);
            cntSumV=cntSumV+1;
            sumV(cntSumV,:)=cross(v1,v2);
        end
    end
    % Unify the direction of vectors
    for i=2:cntV1*cntV2
        if dot(sumV(i,:),sumV(i-1,:))<0
            sumV(i,:)=-sumV(i,:);
        end
    end
    % take the mean SumV as the Y axis of the world frame
    axisY_W=mean(sumV);
    axisY_W=axisY_W./norm(axisY_W);
    axisY_W_=axisY_W*600;
    % convert V1s and V2s to a 2D plane which is perpendicular to axisZ_W
    clear V1_ V2_ V1__ V2__;
    for iV1=1:cntV1
        V1_(iV1,:)=V1(iV1,:)-dot(axisY_W,V1(iV1,:))*axisY_W;
    end
    for iV2=1:cntV2
        V2_(iV2,:)=V2(iV2,:)-dot(axisY_W,V2(iV2,:))*axisY_W;
    end
    % to get X axis, convert V2_ to a perpendicular vector
    for iV2=1:cntV2
        V2__(iV2,:)=cross(V2_(iV2,:),axisY_W);
    end
    % combine V1_ and V2__ as V1__
    V1__(1:cntV1,:)=V1_(1:cntV1,:);
    V1__(cntV1+1:cntV1+cntV2,:)=V2__(1:cntV2,:);
    % vector unitization
    for iV1__=1:cntV1+cntV2
        V1__(iV1__,:)=V1__(iV1__,:)./norm(V1__(iV1__,:));
    end
    % unify the direction of vectors
    for iV1__=2:cntV1+cntV2
        if dot(V1__(iV1__-1,:),V1__(iV1__,:))<0
            V1__(iV1__,:)=-V1__(iV1__,:);
        end
    end
    % take the mean V1__ as the X axis of the world frame
    axisX_W=mean(V1__);
    axisX_W=axisX_W-dot(axisX_W,axisY_W)*axisY_W; % make sure that X is perpendicular to Y
    axisX_W=axisX_W./norm(axisX_W);
    axisX_W_=600*axisX_W;
    % take the cross product of X axis and Y axis as the Z axis
    axisZ_W=cross(axisX_W,axisY_W);
    axisZ_W=axisZ_W./norm(axisZ_W);
    % finally, X, Y, and Z axes form the world frame
    R_D2W=[axisX_W;axisY_W;axisZ_W];
    if isShowFigure
        PlotAxes_TR('k',':',[0,0,0],R_D2W,600,2);
    end
    
    
    %% based on the direction of world frame, get the position of the world frame (gallery frame)
    T_D2W=zeros(1,3);
    % compute the signedDistance(a vector) from zeroPt to the wall planes
    signedDistances=zeros(cntPlanes,3);
    for iPlane=1:cntPlanes
        signedDistances(iPlane,:)=-1*dot([0,0,0,1],Planes(iPlane,:))*Planes(iPlane,1:3);
    end
    % show the signedDistances as vectors
    if isShowFigure
        %         ZerosPoints=zeros(cntPlanes,3);
        %         quiver3(ZerosPoints(:,1),ZerosPoints(:,2),ZerosPoints(:,3),...
        %             signedDistances(:,1),signedDistances(:,2),signedDistances(:,3),'p','LineWidth',3);
    end
    % transform the signed distances from device frame to world frame
    signedDistances=(R_D2W*signedDistances')';
    % get the components of signedDistances in the X axis and Z axis
    clear dX dX_ dZ dZ_;
    cntWalls_X=0; cntWalls_X_=0; % the count num of walls perpendicular to X (note the sign, represents two opposite directions)
    cntWalls_Z=0; cntWalls_Z_=0;
    for iPlane=1:cntPlanes
        v=signedDistances(iPlane,:);
        d=0.9*norm(v);
        if v(1)>d
            cntWalls_X=cntWalls_X+1;
            dX(cntWalls_X)=v(1);
        elseif v(1)<-d
            cntWalls_X_=cntWalls_X_+1;
            dX_(cntWalls_X_)=v(1);
        elseif v(3)>d
            cntWalls_Z=cntWalls_Z+1;
            dZ(cntWalls_Z)=v(3);
        elseif v(3)<-d
            cntWalls_Z_=cntWalls_Z_+1;
            dZ_(cntWalls_Z_)=v(3);
        else
            disp('There is an unexpected error here!');
            continue;
        end
    end
    % check which axis is horizontal in the real world (according to the known size of gallery)
    if (cntWalls_X+cntWalls_X_)*(cntWalls_Z+cntWalls_Z_)<=0
        disp('There is not enough reliable wall planes!');
        continue;
    end
    bIsXHerizontal=1;
    if cntWalls_X>0 && cntWalls_X_>0
        lX=abs(mean(dX))+abs(mean(dX_));
        T_D2W(1)=-(mean(dX)+mean(dX_))/2;
        if abs(lX-galleryWidth)>abs(lX-galleryHeight)
            bIsXHerizontal=0;
            if cntWalls_Z>0 && cntWalls_Z_>0
                lZ=abs(mean(dZ))+abs(mean(dZ_));
                T_D2W(3)=-(mean(dZ)+mean(dZ_))/2;
            elseif cntWalls_Z>0
                T_D2W(3)=galleryWidth/2-mean(dZ);
            else
                T_D2W(3)=-galleryWidth/2-mean(dZ_);
            end
        else
            if cntWalls_Z>0 && cntWalls_Z_>0
                lZ=abs(mean(dZ))+abs(mean(dZ_));
                T_D2W(3)=-(mean(dZ)+mean(dZ_))/2;
            elseif cntWalls_Z>0
                T_D2W(3)=galleryHeight/2-mean(dZ);
            else
                T_D2W(3)=-galleryHeight/2-mean(dZ_);
            end
        end
    elseif cntWalls_Z>0 && cntWalls_Z_>0
        lZ=abs(mean(dZ))+abs(mean(dZ_));
        T_D2W(3)=-(mean(dZ)+mean(dZ_))/2;
        if abs(lZ-galleryWidth)<abs(lZ-galleryHeight)
            bIsXHerizontal=0;
            if cntWalls_X>0
                T_D2W(1)=galleryHeight/2-mean(dX);
            else
                T_D2W(1)=-galleryHeight/2-mean(dX_);
            end
        else
            if cntWalls_X>0
                T_D2W(1)=galleryWidth/2-mean(dX);
            else
                T_D2W(1)=-galleryWidth/2-mean(dX_);
            end
        end
    else
        disp('There is not enough reliable wall planes!');
        continue;
    end
    if isShowFigure
        PlotAxes_RT('k','-',T_D2W,R_D2W,1600,2);
    end
    
    % record the R & T
    R_D2Ws(iGroup,:,:)=R_D2W;
    T_D2Ws(iGroup,:)=T_D2W;
    
end

if ~isShowFigure
    % save the RT to file
    fileName_AllFramesPose=['AllFramesPose_' fileName_RawPC0];
    fileFullPath_AllFramesPose=[RawFilePath fileName_AllFramesPose '.mat'];
    save(fileFullPath_AllFramesPose,'R_D2Ws','T_D2Ws');
end





















