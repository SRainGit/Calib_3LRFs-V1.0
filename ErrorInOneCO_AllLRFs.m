
function Error=ErrorInOneCO_AllLRFs(OneCO,R_LRFs,T_LRFs)
bShowFigure=0;
% bShowFigure=1;
if bShowFigure
    close all;
    figure, h = [-1500 110 1500 980]; set(gcf,'Position',h)
    %         figure, h = [0 0 1500 980]; set(gcf,'Position',h)
    rotate3d on;    axis equal;
    Colors=['r','g','b','m','c'];
    minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
    syms x y z real
end

%% Planarity errors
errors_planarity=zeros(1,4); % the total volume of tatraherons (we have 4 walls in one gallery observation)
for iWall=1:4
    cntTetrahedrons=0;
    Walls(iWall).cntTetrahedrons=0;
    Walls(iWall).TotalVolume=0;
    % extract data
    cntLines=OneCO.CO(iWall).cntLines;
    lines=OneCO.CO(iWall).lines;
    % skip the wall without enough lines
    if cntLines>1
        % splice all inlierss
        clear allInliers
        for iLine1=1:cntLines-1
            % extract data
            iLRF1=lines(iLine1).idLRF;
            endPts1=lines(iLine1).endPts;
            endPts1(:,3)=0;
            % extract rotation matrix and translation matrix
            R1=squeeze(R_LRFs(iLRF1,:,:));
            T1=squeeze(T_LRFs(iLRF1,:));
            % record the endPts
            endPts1=(R1*endPts1')'+[T1;T1];
            % transverse the remaing lines
            for iLine2=iLine1+1:cntLines
                % extract data
                iLRF2=lines(iLine2).idLRF;
                endPts2=lines(iLine2).endPts;
                endPts2(:,3)=0;
                % extract rotation matrix and translation matrix
                R2=squeeze(R_LRFs(iLRF2,:,:));
                T2=squeeze(T_LRFs(iLRF2,:));
                % record the endPts
                endPts2=(R2*endPts2')'+[T2;T2];
                % input vertices of Tetrahedron
                cntTetrahedrons=cntTetrahedrons+1;
                Walls(iWall).Tetrahedrons(cntTetrahedrons).Points(1:2,:)=endPts1;
                Walls(iWall).Tetrahedrons(cntTetrahedrons).Points(3:4,:)=endPts2;
                volume=VolumeOfTetrahedron([endPts1;endPts2]);
                Walls(iWall).Tetrahedrons(cntTetrahedrons).Volume=volume;
                Walls(iWall).TotalVolume=Walls(iWall).TotalVolume+volume;
            end
        end
        Walls(iWall).cntTetrahedrons=cntTetrahedrons;
        errors_planarity(iWall)= Walls(iWall).TotalVolume;%/cntTetrahedrons;
    elseif cntLines==1        
        iOppositeWall=mod(iWall+2-1,4)+1;
        iLine1=1;
        iLRF1=lines(iLine1).idLRF;
        endPts1=lines(iLine1).endPts;
        endPts1(:,3)=0;
        % extract rotation matrix and translation matrix
        R1=squeeze(R_LRFs(iLRF1,:,:));
        T1=squeeze(T_LRFs(iLRF1,:));
        % record the endPts
        endPts1=(R1*endPts1')'+[T1;T1];
        
        cntLines2=OneCO.CO(iOppositeWall).cntLines;
        lines2=OneCO.CO(iOppositeWall).lines;
        volumes=zeros(1,cntLines2);
        for iLine2=1:cntLines2
            % extract data
            iLRF2=lines2(iLine2).idLRF;
            endPts2=lines2(iLine2).endPts;
            endPts2(:,3)=0;
            % extract rotation matrix and translation matrix
            R2=squeeze(R_LRFs(iLRF2,:,:));
            T2=squeeze(T_LRFs(iLRF2,:));
            % record the endPts
            endPts2=(R2*endPts2')'+[T2;T2];
            volumes(iLine2)=VolumeOfTetrahedron([endPts1;endPts2]);
        end
        if max(volumes)<0.5
            errors_planarity(iWall)=max(volumes)+0.1;
        end
    else
        errors_planarity(iWall)=0;
    end
    
    if bShowFigure
        clear allInliers;
        if cntLines>0
            cntAllInliers1=1; cntAllInliers2=0;
            for iLine=1:cntLines
                idLRF=lines(iLine).idLRF;
                R_=squeeze(R_LRFs(idLRF,:,:));
                T_=squeeze(T_LRFs(idLRF,:));
                nOneInliers=lines(iLine).nInliers;
                oneInliers=lines(iLine).inliers;
                oneInliers(:,3)=0;
                oneInliers=(R_*oneInliers')'+repmat(T_,nOneInliers,1);
                cntAllInliers2=cntAllInliers2+nOneInliers;
                allInliers(cntAllInliers1:cntAllInliers2,:)=squeeze(oneInliers(1:nOneInliers,:));
                cntAllInliers1=cntAllInliers2+1;
            end
            minX=min(minX,min(allInliers(:,1)));
            minY=min(minY,min(allInliers(:,2)));
            minZ=min(minZ,min(allInliers(:,3)));
            maxX=max(maxX,max(allInliers(:,1)));
            maxY=max(maxY,max(allInliers(:,2)));
            hold on;
            % The gallery observation in this data frame based on ini poses
            plot3(allInliers(:,1),allInliers(:,2),allInliers(:,3),[Colors(iWall),'.']);
            for iTetrahedron=1:Walls(iWall).cntTetrahedrons
                tetrahedron=Walls(iWall).Tetrahedrons(iTetrahedron).Points(:,:);
                Tes=delaunayn(tetrahedron);%return a m¡Án matrix
                tetramesh(Tes,tetrahedron,'r');%plot a figure of tetrahedron
                alpha(0.2);
            end
            title(num2str(errors_planarity(iWall)));
        end
    end
end

%% wall sequence errors
wallPlanes=GetWallPlanes(OneCO,R_LRFs,T_LRFs);
wallNorms=wallPlanes(:,1:3);
crossVectors=zeros(4,3);
cntWallPairs=4;
error_wallSequence=0;
% Orthogonality errors
for iWall0=1:4
    iWall1=mod(iWall0+1-1,4)+1;
    if norm(wallNorms(iWall0,:))<=0 || norm(wallNorms(iWall1,:))<=0
        cntWallPairs=cntWallPairs-1;
        continue;
    end    
    crossVectors(iWall0,:)=cross(wallNorms(iWall0,:),wallNorms(iWall1,:));
end
for iV0=1:3
    for iV1=iV0+1:4
        error=dot(crossVectors(iV0,:),crossVectors(iV1,:));
        if error<0
            error_wallSequence=error_wallSequence+abs(error);
        end
    end
end

% % exclusive error
% error_exclusive=0;
% for iWall0=1:4
%     iWall1=mod(iWall0+1-1,4)+1;
%     if norm(wallNorms(iWall0,:))>0
%         cntLines=OneGO.GO(iWall1).cntLines;
%         lines=OneGO.GO(iWall1).lines;
%         for iLine=1:cntLines
%             iLRF=lines(iLine).idLRF;
%             vLine=lines(iLine).vector;
%             R=squeeze(R_LRFs(iLRF,:,:));
%             vLine=(R*vLine');
%             error=norm(cross(vLine,wallNorms(iWall0,:)));
%             if error>0.5
%                 error_exclusive=error_exclusive+error;
%             end
%         end
%     end    
% end
% % opposite error
% error_opposite=0;
% for iWall0=1:4
%     iWall1=mod(iWall0+2-1,4)+1;
%     if norm(wallNorms(iWall0,:))>0
%         cntLines=OneGO.GO(iWall1).cntLines;
%         lines=OneGO.GO(iWall1).lines;
%         for iLine=1:cntLines
%             iLRF=lines(iLine).idLRF;
%             centerPt=lines(iLine).centerPt;
%             R=squeeze(R_LRFs(iLRF,:,:));
%             T=squeeze(T_LRFs(iLRF,:));
%             centerPt=(R*centerPt')'+T;
%             
%             error=abs([centerPt,1]*wallPlanes(iWall0,:)');
%             if error<500
%                 error_opposite=error_opposite+(2-error);
%             end
%         end
%     end
% end
% 
% 
% Error=sum(errors_planarity)+error_wallSequence;
% Error=Error+error_exclusive;
% Error=Error+error_opposite; 



Error=sum(errors_planarity)+error_wallSequence;
% Error=sum(errors_planarity);

if bShowFigure
    title(num2str(Error));
end
end











