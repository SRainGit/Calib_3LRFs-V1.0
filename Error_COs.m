
function Error=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_LRFs,T_LRFs,bFilter)

global COs

nCOs=size(COs,2);
error_COs=zeros(nCOs,1);

%% extract Rs and Ts
R0 = squeeze(R_LRFs(FixedLRF,:,:));
T0 = squeeze(T_LRFs(FixedLRF,:));
R1 = squeeze(R_LRFs(CalibLRF,:,:));
T1 = squeeze(T_LRFs(CalibLRF,:));

%% for coumputing wall normal
n_nk_4_2=nchoosek(4,2);
mat_nk_4_2=nchoosek(1:4,2);
n_nk_6_2=nchoosek(6,2);
mat_nk_6_2=nchoosek(1:6,2);


bShowFigure=0;
% bShowFigure=1;
if bShowFigure
    fig_norms=figure;
    h = [-2000 110 1500 980];
%     h = [100 10 1500 980];
    set(gcf,'Position',h)
    rotate3d on; hold on; axis equal;
    WallColors=['r','g','b','m','c'];
    LRFColors=['r','g','b'];
end

for iCO=1:nCOs
    wallNorms=zeros(4,3);
    %% errors_planarity
    OneCO=COs(iCO);
    errors_planarity=zeros(4,1);
    clear Walls;
    cntWall=4;
    for iWall=1:4
        cntTetrahedrons=0;
        Walls(iWall).cntTetrahedrons=0;
        Walls(iWall).TotalVolume_abs=0;
        cntLines=OneCO.CO(iWall).cntLines;
        lines=OneCO.CO(iWall).lines;
        if cntLines>1  % skip the wall without enough lines
            % splice all inlierss
            clear allInliers
            % transverse all the lines
            for iLine0=1:cntLines-1
                for iLine1=iLine0+1:cntLines
                    iLRF0=lines(iLine0).idLRF;
                    iLRF1=lines(iLine1).idLRF;
                    if (iLRF0==FixedLRF && iLRF1==CalibLRF) || (iLRF1==FixedLRF && iLRF0==CalibLRF)
                        % ensure the line1 is one line on the fixed LRF
                        if iLRF0==FixedLRF && iLRF1==CalibLRF
                            line0=lines(iLine0);
                            line1=lines(iLine1);
                        else
                            line0=lines(iLine1);
                            line1=lines(iLine0);
                        end
                        % extract and record endPts
                        endPts0=line0.endPts;
                        endPts0(:,3)=0;
                        v0=line0.vector; v0(3)=0;
                        endPts0=(R0*endPts0')'+[T0;T0];
                        v0=(R0*v0')';
                        endPts1=line1.endPts;
                        endPts1(:,3)=0;
                        v1=line1.vector; v1(3)=0;
                        endPts1=(R1*endPts1')'+[T1;T1];
                        v1=(R1*v1')';
                        % input vertices of Tetrahedron
                        cntTetrahedrons=cntTetrahedrons+1;
                        endPts=[endPts0;endPts1];
                        volume_signed=VolumeOfTetrahedron_signed(endPts);
                        Walls(iWall).TotalVolume_abs=Walls(iWall).TotalVolume_abs+abs(volume_signed);
                    end
                end
            end
            Walls(iWall).cntTetrahedrons=cntTetrahedrons;
            if cntTetrahedrons>0
                % errors_planarity
                errors_planarity(iWall)= Walls(iWall).TotalVolume_abs;%/cntTetrahedrons;                
                % wall norm
                if cntTetrahedrons>1
                    disp('warning, please check.');
                    return;
                end
                vectorBasePts=endPts(mat_nk_4_2(:,1),:);                
                vectorEndPts=endPts(mat_nk_4_2(:,2),:);
                vectors0=vectorEndPts(mat_nk_6_2(:,1),:)-vectorBasePts(mat_nk_6_2(:,1),:);
                vectors1=vectorEndPts(mat_nk_6_2(:,2),:)-vectorBasePts(mat_nk_6_2(:,2),:);
                sumV=zeros(1,3);
                for i=1:n_nk_6_2
                    crossV=cross(squeeze(vectors0(i,:)),squeeze(vectors1(i,:)));
                    if dot(crossV,endPts(1,:))>0
                        crossV=-crossV;  % make sure the cross vector is arrow to zero pt
                    end
                    sumV=sumV+crossV;
                end
                sumV=sumV./n_nk_6_2;
                wallNorms(iWall,:)=sumV./norm(sumV);
            else
                errors_planarity(iWall)=0;
                cntWall=cntWall-1;
            end
        else
            errors_planarity(iWall)=0;
            cntWall=cntWall-1;
        end
    end
     
    
    %% Error_Orthogonality
    cntOrthogonalityPairs=4;
    errors_orthogonality=zeros(1,4);
    % Orthogonality errors
    for iWall0=1:4
        iWall1=mod(iWall0+1-1,4)+1;
        if norm(wallNorms(iWall0,:))<=0 || norm(wallNorms(iWall1,:))<=0
            cntOrthogonalityPairs=cntOrthogonalityPairs-1;
            continue;
        end
        errors_orthogonality(iWall0)=abs(dot(wallNorms(iWall0,:),wallNorms(iWall1,:)));
    end
        
    %% Error_Corner
    cntWallCorners=0;
    errors_Corners=zeros(1,4);
    cornerLines=zeros(4,3);
    % Orthogonality errors
    for iWall=1:4
        cntCorners=OneCO.CO(iWall).cntCorners;
        corners=OneCO.CO(iWall).corners;
        if cntCorners<2
            continue;
        end
        for iCorner0=1:cntCorners-1
            iLRF0=corners(iCorner0).idLRF;
            if iLRF0~=FixedLRF && iLRF0~=CalibLRF
                continue;
            end            
            for iCorner1=iCorner0+1:cntCorners
                iLRF1=corners(iCorner1).idLRF;
                if iLRF1~=FixedLRF && iLRF1~=CalibLRF
                    continue;
                end
                corner0=corners(iCorner0).Pt;
                corner1=corners(iCorner1).Pt;
                corner0=R0*corner0+T0';
                corner1=R1*corner1+T1';
                % compute error
                cntWallCorners=cntWallCorners+1;
                cornerLine=corner0-corner1;
                cornerLine=cornerLine./norm(cornerLine);
                cornerLines(iWall,:)=cornerLine';
                for iWall1=1:4
                    if norm(wallNorms(iWall1,:))>0
                        errors_Corners(iWall)=errors_Corners(iWall)+...
                            abs(dot(cornerLine',squeeze(wallNorms(iWall1,:))));
                    end
                end
            end
        end
    end
    
%     %% Error_ParallelCorner
%     cntParallelCornerPairs=0;
%     if cntWallCorners>=2
%         errors_ParallelCorners=zeros(1,nchoosek(cntWallCorners,2));
%         % Orthogonality errors
%         for iWall0=1:4
%             if norm(cornerLines(iWall0,:))>0
%                 for iWallOffset=1:3
%                     iWall1=mod(iWall0+iWallOffset-1,4)+1;
%                     if norm(cornerLines(iWall1,:))>0
%                         cntParallelCornerPairs=cntParallelCornerPairs+1;
%                         errors_ParallelCorners(cntParallelCornerPairs)=...
%                             norm(cross(cornerLines(iWall0,:),cornerLines(iWall1,:)));
%                     end
%                 end
%             end
%         end
%     end
        
    %% Parallel errors
    cntParallelPairs=2;
    errors_parallel=zeros(1,2);
    for iWall=1:2
        iOppositeWall=iWall+2;
        if norm(wallNorms(iWall,:))<=0 || norm(wallNorms(iOppositeWall,:))<=0
            cntParallelPairs=cntParallelPairs-1;
            continue;
        end
        errors_parallel(iWall)=norm(cross(wallNorms(iWall,:),wallNorms(iOppositeWall,:)));
    end    
    
    %% Error_thisCO
    if cntWall>0
        error_COs(iCO) = error_COs(iCO)+sum(errors_planarity);%/cntWall;
    end
    if cntOrthogonalityPairs>0
        error_COs(iCO) = error_COs(iCO) + sum(errors_orthogonality);%/cntOrthogonalityPairs;
    end
    if cntWallCorners>0
        error_COs(iCO) = error_COs(iCO) + sum(errors_Corners);%/cntOrthogonalityPairs;
    end
%     if cntParallelCornerPairs>0
%         error_COs(iCO)=error_COs(iCO)+sum(errors_ParallelCorners);
%     end
    if cntParallelPairs>0
        error_COs(iCO)=error_COs(iCO)+sum(errors_parallel);%/cntParallelPairs;
    end
    
    if bShowFigure
        ZerosPoints=zeros(4,3);
        plot3(0,0,0,'ro','LineWidth',3);
        quiver3(ZerosPoints(:,1),ZerosPoints(:,2),ZerosPoints(:,3),...
            wallNorms(:,1),wallNorms(:,2),wallNorms(:,3),'g','LineStyle','-','LineWidth',3);
        cla(fig_norms);
    end
    % end of oneCO
end
        
% bad CO filter
if bFilter==1
    [maxError,indexMaxError]=max(error_COs);
    meanError=mean(error_COs);
    stdError=std(error_COs);
    mask=(error_COs>2*meanError+2*stdError);
    if sum(mask)>0
        COs(indexMaxError)=[];
        error_COs(indexMaxError)=[];
        nCOs=nCOs-1;
    end
end

Error=sum(error_COs)/nCOs;

end











