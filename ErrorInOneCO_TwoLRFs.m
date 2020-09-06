
function Error=ErrorInOneCO_TwoLRFs(OneCO,FixedLRF,CalibLRF,R_LRFs,T_LRFs)
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
            for iLine2=iLine1+1:cntLines
                % extract data
                iLRF1=lines(iLine1).idLRF;
                iLRF2=lines(iLine2).idLRF;
                if (iLRF1~=FixedLRF && iLRF1~=CalibLRF) || (iLRF2~=FixedLRF && iLRF2~=CalibLRF)
                    continue;
                end
                endPts1=lines(iLine1).endPts;
                endPts1(:,3)=0;
                % extract rotation matrix and translation matrix
                R1=squeeze(R_LRFs(iLRF1,:,:));
                T1=squeeze(T_LRFs(iLRF1,:));
                % record the endPts
                endPts1=(R1*endPts1')'+[T1;T1];
                % transverse the remaing lines
                % extract data
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
    else
        errors_planarity(iWall)=0;
    end
    
    
end


Error=sum(errors_planarity);

% Error=Error*10^-9;
if bShowFigure
    title(num2str(Error));
end
end











