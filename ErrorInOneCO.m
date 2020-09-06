
function Error=ErrorInOneCO(OneCO,NUMBER_LRFs,R,T)
half_PI=pi/2;
radian2degree=180/pi;

bShowFigure=0;
% bShowFigure=1;
if bShowFigure
    close all;
%     figure, h = [-1500 110 1500 980]; set(gcf,'Position',h)
            figure, h = [110 110 600 400]; set(gcf,'Position',h)
    rotate3d on;    axis equal;
    Colors=['r','g','b','m','c'];
    minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
    syms x y z real
end

%%
errors=zeros(1,4); % we have 4 walls in one gallery
cntTetrahedrons=0;
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
            R1=squeeze(R(iLRF1,:,:));
            T1=squeeze(T(iLRF1,:));
            % record the endPts
            endPts1=(R1*endPts1')'+[T1;T1];
            % transverse the remaing lines
            for iLine2=iLine1+1:cntLines
                % extract data
                iLRF2=lines(iLine2).idLRF;
                endPts2=lines(iLine2).endPts;
                endPts2(:,3)=0;
                % extract rotation matrix and translation matrix
                R2=squeeze(R(iLRF2,:,:));
                T2=squeeze(T(iLRF2,:));
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
        errors(iWall)= Walls(iWall).TotalVolume*(10^-3)^3;
    else
        errors(iWall)=0;
    end
    
    if bShowFigure
        clear allInliers;
        if cntLines>0
            cntAllInliers1=1; cntAllInliers2=0;
            for iLine=1:cntLines
                idLRF=lines(iLine).idLRF;
                R_=squeeze(R(idLRF,:,:));
                T_=squeeze(T(idLRF,:));
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
                Tes=delaunayn(tetrahedron);%返回m×n的数组值
                tetramesh(Tes,tetrahedron,'r');%绘制四面体图
                alpha(0.2);
            end
            title(['Volume=' num2str(errors(iWall))]);
        end
    end
end

Error=sum(errors);
if bShowFigure
    axis off;
    title(['Sum of Volume = ', num2str(Error), '(m^3)']);
end
end











