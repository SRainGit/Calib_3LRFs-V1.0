
function Error=ErrorIn2COs_AllLRFs(CO0,CO1,R_LRFs,T_LRFs)
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
R_Group0=CO0.Pose.R;  T_Group0=CO0.Pose.T;
R_Group1=CO1.Pose.R;  T_Group1=CO1.Pose.T;
errors_planarity=zeros(1,4);
wallPlanes=zeros(4,4);
for iWall=1:4
    cntTetrahedrons=0;
    Walls(iWall).cntTetrahedrons=0;
    Walls(iWall).TotalVolume=0;
    % extract data
    cntLines0=CO0.CO(iWall).cntLines;
    lines0=CO0.CO(iWall).lines;
    cntLines1=CO1.CO(iWall).cntLines;
    lines1=CO1.CO(iWall).lines;
    % skip the wall without enough lines
    if cntLines0>0 && cntLines1>0
        % splice all inlierss
        clear allEndPts
        clear allEndPts_
        allEndPts=[]; allEndPts_=[];
        % pre process lines1 (for reducing the computing)
        for iLine1=1:cntLines1
            % extract data
            iLRF1=lines1(iLine1).idLRF;
            endPts1=lines1(iLine1).endPts;
            endPts1(:,3)=0;
            % extract rotation matrix and translation matrix
            R1=squeeze(R_LRFs(iLRF1,:,:));
            T1=squeeze(T_LRFs(iLRF1,:));
            [R1,T1]=PoseAddition_RT(R1,T1',R_Group1,T_Group1);
            % record the endPts
            endPts1=(R1*endPts1')'+[T1';T1'];
            allEndPts_=[allEndPts_;endPts1];
        end
        for iLine0=1:cntLines0
            % extract data
            iLRF0=lines0(iLine0).idLRF;
            endPts0=lines0(iLine0).endPts;
            endPts0(:,3)=0;
            % extract rotation matrix and translation matrix
            R0=squeeze(R_LRFs(iLRF0,:,:));
            T0=squeeze(T_LRFs(iLRF0,:));
            [R0,T0]=PoseAddition_RT(R0,T0',R_Group0,T_Group0);
            % record the endPts
            endPts0=(R0*endPts0')'+[T0';T0'];
            allEndPts=[allEndPts;endPts0];
            % transverse the remaing lines
            for iLine1=1:cntLines1
                % record the endPts
                endPts1=allEndPts_(iLine1*2-1:iLine1*2,:);
                % input vertices of Tetrahedron
                cntTetrahedrons=cntTetrahedrons+1;
                Walls(iWall).Tetrahedrons(cntTetrahedrons).Points(1:2,:)=endPts0;
                Walls(iWall).Tetrahedrons(cntTetrahedrons).Points(3:4,:)=endPts1;
                volume=VolumeOfTetrahedron([endPts0;endPts1]);
                Walls(iWall).Tetrahedrons(cntTetrahedrons).Volume=volume;
                Walls(iWall).TotalVolume=Walls(iWall).TotalVolume+volume;
            end
        end
        
        % errors_planarity
        Walls(iWall).cntTetrahedrons=cntTetrahedrons;
        errors_planarity(iWall)= Walls(iWall).TotalVolume/cntTetrahedrons;
        
        % wall planes
        allEndPts=[allEndPts;allEndPts_];        
        X=[ones(size(allEndPts,1),1),allEndPts(:,1),allEndPts(:,2)];
        Z=allEndPts(:,3); % z=a0+a1*x+a2*y
        A=regress(Z,X);
        plane=[A(2),A(3),-1,A(1)]./norm([A(2),A(3),-1]); % ax+by+cz+d=0; norm(a,b,c)=0
        % ensure the norm is point to zero point
        if plane(4)<0
            plane=-plane;
        end
        wallPlanes(iWall,:)=plane;
    end
end

%% Error_Orthogonality
cntOrthogonalityPairs=4;
errors_orthogonality=zeros(1,4);
wallNorms=wallPlanes(:,1:3);
% Orthogonality errors
for iWall0=1:4
    iWall1=mod(iWall0+1-1,4)+1;
    if norm(wallNorms(iWall0))<=0 || norm(wallNorms(iWall1))<=0
        cntOrthogonalityPairs=cntOrthogonalityPairs-1;
        continue;
    end
    errors_orthogonality(iWall0)=abs(dot(wallNorms(iWall0,:),wallNorms(iWall1,:)));
end

Error=sum(errors_planarity);
if cntOrthogonalityPairs>0
    Error = Error + sum(errors_orthogonality);%/cntOrthogonalityPairs;
end

if bShowFigure
    hold on;
    for iWall=1:4
        for iTetrahedron=1:Walls(iWall).cntTetrahedrons
            tetrahedron=Walls(iWall).Tetrahedrons(iTetrahedron).Points(:,:);
            Tes=delaunayn(tetrahedron);%返回m×n的数组值
            tetramesh(Tes,tetrahedron,'r');%绘制四面体图
            alpha(0.2);
        end
    end
    title(num2str(Error));
end
end











