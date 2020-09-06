
%% get the four wall norms (vectors from outside point to inside) from on CO
% plane: ax+by+cz+d=0;
function wallPlanes = GetWallPlanes(CO,R_LRFs,T_LRFs)
wallPlanes=zeros(4,4);
oneCO=CO.CO;

bShowFigure=0;
% bShowFigure=1;
if bShowFigure
    figure;
    h = [-1500 110 1500 980];
%     h = [100 10 1500 980];
    set(gcf,'Position',h)
    rotate3d on; hold on; axis equal;
    WallColors=['r','g','b','m','c'];
    LRFColors=['r','g','b'];
    syms x y z real;    
    minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
    ZerosPoints=zeros(1,3);
end

for iWall=1:4
    if oneCO(iWall).cntLines<2
        continue;
    end
    wallEndPts=zeros(2*oneCO(iWall).cntLines,3);
    for iLine=1:oneCO(iWall).cntLines
        oneLine=oneCO(iWall).lines(iLine);
%         idGroup=oneLine.idGroup;
        idLRF=oneLine.idLRF;
        endPts=oneLine.endPts;
        endPts(:,3)=0;
        R=squeeze(R_LRFs(idLRF,:,:));
        T=squeeze(T_LRFs(idLRF,:))';
        endPts=(R*endPts')'+[T';T'];
        wallEndPts(2*iLine-1:2*iLine,:)=endPts;
        if bShowFigure
            line(endPts(:,1),endPts(:,2),endPts(:,3),'color',WallColors(iWall),'LineStyle','-')
        end
    end
    
    X=[ones(size(wallEndPts,1),1),wallEndPts(:,1),wallEndPts(:,2)];
    Z=wallEndPts(:,3); % z=a0+a1*x+a2*y
    A=regress(Z,X);
    plane=[A(2),A(3),-1,A(1)]./norm([A(2),A(3),-1]); % ax+by+cz+d=0;
    % ensure the norm is point to zero point
    if plane(4)<0
        plane=-plane;
    end
    wallPlanes(iWall,:)=plane;
    
    if bShowFigure
        plot3(wallEndPts(:,1),wallEndPts(:,2),wallEndPts(:,3),'ro');
        minX=min(minX,min(wallEndPts(:,1)));
        minY=min(minY,min(wallEndPts(:,2)));
        minZ=min(minZ,min(wallEndPts(:,3)));
        maxX=max(maxX,max(wallEndPts(:,1)));
        maxY=max(maxY,max(wallEndPts(:,2)));
        maxZ=max(maxZ,max(wallEndPts(:,3)));
        f_plane=plane(1)*x+plane(2)*y+plane(3)*z+plane(4);
        ezimplot3(f_plane,[min(min(minX,minY),minZ)  max(max(maxX,maxY),maxZ)],'r');
        quiver3(ZerosPoints(:,1),ZerosPoints(:,2),ZerosPoints(:,3),...
            plane(1),plane(2),plane(3),'k','LineStyle','-','LineWidth',1);
    end    
end

end