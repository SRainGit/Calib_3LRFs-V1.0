%% Generate the simulation 3D points of LRF in its 2D scanning plane
% set the world zero point is on the left down corner of the corridor
% reference: https://blog.csdn.net/smallflyingpig/article/details/51234711?locationNum=3
function Data = SyntheticDataGenerator_LRF(R,T,CorridorWidth,CorridorHeight)
PTS_PER_FRAME=1080;
MAX_DISTANCE=6;
Data=zeros(PTS_PER_FRAME,2);
sigma=0.003;

vectorX=squeeze(R(1,:));
vectorY=squeeze(R(2,:));

%% wall planes (wall1 is the below one, and wall2 is the left one)
% ax+by+cz+d=0
% and set the wall norms are pointing to outside
walls=zeros(4,4);
walls(1,:)=[0,0,-1,0];
walls(2,:)=[-1,0,0,0];
walls(3,:)=[0,0,1,-CorridorHeight];
walls(4,:)=[1,0,0,-CorridorWidth];
dist2Planes=zeros(1,4);
for iWall=1:4
    dist2Planes(iWall)=abs(dot([T,1],walls(iWall,:)));
end

angs=-45:(270/(PTS_PER_FRAME-1)):225;
angs=angs.*pi/180;
noise=sigma*randn(PTS_PER_FRAME,1);
for iLaser=1:PTS_PER_FRAME
    % get laser vector
    ang=angs(iLaser);
    vector=cos(ang)*vectorX+sin(ang)*vectorY;
    %% get the distances to each wall
    signedDistance=zeros(1,4);
    laserLineVector=zeros(4,3);
    for iWall=1:4
        wall=squeeze(walls(iWall,:));
        d_vector=dot(vector,wall(1:3));  % the signed projection length
        if d_vector<=0
            signedDistance(iWall)=Inf;  % set a very large distance
            continue;
        end
        laserLineVector(iWall,:) = vector*(dist2Planes(iWall)/d_vector+noise(iLaser));  % add noise
        signedDistance(iWall) = dist2Planes(iWall)/d_vector;
    end
    [valDis,il]=min(signedDistance);
    laserLineVector_3D=laserLineVector(il,:);
    if norm(laserLineVector_3D)>MAX_DISTANCE
        continue;
    end
    Data(iLaser,1)=dot(laserLineVector_3D,vectorX);
    Data(iLaser,2)=dot(laserLineVector_3D,vectorY);
end


end












