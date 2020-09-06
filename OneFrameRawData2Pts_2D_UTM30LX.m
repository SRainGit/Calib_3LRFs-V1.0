
%将UTM30LX的一帧原始数据转换为一个二维点集
%rawData:一行数据
%PC_2D：每行代表一个点，行数为点数

function PC_2D=OneFrameRawData2Pts_2D_UTM30LX(rawData)

%单LRF的固定参数
midLaserIndex=540;
angStep_degree=270/1080;
angStep_radian=angStep_degree*pi/180;

%申请存储空间
PC_2D=zeros(length(rawData),2);

%逐点进行坐标解算
for laserIndex=1:length(rawData)
    angle=(laserIndex-midLaserIndex)*angStep_radian;
    distance=rawData(laserIndex);
    
    PC_2D(laserIndex,1)=-sin(angle)*distance; % 这个方向对于UTM30LX是正确的
    PC_2D(laserIndex,2)=cos(angle)*distance;
end

end