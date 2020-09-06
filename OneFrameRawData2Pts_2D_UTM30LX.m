
%��UTM30LX��һ֡ԭʼ����ת��Ϊһ����ά�㼯
%rawData:һ������
%PC_2D��ÿ�д���һ���㣬����Ϊ����

function PC_2D=OneFrameRawData2Pts_2D_UTM30LX(rawData)

%��LRF�Ĺ̶�����
midLaserIndex=540;
angStep_degree=270/1080;
angStep_radian=angStep_degree*pi/180;

%����洢�ռ�
PC_2D=zeros(length(rawData),2);

%�������������
for laserIndex=1:length(rawData)
    angle=(laserIndex-midLaserIndex)*angStep_radian;
    distance=rawData(laserIndex);
    
    PC_2D(laserIndex,1)=-sin(angle)*distance; % ����������UTM30LX����ȷ��
    PC_2D(laserIndex,2)=cos(angle)*distance;
end

end