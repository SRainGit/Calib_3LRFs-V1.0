
% ʹ��RANSAC���2D�ռ��ֱ��
% �ο���http://www.cnblogs.com/tiandsp/archive/2013/06/03/3115743.html
% Pts2D��ÿһ��Ϊһ����ά��
% �����lines��ÿ��Ϊһ��line: ax+by+c=0 
function [cntLines,lines,nInliers,inliers,outliers] = RANSAC_DetectLines_2D(...
    Pts2D,nPts,nearestDist,farthestDist,distThreshold,minLineLength,minInliersPts,maxIterTimes,fixedInerIterTimes)

%% ��ʼ��
sigma=distThreshold;     %������ƫ��
cntLines=0;%������ֱ������
lines=zeros(1,3);%��ʼ������һ��ֱ�ߵĿռ�
nInliers(1)=0;
inliers=zeros(1,1,2);
nOutliers=nPts;%ʣ��ĵ���
outliers=Pts2D;
beyondSightPts=[0,0];
nBeyondSightPts=0;

%% Remove beyondSightPts
i=1;
while i<=nOutliers
    if norm(outliers(i,:))>farthestDist || norm(outliers(i,:))<nearestDist
        nBeyondSightPts=nBeyondSightPts+1;
        beyondSightPts(nBeyondSightPts,:)=outliers(i,:);
        outliers(i,:)=[];
        nOutliers=nOutliers-1;
    end
    i=i+1;
end

%% The big loop
cntIters=1;
while cntIters<maxIterTimes && nOutliers>minInliersPts
    
    %% The inner loop
    preSumDist=0;
    cntInerIterTimes=1;
    while cntInerIterTimes<fixedInerIterTimes        
        % sammpling
        sampleIndex=randi(nOutliers,1,2);  %������������������������ã�floor����ȡ��
        samp1=outliers(sampleIndex(1),:);
        samp2=outliers(sampleIndex(2),:);        
        %������������֮��ľ����С
        if norm(samp1-samp2)<minLineLength
            cntInerIterTimes=cntInerIterTimes+1;
            cntIters=cntIters+1;
            continue;
        end        
        %������������ɵ�ֱ��ģ��
        line=TwoPts2Line_2D([samp1;samp2]);    %������������ϳ�ֱ�ߣ�������������Ϸ���        
        %������ģ�͵�inliers�Ƿ��㹻
        mask=abs(line*[outliers ones(size(outliers,1),1)]')<sigma;    %��ÿ�����ݵ����ֱ�ߵľ���
        sumDist=sum(mask);    %�������ݾ���ֱ��С��һ����ֵ�����ݵĸ���
        if sumDist>preSumDist    %�ҵ��������ֱ�������������ֱ��
            preSumDist=sumDist;
            bestLine=line;          %�ҵ���õ����ֱ��
        end        
        cntInerIterTimes=cntInerIterTimes+1;
        cntIters=cntIters+1;
    end    
    
    %% After the inner loop
    % if there is not enough inliners
    if preSumDist<minInliersPts
        cntIters=cntIters+1;
        continue;
    end
    % then record this line
    cntLines=cntLines+1;
    lines(cntLines,:)=bestLine;
    
    % inliers and outliers
    mask=abs(bestLine*[outliers ones(size(outliers,1),1)]')<sigma;
    nInliers(cntLines)=sum(mask);
    inliers(cntLines,1:nInliers(cntLines),:)=outliers(mask==1,:);
    nOutliers=length(mask)-nInliers(cntLines);
    outliers=outliers(mask==0,:);    
    
    % finness the fitting using polyfit
    x=squeeze(inliers(cntLines,1:nInliers(cntLines),1));
    y=squeeze(inliers(cntLines,1:nInliers(cntLines),2));
    fitLine=polyfit(x,y,1);
    fitLine=[fitLine(1) -1 fitLine(2)]./sqrt(fitLine(1)^2+1); % Convert to a general linear equation
        
    % if the polyfit result is better than the original line, use the fit line
    x=inliers(cntLines,1: nInliers(cntLines),1)';
    y=inliers(cntLines,1: nInliers(cntLines),2)';
    sumDist_Ori=sum(abs(squeeze(lines(cntLines,:))*[x y ones(nInliers(cntLines),1)]'));
    sumDist_polyfit=sum(abs(fitLine*[x y ones(nInliers(cntLines),1)]'));
    if sumDist_polyfit<sumDist_Ori
        lines(cntLines,:)=fitLine;
    end     
end

if nBeyondSightPts>0
    outliers=[outliers;beyondSightPts];
end

end






