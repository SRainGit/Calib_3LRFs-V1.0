
% 使用RANSAC检测2D空间的直线
% 参考：http://www.cnblogs.com/tiandsp/archive/2013/06/03/3115743.html
% Pts2D的每一行为一个二维点
% 输出的lines里每行为一个line: ax+by+c=0 
function [cntLines,lines,nInliers,inliers,outliers] = RANSAC_DetectLines_2D(...
    Pts2D,nPts,nearestDist,farthestDist,distThreshold,minLineLength,minInliersPts,maxIterTimes,fixedInerIterTimes)

%% 初始化
sigma=distThreshold;     %最大距离偏差
cntLines=0;%检测出的直线数量
lines=zeros(1,3);%初始化申请一条直线的空间
nInliers(1)=0;
inliers=zeros(1,1,2);
nOutliers=nPts;%剩余的点数
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
        sampleIndex=randi(nOutliers,1,2);  %产生两个随机索引，找样本用，floor向下取整
        samp1=outliers(sampleIndex(1),:);
        samp2=outliers(sampleIndex(2),:);        
        %避免两采样点之间的距离过小
        if norm(samp1-samp2)<minLineLength
            cntInerIterTimes=cntInerIterTimes+1;
            cntIters=cntIters+1;
            continue;
        end        
        %计算采样后生成的直线模型
        line=TwoPts2Line_2D([samp1;samp2]);    %对两个数据拟合出直线，或其他变种拟合方法        
        %评估该模型的inliers是否足够
        mask=abs(line*[outliers ones(size(outliers,1),1)]')<sigma;    %求每个数据到拟合直线的距离
        sumDist=sum(mask);    %计算数据距离直线小于一定阈值的数据的个数
        if sumDist>preSumDist    %找到符合拟合直线数据最多的拟合直线
            preSumDist=sumDist;
            bestLine=line;          %找到最好的拟合直线
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






