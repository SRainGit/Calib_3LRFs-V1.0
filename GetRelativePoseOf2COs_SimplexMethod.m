
%% Set the CO0 as the refer frame, get the relative pose from CO1 to CO0
function [CO1_re,Errors]=GetRelativePoseOf2COs_SimplexMethod(CO0,CO1,R_LRFs,T_LRFs,maxIterTimes,maxBound_Ang,maxBound_Trans)
CO1_re=CO1;
Errors=zeros(2,1);
system_DoF=6;
pose_t_1=CO1.Pose.Exp(1:3);
pose_w_1=CO1.Pose.Exp(4:6);

%% Solving with simplex method
alpha=1;
beta=0.5;
gamma=2;
cntIter=0;
sigma=1e-6;
% initialize the start "points"
x=zeros(system_DoF+1,system_DoF); % 'system_DoF+1' vertexes in the 'system_DoF' space
errors=zeros(system_DoF+1,1);
cntIniTimes=0;
while max(errors)<1 && min(errors)<0.01
    cntIniTimes=cntIniTimes+1;
%     disp(cntIniTimes);
    x(1,:)=[pose_t_1' pose_w_1'];
    for i=1:system_DoF
        rand_trans=(rand([1,3])-0.5)*2*maxBound_Trans;
        rand_w=(rand([1,3])-0.5)*2*maxBound_Ang;
        x(i+1,:)=[rand_trans+x(1,1:3) rand_w+x(1,4:6)];
    end
    for i=1:system_DoF+1
        [CO1.Pose.R, CO1.Pose.T]=Exp_Matrix(x(i,:),0);
        errors(i)=ErrorIn2COs_AllLRFs(CO0,CO1,R_LRFs,T_LRFs);
    end
    if cntIniTimes>100
        disp('Please check the iniPoses.');
        return;
    end
end
% Iteration
for i=1:system_DoF+1
    [CO1.Pose.R, CO1.Pose.T]=Exp_Matrix(x(i,:),0);
    errors(i)=ErrorIn2COs_AllLRFs(CO0,CO1,R_LRFs,T_LRFs);
end
while cntIter<maxIterTimes
    cntIter=cntIter+1;
    
    % the val and index of min&max
    [val_il,il]=min(errors);
    [val_ih,ih]=max(errors);
    % the index with second highest
    errors_=errors;
    errors_(ih)=val_il-10;
    [val_ih2,ih2]=max(errors_);
    % the core point except the ih vertex
    x_core=(1/system_DoF).*(sum(x,1)-x(ih,:));
    % reflection
    x_refelct=x_core+alpha*(x_core-x(ih,:));
    [CO1.Pose.R, CO1.Pose.T]=Exp_Matrix(x_refelct,0);
    error_refelct=ErrorIn2COs_AllLRFs(CO0,CO1,R_LRFs,T_LRFs);
    if error_refelct>=val_il
        if error_refelct>=val_ih2
            % contraction£® ’Àı£©
            if error_refelct<val_ih
                x_contract=x_core+beta*(x_refelct-x_core);
            else
                x_contract=x_core+beta*(x(ih,:)-x_core);
            end
            % get error_contract
            [CO1.Pose.R, CO1.Pose.T]=Exp_Matrix(x_contract,0);
            error_contract=ErrorIn2COs_AllLRFs(CO0,CO1,R_LRFs,T_LRFs);
            if error_contract<min(val_ih,error_refelct)
                x(ih,:)=x_contract;
                errors(ih)=error_contract;
            else
                % shrink£®Àı±ﬂ£©
                for i=1:system_DoF+1
                    x(i,:)=0.5*(x(il,:)+x(i,:));
                    [CO1.Pose.R, CO1.Pose.T]=Exp_Matrix(x(i,:),0);
                    errors(i)=ErrorIn2COs_AllLRFs(CO0,CO1,R_LRFs,T_LRFs);
                end
            end
        else
            x(ih,:)=x_refelct;
            errors(ih)=error_refelct;
        end
    else
        % extand
        x_expand=x_core+gamma*(x_refelct-x_core);
        % get error_extend
        [CO1.Pose.R, CO1.Pose.T]=Exp_Matrix(x_expand,0);
        error_expand=ErrorIn2COs_AllLRFs(CO0,CO1,R_LRFs,T_LRFs);
        % chose the reflect or expand
        if error_expand>error_refelct
            x(ih,:)=x_refelct;
            errors(ih)=error_refelct;
        else
            x(ih,:)=x_expand;
            errors(ih)=error_expand;
        end
    end
    
    % get error_core
    x_core=(1/(system_DoF+1)).*sum(x,1);
    [CO1.Pose.R, CO1.Pose.T]=Exp_Matrix(x_core,0);
    Errors(cntIter)=ErrorIn2COs_AllLRFs(CO0,CO1,R_LRFs,T_LRFs);
%     cnt=0;
%     for i=1:system_DoF
%         for j=i+1:system_DoF+1
%             cnt=cnt+1;
%             edgesLength(cnt)=norm(x(i,:)-x(j,:));
%         end
%     end
    
    % condition to end
    sumDistance2Core=0;
    for i=1:system_DoF+1
        sumDistance2Core=sumDistance2Core+norm(x(i,:)-x_core);
    end
    if sumDistance2Core<sigma
        break;
    end
end

CO1_re.Pose.Exp=x_core';
[CO1_re.Pose.R, CO1_re.Pose.T]=Exp_Matrix(x_core,0);
end






















