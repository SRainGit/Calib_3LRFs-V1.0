
% the solver
function [R_Calib,T_Calib,Errors,updates_ang_t] = Calib_2LRFs_Core_SimplexAlgorithm(NUMBER_LRFs,FixedLRF,CalibLRF,R_LRFsIni,T_LRFsIni)

threshold=0.1;

global COs
COs_bkp=COs;

% if FixedLRF~=1
%     disp('Can not calib without the LRF1 now!');
%     return;
% end

nCOs=size(COs,2);
nCOs_ori=nCOs;
bBadCOsFilter=0;

Errors=zeros(2,1);
iCheckStep=10;
updates_ang_t(1,1:3)=RotateMat2EulerAngle_XYZ(squeeze(R_LRFsIni(CalibLRF,:,:)));
updates_ang_t(1,4:6)=squeeze(T_LRFsIni(CalibLRF,:));

system_DoF=6;
degree2radian=pi/180;
radian2degree=180/pi;
Eulers_LRFsini=zeros(NUMBER_LRFs,3);
for i=1:NUMBER_LRFs
    Eulers_LRFsini(i,:)=RotateMat2EulerAngle_XYZ(squeeze(R_LRFsIni(i,:,:)));
end

bShowFigure = 0 ; % It is used to control whether the drawing is opened.
% bShowFigure = 1 ; % It is used to control whether the drawing is opened.
if bShowFigure
    fig1=figure; h = [-1100 110 1100 680]; set(gcf,'Position',h)
    % h = [-1500 100 1500 980];
    set(gcf,'Position',h);
    rotate3d on; axis equal;
    syms x y z real
end

%% Solving with simplex method
alpha=1;
beta=0.5;
gamma=2;
cntTaltoalIter=0;
cntIter=0;
maxTotalIterTimes=2000;
maxIterTimes=1000;
sigma=1e-4;
% initialize the start "points"
maxBound_Ang=20;
maxBound_Trans=0.5;
x=zeros(system_DoF+1,system_DoF); % 'system_DoF+1' vertexes in the 'system_DoF' space
edgesLength=zeros(nchoosek(system_DoF+1,2),1);
errors=zeros(system_DoF+1,1);
R_temp=R_LRFsIni; T_temp=T_LRFsIni;
cntIniTimes=0;
while max(errors)<1 && min(errors)<0.01
    cntIniTimes=cntIniTimes+1
    x(1,:)=[squeeze(Eulers_LRFsini(CalibLRF,:)) squeeze(T_LRFsIni(CalibLRF,:))];
    for i=1:system_DoF
        rand_eulers=(rand([1,3])-0.5)*2*maxBound_Ang;
        rand_trans=(rand([1,3])-0.5)*2*maxBound_Trans;
        x(i+1,:)=[rand_eulers+x(1,1:3) rand_trans+x(1,4:6)];
    end
    x(:,1:3)=x(:,1:3).*degree2radian;
    for i=1:system_DoF+1
        R_temp(CalibLRF,:,:)=EulerAngle2RotateMat(x(i,1),x(i,2),x(i,3),'xyz');
        T_temp(CalibLRF,:)=x(i,4:6);
        errors(i)=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_temp,T_temp,bBadCOsFilter);
    end
    if cntIniTimes>100
        disp('Please check the iniPoses.');
        return;
    end
end
% Iteration
for i=1:system_DoF+1
    R_temp(CalibLRF,:,:)=EulerAngle2RotateMat(x(i,1),x(i,2),x(i,3),'xyz');
    T_temp(CalibLRF,:)=x(i,4:6);
    errors(i)=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_temp,T_temp,bBadCOsFilter);
end
while cntTaltoalIter<maxTotalIterTimes && cntIter<maxIterTimes
    cntTaltoalIter=cntTaltoalIter+1;
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
    R_temp(CalibLRF,:,:)=EulerAngle2RotateMat(x_refelct(1),x_refelct(2),x_refelct(3),'xyz');
    T_temp(CalibLRF,:)=x_refelct(4:6);
    error_refelct=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_temp,T_temp,bBadCOsFilter);
    if error_refelct>=val_il
        if error_refelct>=val_ih2
            % contraction£® ’Àı£©
            if error_refelct<val_ih
                x_contract=x_core+beta*(x_refelct-x_core);
            else
                x_contract=x_core+beta*(x(ih,:)-x_core);
            end
            % get error_contract
            R_temp(CalibLRF,:,:)=EulerAngle2RotateMat(x_contract(1),x_contract(2),x_contract(3),'xyz');
            T_temp(CalibLRF,:)=x_contract(4:6);
            error_contract=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_temp,T_temp,bBadCOsFilter);
            if error_contract<min(val_ih,error_refelct)
                x(ih,:)=x_contract;
                errors(ih)=error_contract;
            else
                % shrink
                for i=1:system_DoF+1
                    x(i,:)=0.5*(x(il,:)+x(i,:));
                    R_temp(CalibLRF,:,:)=EulerAngle2RotateMat(x(i,1),x(i,2),x(i,3),'xyz');
                    T_temp(CalibLRF,:)=x(i,4:6);
                    errors(i)=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_temp,T_temp,bBadCOsFilter);
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
        R_temp(CalibLRF,:,:)=EulerAngle2RotateMat(x_expand(1),x_expand(2),x_expand(3),'xyz');
        T_temp(CalibLRF,:)=x_expand(4:6);
        error_expand=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_temp,T_temp,bBadCOsFilter);
        % chose the reflect or expand
        if error_expand>error_refelct
            x(ih,:)=x_refelct;
            errors(ih)=error_refelct;
        else
            x(ih,:)=x_expand;
            errors(ih)=error_expand;
        end
    end
    
    if rem(cntIter,iCheckStep)==0
        bBadCOsFilter=1;
    else
        bBadCOsFilter=0;
    end
    
    % get error_core
    x_core=(1/(system_DoF+1)).*sum(x,1);
    R_temp(CalibLRF,:,:)=EulerAngle2RotateMat(x_core(1),x_core(2),x_core(3),'xyz');
    T_temp(CalibLRF,:)=x_core(4:6);
    Errors(cntTaltoalIter)=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_temp,T_temp,bBadCOsFilter);
    
    updates_ang_t(cntTaltoalIter,1:3)=x(1:3).*radian2degree;
    updates_ang_t(cntTaltoalIter,4:6)=squeeze(T_temp(CalibLRF,:));
    
    % condition to end
    sumDistance2Core=0;
    for i=1:system_DoF+1
        sumDistance2Core=sumDistance2Core+norm(x(i,:)-x_core);
    end    
    disp(['cntTaltoalIter=',num2str(cntTaltoalIter),',cntIter=',num2str(cntIter),...
        ',Error=',num2str(Errors(cntTaltoalIter)),',sumDist=',num2str(sumDistance2Core)]);
    
    % avioid to get stuck
    stuckLength=100;
    if cntIter>stuckLength
        if max(Errors(cntTaltoalIter-stuckLength:cntTaltoalIter))-...
                min(Errors(cntTaltoalIter-stuckLength:cntTaltoalIter))<0.001...
                || sumDistance2Core<sigma
%             if Errors(cntTaltoalIter)>0.02
            if Errors(cntTaltoalIter)>threshold
                COs=COs_bkp;
                cntIter=0;
                x(1,:)=[squeeze(Eulers_LRFsini(CalibLRF,:)) squeeze(T_LRFsIni(CalibLRF,:))];
                for i=1:system_DoF
                    rand_eulers=(rand([1,3])-0.5)*2*maxBound_Ang;
                    rand_trans=(rand([1,3])-0.5)*2*maxBound_Trans;
                    x(i+1,:)=[rand_eulers+x(1,1:3) rand_trans+x(1,4:6)];
                end
                x(:,1:3)=x(:,1:3).*degree2radian;
                for i=1:system_DoF+1
                    R_temp(CalibLRF,:,:)=EulerAngle2RotateMat(x(i,1),x(i,2),x(i,3),'xyz');
                    T_temp(CalibLRF,:)=x(i,4:6);
                    errors(i)=Error_COs(NUMBER_LRFs,FixedLRF,CalibLRF,R_temp,T_temp,bBadCOsFilter);
                end
            else
                break;
            end
        end
    end
end
nCOs_ori
nGOs_lefted=size(COs,2)

R_Calib=R_LRFsIni; T_Calib=T_LRFsIni;
R_Calib(CalibLRF,:,:)=EulerAngle2RotateMat(x_core(1),x_core(2),x_core(3),'xyz');
T_Calib(CalibLRF,:)=x_core(4:6);
end






















