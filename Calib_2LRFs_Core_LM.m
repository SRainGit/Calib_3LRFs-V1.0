

function [R_Calib,T_Calib,Errors,updates_ang_t] = Calib_2LRFs_Core_LM(NUMBER_LRFs,FixedLRF,CalibLRF,R_LRFsIni,T_LRFsIni)

global COs

if FixedLRF~=1
    disp('Can not calib without the LRF1 now!');
    return;
end
nCOs=size(COs,2);
R=R_LRFsIni;
T=T_LRFsIni;

updates_ang_t(1,1:3)=RotateMat2EulerAngle_XYZ(squeeze(R_LRFsIni(CalibLRF,:,:)));
updates_ang_t(1,4:6)=squeeze(T_LRFsIni(CalibLRF,:));

bShowFigure = 0 ; % It is used to control whether the drawing is opened.
% bShowFigure = 1 ; % It is used to control whether the drawing is opened.
if bShowFigure
    fig1=figure; h = [-1100 110 1100 680]; set(gcf,'Position',h)
    % h = [-1500 100 1500 980];
    set(gcf,'Position',h);
    rotate3d on; axis equal;
    syms x y z real
end

%% Solve with Levenberg-Marquardt
% Set Levenberg-Marquardt parameters
% lambda = 0.001;
lambda = 100000;
step = 10; % Update step
max_LM_it_lambda = 50;
cntIteration=0;
maxIteration=200;
% tol=10^-24;
tol=10^-30;
% initialize variables
system_DoF = 6;
Hessian=zeros(system_DoF,system_DoF);
Gradient=zeros(system_DoF,1);
cov_ini_2D=[1,0;0,1];
update=ones(6,1);
error=Error_COs_LM(COs,NUMBER_LRFs,FixedLRF,CalibLRF,R_LRFsIni,T_LRFsIni);
diff_error=1;
while (cntIteration<maxIteration && norm(update)>tol && diff_error>tol)
    jac_error_planarity=zeros(1,system_DoF);
    jac_error_orthogonality=zeros(1,system_DoF/2);
    Hessian=zeros(system_DoF,system_DoF);
    Gradient=zeros(system_DoF,1);
    
    %% Transvers each CO
    for iCO=1:nCOs
        % update R* and T* using the changed R and T
        R1 = squeeze(R(FixedLRF,:,:));
        T1 = squeeze(T(FixedLRF,:));
        R2 = squeeze(R(CalibLRF,:,:));
        T2 = squeeze(T(CalibLRF,:));
        
        %% Compute the residual and the Jacobian of the co-planarity constraint
        for iWall=1:4
            % extract data
            cntLines=COs(iCO).CO(iWall).cntLines;
            lines=COs(iCO).CO(iWall).lines;
            
            % skip the wall without enough lines
            if cntLines>=2
                for iLine1=1:cntLines-1
                    for iLine2=iLine1+1:cntLines
                        line1=lines(iLine1);
                        line2=lines(iLine2);
                        iLRF1=line1.idLRF;
                        iLRF2=line2.idLRF;
                        if (iLRF1~=FixedLRF && iLRF1~=CalibLRF) || (iLRF2~=FixedLRF && iLRF2~=CalibLRF)
                            continue;
                        else
                            if iLRF1~=FixedLRF % ensure the iLRF1 is the fixed lRF
                                t=line1; line1=line2; line2=t;
                                iLRF1=line1.idLRF;
                                iLRF2=line2.idLRF;
                            end
                        end
                        % line1
                        l1=line1.coefficients;
                        c1=zeros(1,3);
                        c1(1)=1/l1(1);% according to the regerence!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        c1(2)=(-l1(3)-1)/l1(2);
                        c1(3)=0;
                        v1=line1.vector;
                        endPts1=line1.endPts; endPts1(:,3)=0;
                        % rotation
                        c1_rot=R1*c1';
                        c1=(T1+(R1*c1')')';
                        v1=R1*v1';
                        covC_1=R1(:,1:2)*cov_ini_2D*R1(:,1:2)';
                        covV_1=R1(:,1:2)*cov_ini_2D*R1(:,1:2)';
                        % line2
                        l2=line2.coefficients;
                        c2=zeros(1,3);
                        c2(1)=1/l2(1);% according to the regerence!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        c2(2)=(-l2(3)-1)/l2(2);
                        c2(3)=0;
                        v2=line2.vector;
                        % rotation
%                         c2_rot=R2*c2';
                        c2=(T2+(R2*c2')')';
                        c2_rot=R2*c2;
                        v2=R2*v2';
                        endPts2=line2.endPts; endPts2(:,3)=0;
                        endPts2=[T2;T2]+(R2*endPts2')';
                        covC_2=R2(:,1:2)*cov_ini_2D*R2(:,1:2)';
                        covV_2=R2(:,1:2)*cov_ini_2D*R2(:,1:2)';
                        
                        % Compute the residual and the Jacobian of the orthogonality constraint
                        endPts=[endPts1;endPts2];
                        X=[ones(4,1) endPts(:,1:2)];
                        A = regress(endPts(:,3),X);
                        plane=[A(2),A(3),-1,A(1)]./norm([A(2),A(3),-1]); % ax+by+cz+d=0;
                        
                        if bShowFigure
                            minX=10^9;maxX=-10^9;minY=10^9;maxY=-10^9;minZ=10^9;maxZ=-10^9;
                            plot3(endPts(:,1),endPts(:,2),endPts(:,3),'.');
                            f_plane=plane(1)*x+plane(2)*y+plane(3)*z+plane(4);
                            minX=min(minX,min(endPts(:,1)));
                            minY=min(minY,min(endPts(:,2)));
                            minZ=min(minZ,min(endPts(:,3)));
                            maxX=max(maxX,max(endPts(:,1)));
                            maxY=max(maxY,max(endPts(:,2)));
                            ezimplot3(f_plane,[min(min(minX,minY),minZ)  max(max(maxX,maxY),maxZ)],'r');
                        end
                        
                        vNormal=cross(v1,v2);
%                                                 vNormal=[plane(1);plane(2);plane(3)]; vNormal=vNormal./norm(vNormal);
                        cov_vNormal=-Skew_Symmetric3(v1)*covV_2*Skew_Symmetric3(v1)-Skew_Symmetric3(v2)*covV_1*Skew_Symmetric3(v2);
                        sigma_planar_constriant=sqrt(vNormal'*(covC_1+covC_2)*vNormal+(c1-c2)'*cov_vNormal*(c1-c2)); %注意这里c1-c2的顺序跟下面计算jac矩阵的正负和顺序有关系
                        
                        % Compute the residuals of the co-planarity constraints
                        error_planarity=dot(vNormal,(c1-c2))/sigma_planar_constriant;
                        
                        jac_error_planarity(1,4:6)=(vNormal'*Skew_Symmetric3(c2_rot)+(c2-c1)'*Skew_Symmetric3(v1)*Skew_Symmetric3(v2))/sigma_planar_constriant;
                        jac_error_planarity(1,1:3)=-vNormal'/sigma_planar_constriant;
                        
                        % update
                        Hessian=Hessian+jac_error_planarity'*jac_error_planarity;
                        Gradient=Gradient+jac_error_planarity'*error_planarity;
                    end
                end
            end
        end
        
        %% Compute the residual and the Jacobian of the orthogonality constraint
        % Tag the walls with line pairs (have both LRF1 line and LRF2 line)
        bWithLinePairs=zeros(1,4);
        for iWall=1:4
            cntLines=COs(iCO).CO(iWall).cntLines;
            lines=COs(iCO).CO(iWall).lines;
            bFindLine_LRF1=0; bFindLine_LRF2=0;
            if cntLines>2
                for iLine=1:cntLines
                    iLRF=lines(iLine).idLRF;
                    if iLRF==FixedLRF
                        bFindLine_LRF1=1;
                    elseif iLRF==CalibLRF
                        bFindLine_LRF2=1;
                    end
                end
            else
                continue;
            end
            if bFindLine_LRF1==1 && bFindLine_LRF2==1
                bWithLinePairs(iWall)=1;
            end
        end
        % Compute Jacobian of the orthogonality
        for iWall=1:4
            if bWithLinePairs(iWall)~=1 || bWithLinePairs(mod(iWall,4)+1)~=1
                continue;
            end
            cntLines1=COs(iCO).CO(iWall).cntLines;
            lines1=COs(iCO).CO(iWall).lines;
            cntLines2=COs(iCO).CO(mod(iWall,4)+1).cntLines;
            lines2=COs(iCO).CO(mod(iWall,4)+1).lines;
            for iLine=1:cntLines1
                if lines1(iLine).idLRF==FixedLRF
                    v1_1=lines1(iLine).vector;
                    endPts1_1=lines1(iLine).endPts; endPts1_1(:,3)=0;
                elseif lines1(iLine).idLRF==CalibLRF
                    v1_2=lines1(iLine).vector;
                    endPts1_2=lines1(iLine).endPts; endPts1_2(:,3)=0;
                end
            end
            for iLine=1:cntLines2
                if lines2(iLine).idLRF==FixedLRF
                    v2_1=lines2(iLine).vector;
                    endPts2_1=lines1(iLine).endPts; endPts2_1(:,3)=0;
                elseif lines2(iLine).idLRF==CalibLRF
                    v2_2=lines2(iLine).vector;
                    endPts2_2=lines1(iLine).endPts; endPts2_2(:,3)=0;
                end
            end
            
            v1_1=R1*v1_1'; v2_1=R1*v2_1';
            v1_2=R2*v1_2'; v2_2=R2*v2_2';
            covV1_1=R1(:,1:2)*cov_ini_2D*R1(:,1:2)';
            covV1_2=R1(:,1:2)*cov_ini_2D*R1(:,1:2)';
            covV2_1=R2(:,1:2)*cov_ini_2D*R2(:,1:2)';
            covV2_2=R2(:,1:2)*cov_ini_2D*R2(:,1:2)';
            
            endPts1_2=[T2;T2]+(R2*endPts1_2')';
            endPts2_2=[T2;T2]+(R2*endPts2_2')';
            endPts1=[endPts1_1;endPts1_2];
            endPts2=[endPts2_1;endPts2_2];
            
            vNormal1=cross(v1_1,v1_2);
%                         X=[ones(4,1) endPts1(:,1:2)];
%                         A = regress(endPts1(:,3),X);
%                         plane1=[A(2),A(3),-1,A(1)]./norm([A(2),A(3),-1]); % ax+by+cz+d=0;
%                         vNormal1=[plane1(1);plane1(2);plane1(3)]; vNormal1=vNormal1./norm(vNormal1);
            cov_vNormal1=-Skew_Symmetric3(v1_1)*covV1_2*Skew_Symmetric3(v1_1)-Skew_Symmetric3(v1_2)*covV1_1*Skew_Symmetric3(v1_2);
            vNormal2=cross(v2_1,v2_2);
%                         X=[ones(4,1) endPts2(:,1:2)];
%                         A = regress(endPts2(:,3),X);
%                         plane2=[A(2),A(3),-1,A(1)]./norm([A(2),A(3),-1]); % ax+by+cz+d=0;
%                         vNormal2=[plane2(1);plane2(2);plane2(3)]; vNormal2=vNormal2./norm(vNormal2);
            cov_vNormal2=-Skew_Symmetric3(v2_1)*covV2_2*Skew_Symmetric3(v2_1)-Skew_Symmetric3(v2_2)*covV2_1*Skew_Symmetric3(v2_2);
            
            sigma_orthogonal_constraint = sqrt(vNormal1'*R2*cov_vNormal2*R2'*vNormal1 + vNormal2'*R2'*cov_vNormal1*R2*vNormal2);
            
            error_orthogonality = dot(vNormal1,vNormal2)/sigma_orthogonal_constraint;
            
            jac_error_orthogonality=-(vNormal1'*Skew_Symmetric3(v2_1)*Skew_Symmetric3(v2_2) +...
                vNormal2'*Skew_Symmetric3(v1_1)*Skew_Symmetric3(v1_2))/sigma_orthogonal_constraint;
            
            % update
            Hessian(4:6,4:6)=Hessian(4:6,4:6)+jac_error_orthogonality'*jac_error_orthogonality;
            Gradient(4:6,:)=Gradient(4:6,:)+jac_error_orthogonality'*error_orthogonality;
        end
        % end of oneCO
    end
    
    %% Update
    update=-inv(Hessian+lambda.*diag(diag(Hessian)))*Gradient;
    %     update=-update;
    [R_update,T_update]=Exp_Matrix(update,0);
    [R_temp,T_temp]=PoseAddition_RT(R_update,T_update,squeeze(R(CalibLRF,:,:)),T(CalibLRF,:)');
    R_=R; T_=T;
    R_(CalibLRF,:,:)=R_temp;
    T_(CalibLRF,:)=T_temp';
    
    new_error=Error_COs_LM(COs,NUMBER_LRFs,FixedLRF,CalibLRF,R_,T_);
    diff_error=error-new_error;
    if diff_error>0
        lambda = lambda/step;
        R = R_;
        T = T_;
        Errors(cntIteration+1)=error;
        Errors(cntIteration+2)=new_error;
        error = new_error;
        
        updates_ang_t(cntIteration+2,1:3)=RotateMat2EulerAngle_XYZ(squeeze(R(CalibLRF,:,:)));
        updates_ang_t(cntIteration+2,4:6)=squeeze(T(CalibLRF,:));
        
        cntIteration=cntIteration+1;
    else
        LM_it=0;
        while LM_it<max_LM_it_lambda && diff_error<0
            lambda=lambda*step;
            update=-inv(Hessian+lambda.*diag(diag(Hessian)))*Gradient;
            [R_update,T_update]=Exp_Matrix(update,0);
            [R_temp,T_temp]=PoseAddition_RT(R_update,T_update,squeeze(R(CalibLRF,:,:)),T(CalibLRF,:)');
            R_=R; T_=T;
            R_(CalibLRF,:,:)=R_temp;
            T_(CalibLRF,:)=T_temp';
            
            new_error=Error_COs_LM(COs,NUMBER_LRFs,FixedLRF,CalibLRF,R_,T_);
            diff_error=error-new_error;
            if diff_error>0
                R = R_;
                T = T_;
                Errors(cntIteration+1)=error;
                Errors(cntIteration+2)=new_error;
                error = new_error;
        
                updates_ang_t(cntIteration+2,1:3)=RotateMat2EulerAngle_XYZ(squeeze(R(CalibLRF,:,:)));
                updates_ang_t(cntIteration+2,4:6)=squeeze(T(CalibLRF,:));

                cntIteration=cntIteration+1;
            end
            LM_it=LM_it+1;
        end
    end

    errorsCO=zeros(nCOs,1);
    for iCO=1:nCOs
        oneCO=COs(iCO);
        errorsCO(iCO)=ErrorInOneCO_TwoLRFs(oneCO,FixedLRF,CalibLRF,R,T);
    end
    [maxError,indexMaxError]=max(errorsCO)
    meanError=mean(errorsCO);
    stdError=std(errorsCO);
    mask=(errorsCO>2*meanError+2*stdError);
    if sum(mask)>0
        COs(indexMaxError)=[];
        nCOs=nCOs-1
    end
end
nCOs;
R_Calib=R;
T_Calib=T;

end






















