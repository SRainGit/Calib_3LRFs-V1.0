

function [R_Calib,T_Calib,Errors] = Calib_2LRFs_Core_LM_Terahedron(COs,NUMBER_LRFs,FixedLRF,CalibLRF,R_LRFsIni,T_LRFsIni)
if FixedLRF~=1
    disp('Can not calib without the LRF1 now!');
    return;
end
nCOs=size(COs,2);
R=R_LRFsIni;
T=T_LRFsIni;


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
lambda = 0.0001;
% step = 10; % Update step
step = 10; % Update step
max_LM_it_lambda = 20;
cntIteration=0;
maxIteration=40;
tol=10^-24;
% tol=10^-30;
% initialize variables
system_DoF = 6;
update=ones(6,1);
cov_ini_2D=[1,0;0,1];
error=Error_COs_LM(COs,NUMBER_LRFs,FixedLRF,CalibLRF,R_LRFsIni,T_LRFsIni);
diff_error=1;
while (cntIteration<maxIteration && norm(update)>tol && diff_error>tol)
    jac_error_planarity=zeros(1,system_DoF);
    jac_error_orthogonality=zeros(1,system_DoF/2);
    Hessian=zeros(system_DoF,system_DoF);
    Gradient=zeros(system_DoF,1);
    
    %% update R* and T* using the changed R and T
    R1 = squeeze(R(FixedLRF,:,:));
    T1 = squeeze(T(FixedLRF,:));
    R2 = squeeze(R(CalibLRF,:,:));
    T2 = squeeze(T(CalibLRF,:));
    
%     if bShowFigure
%         jacbians_w=zeros(nCOs*4,3);
%         jacbians_t=zeros(nCOs*4,3);
%         cntJacbians=0;
%     end
    for iCO=1:nCOs
        %% generate all the tetrahedrons in one CO
        OneCO=COs(iCO);
        errors_walls=zeros(4,1);
        clear Walls;
        for iWall=1:4
            cntTetrahedrons=0;
            Walls(iWall).cntTetrahedrons=0;
            Walls(iWall).TotalVolume=0;
            % extract data
            cntLines=OneCO.CO(iWall).cntLines;
            lines=OneCO.CO(iWall).lines;
            % generating
            if cntLines>1  % skip the wall without enough lines
                % splice all inlierss
                clear allInliers
                % transverse all the lines
                for iLine1=1:cntLines-1
                    for iLine2=iLine1+1:cntLines
                        iLRF1=lines(iLine1).idLRF;
                        iLRF2=lines(iLine2).idLRF;
                        if (iLRF1==FixedLRF && iLRF2==CalibLRF) || (iLRF2==FixedLRF && iLRF1==CalibLRF)
                            % ensure the line1 is one line on the fixed LRF
                            if iLRF1==FixedLRF && iLRF2==CalibLRF
                                line1=lines(iLine1);
                                line2=lines(iLine2);
                            else
                                line1=lines(iLine2);
                                line2=lines(iLine1);
                            end
                            % extract and record endPts
                            endPts1=line1.endPts;
                            endPts1(:,3)=0;
                            endPts1_=(R1*endPts1')'+[T1;T1];
                            endPts2=line2.endPts;
                            endPts2(:,3)=0;
                            endPts2_=(R2*endPts2')'+[T2;T2];
                            % input vertices of Tetrahedron
                            cntTetrahedrons=cntTetrahedrons+1;
                            Walls(iWall).Tetrahedrons(cntTetrahedrons).Points_2D(1:2,:)=endPts1; % pt11 and pt 12
                            Walls(iWall).Tetrahedrons(cntTetrahedrons).Points_2D(3:4,:)=endPts2; % pt21 and pt 22
                            volume_signed=VolumeOfTetrahedron_signed([endPts1_;endPts2_]);
                            Walls(iWall).Tetrahedrons(cntTetrahedrons).Volume_signed=volume_signed;
                        end
                    end
                end
                Walls(iWall).cntTetrahedrons=cntTetrahedrons;
            end
            % end of this wall
        end
        
        %% Compute the residual and the Jacobian of the co-planarity constraint
        for iWall = 1:4
            for iTetrahedron=1:Walls(iWall).cntTetrahedrons  % skip the wall with no tetrahedrons automatically
                % extract points
                Points_2D=Walls(iWall).Tetrahedrons(iTetrahedron).Points_2D;
                p11=Points_2D(1,:)'; p12=Points_2D(2,:)';
                p21=Points_2D(3,:)'; p22=Points_2D(4,:)';
                vP11P12=p12-p11;
                error_planarity=Walls(iWall).Tetrahedrons(iTetrahedron).Volume_signed;
                
                % compute co-planarity jacobian error
                jac_error_planarity(1,4:6)=Skew_Symmetric3(R1*vP11P12)*Skew_Symmetric3(R2*p21+T2')'*(R2*p22+T2'-R1*p11-T1')-...
                    Skew_Symmetric3(R2*p22+T2')*cross(R1*vP11P12,(R2*p21+T2'-R1*p11-T1'));
                jac_error_planarity(1,1:3)=Skew_Symmetric3(R1*vP11P12)*(R2*p21+R2*p22+2*T2'-2*R1*p11-2*T1');
                jac_error_planarity=jac_error_planarity./6;
                
                % Hessian & Gradient
                Hessian=Hessian+jac_error_planarity'*jac_error_planarity;
                Gradient=Gradient+jac_error_planarity'*error_planarity;
            end
        end
        
        %% Compute the residual and the Jacobian of the orthogonality constraint
        % Tag the walls with line pairs (have both LRF1 line and LRF2 line)
        bWithLinePairs=zeros(1,4);
        for iWall=1:4
            cntLines=OneCO.CO(iWall).cntLines;
            lines=OneCO.CO(iWall).lines;
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
            cov_vNormal1=-Skew_Symmetric3(v1_1)*covV1_2*Skew_Symmetric3(v1_1)-Skew_Symmetric3(v1_2)*covV1_1*Skew_Symmetric3(v1_2);
            vNormal2=cross(v2_1,v2_2);
            cov_vNormal2=-Skew_Symmetric3(v2_1)*covV2_2*Skew_Symmetric3(v2_1)-Skew_Symmetric3(v2_2)*covV2_1*Skew_Symmetric3(v2_2);
            
            sigma_orthogonal_constraint = sqrt(vNormal1'*R2*cov_vNormal2*R2'*vNormal1 + vNormal2'*R2'*cov_vNormal1*R2*vNormal2);
%             sigma_orthogonal_constraint=1;
            error_orthogonality = dot(vNormal1,vNormal2)/sigma_orthogonal_constraint;
            
            jac_error_orthogonality=-(vNormal1'*Skew_Symmetric3(v2_1)*Skew_Symmetric3(v2_2) +...
                vNormal2'*Skew_Symmetric3(v1_1)*Skew_Symmetric3(v1_2))/sigma_orthogonal_constraint;
            
            % update
            error_orthogonality=error_orthogonality*100;
            jac_error_orthogonality=jac_error_orthogonality*100;
            Hessian(4:6,4:6)=Hessian(4:6,4:6)+jac_error_orthogonality'*jac_error_orthogonality;
            Gradient(4:6,:)=Gradient(4:6,:)+jac_error_orthogonality'*error_orthogonality;
        end
        
        % end in this CO
    end
       
    
    %% Update
    update=-inv(Hessian+lambda.*diag(diag(Hessian)))*Gradient;
    [R_update,T_update]=Exp_Matrix(update,0);
    [R_temp,T_temp]=PoseAddition_RT(squeeze(R(CalibLRF,:,:)),T(CalibLRF,:)',R_update,T_update);
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
        cntIteration=cntIteration+1;
    else
        LM_it=0;
        while LM_it<max_LM_it_lambda && diff_error<0
            lambda=lambda*step;
            update=-inv(Hessian+lambda.*diag(diag(Hessian)))*Gradient;
            [R_update,T_update]=Exp_Matrix(update,0);
            [R_temp,T_temp]=PoseAddition_RT(squeeze(R(CalibLRF,:,:)),T(CalibLRF,:)',R_update,T_update);
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
                cntIteration=cntIteration+1;
            end
            LM_it=LM_it+1;
        end
    end
end

R_Calib=R;
T_Calib=T;

end






















