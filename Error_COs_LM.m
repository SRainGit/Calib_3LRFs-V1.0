
function Error=Error_COs_LM(COs,NUMBER_LRFs,FixedLRF,CalibLRF,R,T)
nCOs=size(COs,2);
errors=zeros(1,nCOs);

cov_ini_2D=[1,0;0,1];

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
                    c1=(T1+(R1*c1')')';
                    c1_rot=R1*c1;
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
                    
                    vNormal=cross(v1,v2);
                    %                         vNormal=[plane(1);plane(2);plane(3)]; vNormal=vNormal./norm(vNormal);
                    cov_vNormal=-Skew_Symmetric3(v1)*covV_2*Skew_Symmetric3(v1)-Skew_Symmetric3(v2)*covV_1*Skew_Symmetric3(v2);
                    sigma_planar_constriant=sqrt(vNormal'*(covC_1+covC_2)*vNormal+(c1-c2)'*cov_vNormal*(c1-c2)); %注意这里c1-c2的顺序跟下面计算jac矩阵的正负和顺序有关系
                    
                    % Compute the residuals of the co-planarity constraints
                    error_planarity=dot(vNormal,(c1-c2))/sigma_planar_constriant;
                    
                    errors(iCO)=errors(iCO)+error_planarity^2;
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
        cov_vNormal1=-Skew_Symmetric3(v1_1)*covV1_2*Skew_Symmetric3(v1_1)-Skew_Symmetric3(v1_2)*covV1_1*Skew_Symmetric3(v1_2);
        vNormal2=cross(v2_1,v2_2);
        cov_vNormal2=-Skew_Symmetric3(v2_1)*covV2_2*Skew_Symmetric3(v2_1)-Skew_Symmetric3(v2_2)*covV2_1*Skew_Symmetric3(v2_2);
        
        sigma_orthogonal_constraint = sqrt(vNormal1'*R2*cov_vNormal2*R2'*vNormal1 + vNormal2'*R2'*cov_vNormal1*R2*vNormal2);
        
        error_orthogonality = dot(vNormal1,vNormal2)/sigma_orthogonal_constraint;
        
        errors(iCO)=errors(iCO)+error_orthogonality^2;
    end
    % end of oneCO
end

Error=sum(errors)/nCOs;

end











