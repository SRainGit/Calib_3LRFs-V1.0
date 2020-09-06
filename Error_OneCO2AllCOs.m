
function Error=Error_OneCO2AllCOs(Walls_AllCOs,oneCO,Poses_AllGroups,R_LRFs,T_LRFs)
Error=0;
iGroup0=oneCO.indexDataGroup;
CO=oneCO.CO;
%% compute the error between each line in "oneCO" to all the lines in "Walls_AllCOs"
for iWall=1:4    
    [R_CO0,T_CO0]=Exp_Matrix(Poses_AllGroups(iGroup0,:),0);
    for iLine0=1:CO(iWall).cntLines
        % extract the endPts in this line
        oneLine0=CO(iWall).lines(iLine0);
        idLRF0=oneLine0.idLRF;
        endPts0=oneLine0.endPts;
        endPts0(:,3)=0;
        R_device0=squeeze(R_LRFs(idLRF0,:,:));
        T_device0=squeeze(T_LRFs(idLRF0,:))';
        [R0,T0]=PoseAddition_RT(R_CO0,T_CO0,R_device0,T_device0);
        endPts0=(R0*endPts0')'+[T0';T0'];
        % for each line in all COs
        for iLine1=1:Walls_AllCOs(iWall).cntLines
            oneLine1=Walls_AllCOs(iWall).lines(iLine1);
            iGroup1=oneLine1.idGroup;
            [R_CO1,T_CO1]=Exp_Matrix(Poses_AllGroups(iGroup1,:),0);
            idLRF1=oneLine1.idLRF;
            endPts1=oneLine1.endPts;
            endPts1(:,3)=0;
            R_device1=squeeze(R_LRFs(idLRF1,:,:));
            T_device1=squeeze(T_LRFs(idLRF1,:))';
            [R1,T1]=PoseAddition_RT(R_CO1,T_CO1,R_device1,T_device1);
            endPts1=(R1*endPts1')'+[T1';T1'];
            Error=Error+VolumeOfTetrahedron_signed([endPts0;endPts1])^2;
        end            
    end    
end

end