
%% result: let "R*Pts2=Pts1"
function [R,isCredible]=RotationRegistrationBySVD(Pts1,Pts2,cntParis)
isCredible=1;
H=zeros(3,3);
for i=1:cntParis
    oneLSAxis=squeeze(Pts1(:,i));
    oneIMUAxis=squeeze(Pts2(:,i));
    H=H+(oneIMUAxis*oneLSAxis');
end
[U,S,V]=svd(H);
R=V*U';
if det(R) < 0
    isCredible=-1;
    disp('Reflection detected');
    V(:,3) = V(:,3).*-1;
    R = V*U';
end

end