

function M=Skew_Symmetric3(v)
M=zeros(3,3);
M(1,1)=0; M(1,2)=-v(3); M(1,3)=v(2);
M(2,1)=v(3); M(2,2)=0; M(2,3)=-v(1);
M(3,1)=-v(2); M(3,2)=v(1); M(3,3)=0;
end