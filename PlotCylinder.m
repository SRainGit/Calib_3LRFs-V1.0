
%����Բ���棬CorePointΪԲ�������ĵ㣬RotationMatrixΪԪת����ת����
%IsClosed��0Ϊ����գ�1Ϊ���
function PlotCylinder(CorePoint, Radius, Height, RotationMatrix, Color, IsClosed)

%% ���Ʋ���
m=100;%�ָ��ߵ�����
[x,y,z]=cylinder(Radius,m);%������(0,0)ΪԲ�ģ��߶�Ϊ[0,1]���뾶ΪR��Բ��
x=x+CorePoint(1);%ƽ��x��
y=y+CorePoint(2);%ƽ��y�ᣬ��Ϊ(a,b)Ϊ��Բ��Բ��
z=Height*z;%�߶ȷŴ�h��
z=z-Height/2+CorePoint(3);

%��ת����
a1=[x(1,:);y(1,:);z(1,:)];
a2=[x(2,:);y(2,:);z(2,:)];
a1=RotationMatrix*a1;
a2=RotationMatrix*a2;
x1=[a1(1,:);a2(1,:)];
y1=[a1(2,:);a2(2,:)];
z1=[a1(3,:);a2(3,:)];
p1=mesh(x1,y1,z1);%��ͼ


%% ����������
if IsClosed==1
    %������һ����λԲ������
    t=deg2rad(0:360);
    xEndPlane=cos(t);
    yEndPlane=sin(t);
    [xEndPlane,yEndPlane]=meshgrid(xEndPlane,yEndPlane);    
    xEndPlane(xEndPlane.^2+yEndPlane.^2>1)=NaN;
    yEndPlane(xEndPlane.^2+yEndPlane.^2>1)=NaN;
    zUp=CorePoint(3)+Height/2+0*xEndPlane;
    zDown=CorePoint(3)-Height/2+0*xEndPlane;
    
    %Ȼ���ڽ��зŴ��ƽ��
    xEndPlane=Radius*xEndPlane+CorePoint(1);
    yEndPlane=Radius*yEndPlane+CorePoint(2);
    
    %��������ת
    xEndPlane=reshape(xEndPlane,1,size(xEndPlane,1)*size(xEndPlane,2));
    yEndPlane=reshape(yEndPlane,1,size(yEndPlane,1)*size(yEndPlane,2));
    zUp=reshape(zUp,1,size(zUp,1)*size(zUp,2));
    zDown=reshape(zDown,1,size(zDown,1)*size(zDown,2));    
    Pts1=RotationMatrix*[xEndPlane;yEndPlane;zUp];
    Pts2=RotationMatrix*[xEndPlane;yEndPlane;zDown];
    xEndPlane_Up=reshape(Pts1(1,:),size(t,2),size(t,2));
    yEndPlane_Up=reshape(Pts1(2,:),size(t,2),size(t,2));
    zEndPlane_Up=reshape(Pts1(3,:),size(t,2),size(t,2));
    xEndPlane_Down=reshape(Pts2(1,:),size(t,2),size(t,2));
    yEndPlane_Down=reshape(Pts2(2,:),size(t,2),size(t,2));
    zEndPlane_Down=reshape(Pts2(3,:),size(t,2),size(t,2));
    
    %������������
    p2=mesh(xEndPlane_Up,yEndPlane_Up,zEndPlane_Up);
    p3=mesh(xEndPlane_Down,yEndPlane_Down,zEndPlane_Down);
end

%% ������ɫ
set(p1,'FaceColor','white','EdgeColor',Color);
set(p2,'FaceColor','white','EdgeColor',Color);
set(p3,'FaceColor','white','EdgeColor',Color);

end





