
%绘制圆柱面，CorePoint为圆柱面中心点，RotationMatrix为元转的旋转矩阵
%IsClosed：0为不封闭，1为封闭
function PlotCylinder(CorePoint, Radius, Height, RotationMatrix, Color, IsClosed)

%% 绘制侧面
m=100;%分割线的条数
[x,y,z]=cylinder(Radius,m);%创建以(0,0)为圆心，高度为[0,1]，半径为R的圆柱
x=x+CorePoint(1);%平移x轴
y=y+CorePoint(2);%平移y轴，改为(a,b)为底圆的圆心
z=Height*z;%高度放大h倍
z=z-Height/2+CorePoint(3);

%旋转操作
a1=[x(1,:);y(1,:);z(1,:)];
a2=[x(2,:);y(2,:);z(2,:)];
a1=RotationMatrix*a1;
a2=RotationMatrix*a2;
x1=[a1(1,:);a2(1,:)];
y1=[a1(2,:);a2(2,:)];
z1=[a1(3,:);a2(3,:)];
p1=mesh(x1,y1,z1);%绘图


%% 绘制上下面
if IsClosed==1
    %先制作一个单位圆的数据
    t=deg2rad(0:360);
    xEndPlane=cos(t);
    yEndPlane=sin(t);
    [xEndPlane,yEndPlane]=meshgrid(xEndPlane,yEndPlane);    
    xEndPlane(xEndPlane.^2+yEndPlane.^2>1)=NaN;
    yEndPlane(xEndPlane.^2+yEndPlane.^2>1)=NaN;
    zUp=CorePoint(3)+Height/2+0*xEndPlane;
    zDown=CorePoint(3)-Height/2+0*xEndPlane;
    
    %然后在进行放大和平移
    xEndPlane=Radius*xEndPlane+CorePoint(1);
    yEndPlane=Radius*yEndPlane+CorePoint(2);
    
    %最后进行旋转
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
    
    %绘制两个底面
    p2=mesh(xEndPlane_Up,yEndPlane_Up,zEndPlane_Up);
    p3=mesh(xEndPlane_Down,yEndPlane_Down,zEndPlane_Down);
end

%% 设置颜色
set(p1,'FaceColor','white','EdgeColor',Color);
set(p2,'FaceColor','white','EdgeColor',Color);
set(p3,'FaceColor','white','EdgeColor',Color);

end





