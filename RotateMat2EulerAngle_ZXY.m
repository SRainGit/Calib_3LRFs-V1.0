%旋转矩阵到欧拉角——左乘（矩阵*待旋转目标）、右旋（坐标系顺时针旋转）
%按照依次旋转ZXY轴的顺序进行解算
function [a,b,c]=RotateMat2EulerAngle_ZXY(R)
a=-asin(R(2,3))*180/pi;
b=atan2(-R(1,3),R(3,3))*180/pi;
c=atan2(R(2,1),R(2,2))*180/pi;
end