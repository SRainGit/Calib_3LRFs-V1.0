%旋转矩阵到欧拉角——左乘（矩阵*待旋转目标）、右旋（坐标系顺时针旋转）
%按照依次旋转ZYX轴的顺序进行解算
function angles=RotateMat2EulerAngle_ZYX(R)
angles(1)=atan2(-R(2,3),R(3,3));
angles(2)=asin(R(1,3));
angles(3)=atan2(-R(1,2),R(1,1));
end