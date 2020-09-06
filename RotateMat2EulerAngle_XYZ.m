%旋转矩阵到欧拉角——左乘（矩阵*待旋转目标）、右旋（坐标系顺时针旋转）
%按照依次旋转XYZ轴的顺序进行解算
function angles=RotateMat2EulerAngle_XYZ(R)
angles(1)=atan2(R(3,2),R(3,3))*180/pi;
angles(2)=atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2))*180/pi;
angles(3)=atan2(R(2,1),R(1,1))*180/pi;
end