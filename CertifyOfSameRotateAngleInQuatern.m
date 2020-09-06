% 验证deltaR_LS与deltaR_IMU所对应的q拥有共同的theta
close all
clc
clear

% w x y z i为IMU2LS的四元数，wIMU xIMU yIMU zIMU为deltaIMU的四元数
syms w x y z  wIMU xIMU yIMU zIMU real


%% 四元数计算出q_deltaLS，并验证旋转角度是否一致
%证明q_deltaLS和根据q_deltaLS = q_IMU2LS x deltaR_IMU x q_LS2IMU...
%得到的q_deltaLS所对应的旋转角相同，并得到通过四元数计算得到的deltaLS的旋转轴
q_IMU2LS=[w x y z];
q_deltaIMU=[wIMU xIMU yIMU zIMU];
q_LS2IMU=[w -x -y -z];

q1=QuanternMultiply(q_deltaIMU(1),q_deltaIMU(2),q_deltaIMU(3),q_deltaIMU(4),...
    q_LS2IMU(1),q_LS2IMU(2),q_LS2IMU(3),q_LS2IMU(4))
q_deltaLS=QuanternMultiply(q_IMU2LS(1),q_IMU2LS(2),q_IMU2LS(3),q_IMU2LS(4),...
    q1(1),q1(2),q1(3),q1(4))
w_deltaLS_FromQuat=simplify(q_deltaLS(1))
axis_deltaLS_FromQuat=simplify(q_deltaLS(2:4)')


%% 矩阵的表示形式计算出R_deltaLS
R_q_IMU2LS=[
    1-2*(y^2+z^2)    2*(x*y-z*w)      2*(x*z+y*w);
    2*(x*y+z*w)      1-2*(x^2+z^2)    2*(y*z-x*w);
    2*(x*z-y*w)      2*(y*z+x*w)      1-2*(x^2+y^2)];
%R_q_LS2IMU与R_q_IMU2LS对应的q互为共轭四元数
R_q_LS2IMU=[
    1-2*(y^2+z^2)    2*(x*y+z*w)      2*(x*z-y*w);
    2*(x*y-z*w)      1-2*(x^2+z^2)    2*(y*z+x*w);
    2*(x*z+y*w)      2*(y*z-x*w)      1-2*(x^2+y^2)];

R_q_deltaIMU=[
    1-2*(yIMU^2+zIMU^2)    2*(xIMU*yIMU-zIMU*wIMU)      2*(xIMU*zIMU+yIMU*wIMU);
    2*(xIMU*yIMU+zIMU*wIMU)      1-2*(xIMU^2+zIMU^2)    2*(yIMU*zIMU-xIMU*wIMU);
    2*(xIMU*zIMU-yIMU*wIMU)      2*(yIMU*zIMU+xIMU*wIMU)      1-2*(xIMU^2+yIMU^2)];

R_deltaLS=simplify(R_q_IMU2LS*R_q_deltaIMU*R_q_LS2IMU)

%% 验证旋转轴是否一致
%使用四元数表示的R_q_IMU2LS来对deltaIMU所对应的旋转轴进行旋转...
%检验是否与四元数法所得到的deltaLS所对应的旋转轴相同
axis_deltaIMU=[xIMU yIMU zIMU];
axis_deltaLS_FromRotate=R_q_IMU2LS*axis_deltaIMU';
axis_deltaLS_FromRotate=simplify(axis_deltaLS_FromRotate)

axis_error=axis_deltaLS_FromQuat-axis_deltaLS_FromRotate
axis_error=simplify(axis_error)










