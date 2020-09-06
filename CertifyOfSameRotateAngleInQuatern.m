% ��֤deltaR_LS��deltaR_IMU����Ӧ��qӵ�й�ͬ��theta
close all
clc
clear

% w x y z iΪIMU2LS����Ԫ����wIMU xIMU yIMU zIMUΪdeltaIMU����Ԫ��
syms w x y z  wIMU xIMU yIMU zIMU real


%% ��Ԫ�������q_deltaLS������֤��ת�Ƕ��Ƿ�һ��
%֤��q_deltaLS�͸���q_deltaLS = q_IMU2LS x deltaR_IMU x q_LS2IMU...
%�õ���q_deltaLS����Ӧ����ת����ͬ�����õ�ͨ����Ԫ������õ���deltaLS����ת��
q_IMU2LS=[w x y z];
q_deltaIMU=[wIMU xIMU yIMU zIMU];
q_LS2IMU=[w -x -y -z];

q1=QuanternMultiply(q_deltaIMU(1),q_deltaIMU(2),q_deltaIMU(3),q_deltaIMU(4),...
    q_LS2IMU(1),q_LS2IMU(2),q_LS2IMU(3),q_LS2IMU(4))
q_deltaLS=QuanternMultiply(q_IMU2LS(1),q_IMU2LS(2),q_IMU2LS(3),q_IMU2LS(4),...
    q1(1),q1(2),q1(3),q1(4))
w_deltaLS_FromQuat=simplify(q_deltaLS(1))
axis_deltaLS_FromQuat=simplify(q_deltaLS(2:4)')


%% ����ı�ʾ��ʽ�����R_deltaLS
R_q_IMU2LS=[
    1-2*(y^2+z^2)    2*(x*y-z*w)      2*(x*z+y*w);
    2*(x*y+z*w)      1-2*(x^2+z^2)    2*(y*z-x*w);
    2*(x*z-y*w)      2*(y*z+x*w)      1-2*(x^2+y^2)];
%R_q_LS2IMU��R_q_IMU2LS��Ӧ��q��Ϊ������Ԫ��
R_q_LS2IMU=[
    1-2*(y^2+z^2)    2*(x*y+z*w)      2*(x*z-y*w);
    2*(x*y-z*w)      1-2*(x^2+z^2)    2*(y*z+x*w);
    2*(x*z+y*w)      2*(y*z-x*w)      1-2*(x^2+y^2)];

R_q_deltaIMU=[
    1-2*(yIMU^2+zIMU^2)    2*(xIMU*yIMU-zIMU*wIMU)      2*(xIMU*zIMU+yIMU*wIMU);
    2*(xIMU*yIMU+zIMU*wIMU)      1-2*(xIMU^2+zIMU^2)    2*(yIMU*zIMU-xIMU*wIMU);
    2*(xIMU*zIMU-yIMU*wIMU)      2*(yIMU*zIMU+xIMU*wIMU)      1-2*(xIMU^2+yIMU^2)];

R_deltaLS=simplify(R_q_IMU2LS*R_q_deltaIMU*R_q_LS2IMU)

%% ��֤��ת���Ƿ�һ��
%ʹ����Ԫ����ʾ��R_q_IMU2LS����deltaIMU����Ӧ����ת�������ת...
%�����Ƿ�����Ԫ�������õ���deltaLS����Ӧ����ת����ͬ
axis_deltaIMU=[xIMU yIMU zIMU];
axis_deltaLS_FromRotate=R_q_IMU2LS*axis_deltaIMU';
axis_deltaLS_FromRotate=simplify(axis_deltaLS_FromRotate)

axis_error=axis_deltaLS_FromQuat-axis_deltaLS_FromRotate
axis_error=simplify(axis_error)










