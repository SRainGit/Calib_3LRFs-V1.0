function F=GetCalibrationMatrix(w,x,y,z)

R_q_IMU2LS=[
    1-2*(y^2+z^2)    2*(x*y-z*w)      2*(x*z+y*w);
    2*(x*y+z*w)      1-2*(x^2+z^2)    2*(y*z-x*w);
    2*(x*z-y*w)      2*(y*z+x*w)      1-2*(x^2+y^2)];

% %R_q_LS2IMU与R_q_IMU2LS对应的q互为共轭四元数
% R_q_LS2IMU=[
%     1-2*(y^2+z^2)    2*(x*y+z*w)      2*(x*z-y*w);
%     2*(x*y-z*w)      1-2*(x^2+z^2)    2*(y*z+x*w);
%     2*(x*z+y*w)      2*(y*z-x*w)      1-2*(x^2+y*2)];


F=[
    deltaR_LS12*R_q_IMU2LS - R_q_IMU2LS*deltaR_IMU12;
    deltaR_LS13*R_q_IMU2LS - R_q_IMU2LS*deltaR_IMU13;
    deltaR_LS14*R_q_IMU2LS - R_q_IMU2LS*deltaR_IMU14;
    deltaR_LS23*R_q_IMU2LS - R_q_IMU2LS*deltaR_IMU23;
    deltaR_LS24*R_q_IMU2LS - R_q_IMU2LS*deltaR_IMU24;
    deltaR_LS34*R_q_IMU2LS - R_q_IMU2LS*deltaR_IMU34;
    ];

end