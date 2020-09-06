% The addition principle for translation and rotation matrix
% RT: X_new=R*X+T
% return ([RA,TA]+[RB,TB])
function [RC,TC]=PoseAddition_RT(RA,TA,RB,TB)

TC = TB + RB * TA;
RC = RB * RA;

end