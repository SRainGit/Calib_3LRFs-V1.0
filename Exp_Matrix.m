
%% 
function [R,T]=Exp_Matrix(mu,pseudo_exponential)
T=zeros(3,1);

one_6th = 1.0/6.0;
one_20th = 1.0/20.0;

mu_xyz = mu(1:3);
w = mu(4:6);

theta=norm(w);
theta_sq=theta^2;

vCross=cross(w,mu_xyz);

if theta_sq < 1e-8
    A = 1.0 - one_6th * theta_sq;
    B = 0.5;
    
    if pseudo_exponential==0
        T(1) = mu_xyz(1) + 0.5 * vCross(1);
        T(2) = mu_xyz(2) + 0.5 * vCross(2);
        T(3) = mu_xyz(3) + 0.5 * vCross(3);
    end
else
    if theta_sq < 1e-6
        C = one_6th*(1.0 - one_20th * theta_sq);
        A = 1.0 - theta_sq * C;
        B = 0.5 - 0.25 * one_6th * theta_sq;
    else
        inv_theta = 1.0/theta;
        A = sin(theta) * inv_theta;
        B = (1 - cos(theta)) * (inv_theta * inv_theta);
        C = (1 - A) * (inv_theta * inv_theta);
    end
    
    w_cross=cross(w,vCross);	% = w^cross
    
    if pseudo_exponential==0
        % result.get_translation() = mu_xyz + B * cross + C * (w ^ cross);
        T(1) = mu_xyz(1) + B * vCross(1) + C * w_cross(1);
        T(2) = mu_xyz(2) + B * vCross(2) + C * w_cross(2);
        T(3) = mu_xyz(3) + B * vCross(3) + C * w_cross(3);
    end
end

% 3x3 rotation part:
R=Rodrigues_SO3_Exp(w, A, B);

if pseudo_exponential~=0
    T = mu_xyz;
    % else: has been already filled in above.
end

end






