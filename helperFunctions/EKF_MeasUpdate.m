function [X, P, dX] = EKF_MeasUpdate(Xin, Pin, innov, H, R)
%{
    Performs EKF measurement update equations and returns correct state and
    covariance
%} 

% --- Kalman gain
K = Pin*H.' * inv( H*Pin*H.' + R);

% --- update states
dX = K * innov;
X = Xin + dX;

% --- update covariance estim
P = (eye(length(Xin)) - K*H) * Pin;

end

