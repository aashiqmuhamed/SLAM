function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)


% TODO: Compute the 3x3 Jacobian Gx of the motion model


% TODO: Construct the full Jacobian G


% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

N = (size(mu,1)-3)/2;
Fx = zeros(3,2*N+3);
Fx(1:3,1:3) = eye(3,3);

x = mu(1:3,1);
delx = [u.t*cos(x(3)+u.r1); u.t*sin(x(3)+u.r1); u.r1+u.r2];


mu = mu+Fx'*delx;
mu(3,1) = normalize_angle(mu(3,1));

G = [0 0 -u.t* sin(x(3)+u.r1); 0 0 u.t*cos(x(3)+u.r1);0 0 0];
Gt = eye(2*N+3,2*N+3) + Fx'*G*Fx;

sigma = Gt*sigma*Gt' + R


% TODO: Compute the predicted sigma after incorporating the motion


end
