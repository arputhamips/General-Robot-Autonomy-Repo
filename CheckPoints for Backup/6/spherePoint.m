function [phi] = spherePoint(map, goal, k, lambda, q)
% spherePoint  Navigation–function value at a configuration q in a sphere world
%
%   INPUTS
%     map      : m×3 array. Row‑1 is the workspace boundary sphere
%                [xc  yc  R]  (centre & radius).  Rows 2…m are obstacle
%                spheres [xi  yi  ri].
%     goal     : 1×2 array  [xg  yg]  (goal position).
%     k        : positive integer – tuning exponent (k ≥ 1, typically 2–5).
%     lambda   : positive scalar – blending coefficient (λ > 0).
%     q        : 1×2 array  [x  y]  (query position).
%
%   OUTPUT
%     phi      : navigation–function value Φ(q)  (0 ≤ Φ ≤ 1   in free space).
%                If q lies inside an obstacle or outside the workspace
%                boundary, phi = NaN.
%
%       α(q)  = ‖q − goal‖²                                   (attractive term)
%       β0(q) = Rw² − ‖q − cw‖²                              (boundary term)
%       βi(q) = ‖q − ci‖² − Ri² ,  i = 1…(m−1)               (obstacle terms)
%       β(q)  = β0(q) · Π βi(q)
%       Φ(q)  = α(q) / ( α(q)^k + λ β(q) )^(1/k)
%
%   The function returns a scalar in [0,1] for all free configurations.
%   Φ(q) → 0 at the goal, Φ(q) → 1 on the boundary / obstacle surfaces.
%

% Autonomous Mobile Robots - HW6
% NIRMAL A J L A

% ---------------------------------------------------------
%% Extract workspace and obstacles
cw   = map(1,1:2);      % workspace centre
Rw   = map(1,3);        % workspace radius
obsC = map(2:end,1:2);  % obstacle centres
obsR = map(2:end,3);    % obstacle radii

%% Distances
alpha = sum( (q - goal).^2 );                 % ‖q - qg‖²

beta0 = Rw^2 - sum( (q - cw).^2 );            % workspace term
beta  = beta0;                                % initialise product

for j = 1:size(obsC,1)
    betaj = sum( (q - obsC(j,:)).^2 ) - obsR(j)^2;
    beta  = beta * betaj;                     % accumulate Π βi
end

%% Check if q is in free space
if beta0 <= 0          % outside workspace
    phi = NaN;  return
end
for j = 1:size(obsC,1)
    if ( sum( (q - obsC(j,:)).^2 ) <= obsR(j)^2 )   % inside obstacle
        phi = NaN;  return
    end
end

%% Navigation‑function value
phi = alpha / ( alpha^k + lambda*beta )^(1/k);

end



