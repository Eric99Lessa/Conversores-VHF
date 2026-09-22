function resultsTable = eigenvalueMetrics(eigVals)
% eigenvalueMetrics Compute modal metrics from eigenvalues.
%
%   resultsTable = eigenvalueMetrics(eigVals)
%
%   Given an array of continuous-time eigenvalues/poles, this function
%   computes:
%
%       - natural frequency
%       - damping ratio
%       - damping coefficient / decay rate
%       - damped natural frequency
%       - settling time
%       - percent overshoot
%
%   Input:
%       eigVals : array of eigenvalues, real or complex
%
%   Output:
%       resultsTable : MATLAB table containing the computed values
%
%   Assumption:
%       Eigenvalues are continuous-time poles:
%
%           lambda = real(lambda) + j*imag(lambda)
%
%       For a second-order mode:
%
%           wn    = abs(lambda)
%           zeta  = -real(lambda)/wn
%           sigma = -real(lambda)
%           wd    = abs(imag(lambda))
%
%       Settling time uses the 2 percent criterion:
%
%           Ts ≈ 4/(zeta*wn) = 4/(-real(lambda))
%
%       Percent overshoot is:
%
%           OS = 100*exp(-zeta*pi/sqrt(1 - zeta^2))
%
%       for 0 < zeta < 1.

    % Force eigenvalues into a column vector
    eigVals = eigVals(:);

    % Number of eigenvalues
    n = length(eigVals);

    % Preallocate arrays

    wn = zeros(n,1);        % Natural frequency
    zeta = zeros(n,1);      % Damping ratio
    sigma = zeros(n,1);     % Damping coefficient / decay rate
    wd = zeros(n,1);        % Damped natural frequency
    Ts = zeros(n,1);        % Settling time
    OS = zeros(n,1);        % Percent overshoot

    for k = 1:n

        lambda = eigVals(k);

        sigma(k) = -real(lambda);
        wd(k) = abs(imag(lambda));
        wn(k) = abs(lambda);

        if wn(k) ~= 0
            zeta(k) = sigma(k)/wn(k);
        else
            zeta(k) = NaN;
        end

        % Settling time, 2 percent criterion
        if sigma(k) > 0
            Ts(k) = 4/sigma(k);
        elseif sigma(k) == 0
            Ts(k) = Inf; % marginally stable
        else
            Ts(k) = Inf; % unstable
        end

        % Percent overshoot
        if zeta(k) > 0 && zeta(k) < 1
            OS(k) = 100*exp((-zeta(k)*pi)/sqrt(1 - zeta(k)^2));
        elseif zeta(k) >= 1
            OS(k) = 0; % critically damped or overdamped
        else
            OS(k) = NaN; % unstable or invalid
        end

    end

    % Store results in a table
    resultsTable = table( ...
        eigVals, ...
        wn, ...
        zeta, ...
        sigma, ...
        wd, ...
        Ts, ...
        OS, ...
        'VariableNames', { ...
            'Eigenvalue', ...
            'Wn', ...
            'Zeta', ...
            'Sigma', ...
            'Wd', ...
            'SettlingTime_Ts', ...
            'Overshoot_percent' ...
        });
end