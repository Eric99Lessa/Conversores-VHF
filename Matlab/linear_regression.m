function [a, y_fit, R_sqr] = linear_regression(x, y)
    a = x\y; % Coeficiente da reta
    y_fit = a*x;
    
    SStot = sum((y - mean(y)).^2);      % Total Sum-Of-Squares
    SSres = sum((y - y_fit).^2);        % Residual Sum-Of-Squares
    R_sqr = 1 - SSres/SStot; 
end