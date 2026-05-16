function [pp] = polyfit_esp32(x, y, x_break)
    x_r1 = x(x < x_break);
    x_r2 = x(x >= x_break);
    y_r1 = y(x < x_break);
    y_r2 = y(x >= x_break);

    p_r1 = polyfit(x_r1, y_r1, 1);

    if ~isempty(x_r2)
        b_r2 = polyval(p_r1, x_break);
        a_r2 = (x_r2 - x_break)\(y_r2 - b_r2);
        p_r2 = [a_r2 b_r2];
        pp = mkpp([0 x_break 4096], [p_r1; p_r2]);

        fprintf('%.15f, %.15f, %.15f, %.15f);\n', ...
            p_r1(1), p_r1(2), a_r2, b_r2 - a_r2*x_break);
    else
        pp = mkpp([0 4096], p_r1);
        fprintf('%.15f, %.15f, %.15f, %.15f);\n', ...
            p_r1(1), p_r1(2), p_r1(1), p_r1(2));
    end
    
end