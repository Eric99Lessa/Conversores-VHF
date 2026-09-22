function tf = hasFractionalPower(expr, pwr)
    pows = findSymType(expr, 'power');
    tf = false;

    for k = 1:numel(pows)
        parts = children(pows(k));
        exponent = parts{2};

        if isequal(exponent, pwr)
            tf = true;
            return
        end
    end
end