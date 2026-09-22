function y = measurementModel(x, n_phases)

    % State vector:
    % x = [IL_1; IL_2; ...; IL_n; Vout; Iload; Vin]
    %
    % Measurement vector:
    % y = [IL_1; IL_2; ...; IL_n; Vout; Vin]
    %
    % Iload is not directly measured.

    IL   = x(1:n_phases);
    Vout = x(n_phases + 1);
    Vin  = x(n_phases + 3);

    y = [
        IL;
        Vout;
        Vin
    ];
end