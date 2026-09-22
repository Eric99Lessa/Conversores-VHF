function x_next = boostConverterModel(x, d, L, C, RL, VD, Ts, n_phases)

    % State vector:
    % x = [IL_1; IL_2; ...; IL_n; Vout; Vin; Iload]

    IL    = x(1:n_phases);
    Vout  = x(n_phases + 1);
    Vin   = x(n_phases + 2);
    Iload = x(n_phases + 3);

    IL_dot = zeros(n_phases, 1);

    %% Inductor current dynamics
    for i = 1:n_phases
        IL_dot(i) = (Vin - RL*IL(i) - (VD + Vout)*(1 - d(i))) / L;
    end

    %% Diode/output current contribution
    Iout_from_inductors = sum((1 - d(:)).*IL);

    %% Capacitor voltage dynamics
    Vout_dot = (Iout_from_inductors - Iload) / C;

    %% Unknown input voltage model
    % Vin is filtered as a slowly varying state.
    % Process noise q_Vin in Q allows it to adapt to Vin_m.
    Vin_dot = 0;

    %% Unknown load current model
    % Iload is the last state and is modeled as slowly varying.
    % Process noise q_Iload in Q allows it to adapt.
    Iload_dot = 0;

    %% Euler integration
    IL_next    = IL    + Ts*IL_dot;
    Vout_next  = Vout  + Ts*Vout_dot;
    Vin_next   = Vin   + Ts*Vin_dot;
    Iload_next = Iload + Ts*Iload_dot;

    x_next = [
        IL_next;
        Vout_next;
        Vin_next;
        Iload_next
    ];
end