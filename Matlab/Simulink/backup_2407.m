function [D_bot, D_top, debug_obs, debug_deriv, debug_ctrl] = fcn(Vin_m, Vout_m, IL_m, converter_mode, Vout_ref, Ts)
%#codegen
    persistent cont w ...
        L C RL VD G_load Idc_load CP_load ...
        gamma_RL gamma_VD gamma_G_load gamma_Idc_load gamma_CP_load ...
        c_val c_der y_last D_bot_last D_top_last ...
        int_eVout initialized

    %% Fixed dimensions
    n_states = 5;

    %% Initialization
    if isempty(initialized)
        cont = 0;

        %% Filter Coefficients
        m = 14;
        w = 2*m + 1;
        d = 2;
        [c_val, c_der] = savgol_coeffs(m, d, m, Ts);

        y_last  = zeros(w, n_states);

        %% Parameters
        L_nom   = 80/1000;
        C_nom   = 12/1000;
        L = L_nom*ones(3, 1);
        RL = 10e-3*ones(3, 1);
        VD  = 0.8*ones(3, 1);
        C = C_nom;
        G_load = 0;
        Idc_load = 0;
        CP_load = 0;

        %% 
        gamma_RL = 0.01;
        gamma_VD = 0.05;
        gamma_G_load = 0.1; 
        gamma_Idc_load = 1;
        gamma_CP_load = 0.1;

        %% 
        D_bot_last = zeros(3, 1);
        D_top_last = zeros(3, 1);

        %% Integral term
        int_eVout = 0.0;

        initialized = true;
    end

    %% Measurement vector at current sample
    y_last = [y_last(2:end, :); IL_m' Vout_m Vin_m]; 

    %% Filtered values and derivatives
    y = c_val*y_last;
    y_dot = c_der*y_last;

    IL = y(1:3)';
    Vout = y(4);
    Vin = y(5);

    IL_dot = y_dot(1:3)';
    Vout_dot = y_dot(4);
    % Vin_dot = y_dot(n_phases + 2);

    %% Load current from model
    if converter_mode == 0
        I_load = sum(IL.*(1 - D_bot_last)) - C*Vout_dot;
    else
        I_load = sum(IL) - C*Vout_dot;
    end

    %% Power equations
    Pin = Vin*sum(IL); % Input power
    Pout = Vout*I_load; % Output power
    PR = sum(RL.*IL.^2); % Resistive losses
    if converter_mode == 0
        PD = sum(VD.*(1 - D_bot_last).*IL); % Diode losses
    else
        PD = sum(VD.*(1 - D_top_last).*IL); % Diode losses
    end
    PL = sum(L.*IL.*IL_dot); % Inductive loading
    PC = C*Vout*Vout_dot; % Capacitive loading

    %% MRAC to better estimate parameters
    res_P = Pin - Pout - PR - PD - PL - PC;
    if cont > w
        y_IL = Vin - Vout*(1 - D_bot_last);
        y_IL_hat = L.*IL_dot + RL.*IL + VD.*(1 - D_bot_last);
        res_IL = y_IL - y_IL_hat;
    
        y_I_load_hat = G_load*Vout + Idc_load + CP_load/(Vout + 0.001);
        res_I_load = I_load - y_I_load_hat;

        % Update
        w_P = 1/50;
        RL = RL + Ts*gamma_RL*(res_IL.*IL + w_P*res_P.*IL.^2);
        VD = VD + Ts*gamma_VD*(res_IL.*(1 - D_bot_last) + w_P*res_P.*(1 - D_bot_last).*IL);
        G_load = G_load + Ts*gamma_G_load*(res_I_load*Vout);
        Idc_load = Idc_load + Ts*gamma_Idc_load*(res_I_load);
        CP_load = CP_load + Ts*gamma_CP_load*(res_I_load/(Vout + 0.001));
    else
        res_IL = zeros(3, 1);
        res_I_load = 0;
    end

    %% Derivatives by model
    IL_dot_model = (Vin - RL.*IL - (Vout + VD).*(1 - D_bot_last))./L;
    Vout_dot_model = (sum(IL.*(1 - D_bot_last)) - G_load*Vout - Idc_load - CP_load/(Vout + 0.001))/C;

    %% Voltage error and integral
    e_Vout = Vout_ref - Vout;

    if abs(e_Vout) < 0.5*max(Vout_ref, 1.0)
        int_eVout = int_eVout + e_Vout * Ts;
    end

    %% Control laws
    if converter_mode == 0
        D_bot = 0.5*ones(3, 1) + 0.01*sin(2*6.28*cont*Ts);
        D_top = zeros(3, 1);
        cont = cont + 1;
    else
        D_bot = zeros(3, 1);
        D_top = 0.5*ones(3, 1);
    end
    u = [D_bot; D_top];
    D_bot_last = D_bot;
    D_top_last = D_top;

    %% Debug variables
    debug_obs = zeros(40, 1);
    debug_obs(1:5) = y;

    debug_obs(11:13) = IL;
    debug_obs(14) = Vout;
    debug_obs(15) = Vin;
    debug_obs(16) = I_load;
    debug_obs(17) = G_load;
    debug_obs(18) = Idc_load;
    debug_obs(19) = CP_load;
    debug_obs(20:22) = L;
    debug_obs(23) = C;
    debug_obs(24:26) = RL;
    debug_obs(27:29) = VD;
    debug_obs(30) = Pin;
    debug_obs(31) = Pout;
    debug_obs(32) = PR;
    debug_obs(33) = PD;
    debug_obs(34) = PL;
    debug_obs(35) = PC;
    debug_obs(36) = res_P;
    debug_obs(37:39) = res_IL;
    debug_obs(40) = res_I_load;

    debug_deriv = zeros(25, 1);
    debug_deriv(1:5) = y_dot;
    debug_deriv(6:8) = IL_dot_model;
    debug_deriv(9) = Vout_dot_model;

    debug_ctrl = zeros(25, 1);
    debug_ctrl(1:6) = u;
    debug_ctrl(7) = int_eVout;
end

function [c_val, c_der] = savgol_coeffs(m, d, x_eval, dx)
    if nargin < 3; dx = 1; end
    x = (-m:m).';
    if d >= length(x); error('Polynomial degree d must be smaller than window length.'); end
    if d < 1;          error('Polynomial degree d must be at least 1 for first derivative.'); end
    A = zeros(length(x), d+1);
    v_val = zeros(d+1,1);
    v_der = zeros(d+1,1);
    for j = 0:d
        A(:,j+1) = x.^j; 
        v_val(j+1) = x_eval^j;
        v_der(j+1) = j * x_eval^(j-1) / dx; 
    end
    B     = (A.'*A) \ A.';
    c_val = v_val.' * B;
    c_der = v_der.' * B;
end