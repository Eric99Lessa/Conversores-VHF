function [X_dot] = interleavedConverterModel(n_phases, order_approx, converter_str, switch_mode)
    arguments
        n_phases, order_approx,
        converter_str = "", % 
        switch_mode = 0,
    end

    n_states_eq = 1 + 2*order_approx; % Number of states per equation
    n_eq = n_phases + 1; % 1 equation for capacitor and 1 for each inductor

    %% States variables
    n_states = n_states_eq*n_eq;
    syms xn xn_dot [n_states 1]
    
    [basis, un_approx] = approxPWM(n_phases, order_approx);

    for i = 1:n_states
        syms(sprintf('x%d(t)', i)) %declare each element in the array as a single symbolic function
        xn(i) = symfun(eval(sprintf('x%d(t)', i)), t); %declare each element to a symbolic "handle"
        xn_dot(i) = diff(xn(i), t);
    end
    
    syms IL IC_i d [n_phases 1]
    syms LHS RHS [n_eq 1]
    syms Vout Vin R_L R_C L C VD I_load real
    
    %% LHS
    % O lado esquerdo das equações do modelo é composto pelas equações das
    % correntes em cada indutor e tensão no capacitor de saída
    
    for i = 1:n_eq
        if i <= n_phases
            IL(i) = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
            % IL(i) = xn(i);
            LHS(i) = L*diff(IL(i), t);
        end
        if i == n_phases + 1
            Vout = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
            % Vout = xn(n_phases + 1);
            LHS(i) = C*diff(Vout, t);
        end
    end

    %% RHS
    % O lado direito das equações inclui a influencia da tensão de entrada,
    % resistencia e indutancia dos indutores em cada fase
    
    dictionaries
    
    if isKey(map_conv, converter_str)
        converter = map_conv(converter_str);
        for i = 1:n_phases
            if converter == 0 % Boost
                % se transistor aberto
                VLi_open = Vin - R_L*IL(i) - VD - Vout;
                IC_open = IL(i);
                % se transistor fechado
                VLi_closed = Vin - R_L*IL(i);
                IC_closed = 0;
            elseif converter == 1 % Buck
                % se transistor aberto
                VLi_open = -VD - R_L*IL(i) - Vout;
                IC_open = IL(i);
                % se transistor fechado
                VLi_closed = Vin - R_L*IL(i) - Vout;
                IC_closed = IL(i);
            elseif converter == 2 % Buck-Boost
                % se transistor aberto
                VLi_open = Vout - R_L*IL(i) - VD;
                IC_open = IL(i);
                % se transistor fechado
                VLi_closed = Vin - R_L*IL(i);
                IC_closed = 0;
            end
    
            if switch_mode == 0 %% NO
                VLi_u = VLi_open*(1 - d(i)) + VLi_closed*d(i);
                IC_i(i) = IC_open*(1 - d(i)) + IC_closed*d(i);
            else %% NC
                VLi_u = VLi_open*d(i) + VLi_closed*(1 - d(i));
                IC_i(i) = IC_open*d(i) + IC_closed*(1 - d(i));
            end
    
            RHS(i) = VLi_u;
        end
    else
        error("Converter not defined");
    end
    RHS(n_phases + 1) = sum(IC_i) - I_load - Vout/R_C;

    %% Substitui funcoes do tempo
    syms X X_dot [n_states 1]
    
    old_vars = [xn_dot; xn];
    new_vars = [X_dot; X];

    for i = 1:n_eq
        LHS(i) = subsExpression(LHS(i), old_vars, new_vars);
        LHS(i) = subsExpression(LHS(i), d, un_approx);
        LHS(i) = collect(LHS(i), basis);
        RHS(i) = subsExpression(RHS(i), old_vars, new_vars);
        RHS(i) = subsExpression(RHS(i), d, un_approx);
        RHS(i) = collect(RHS(i), basis);
    end

    eqs = approxSystemFourier(order_approx, LHS, RHS, basis);
    eqs = subs(eqs, R_C, inf);
    
    %% Solve for X_dot
    solution = solve(eqs, X_dot);
    
    % Convert struct to symbolic vector
    X_dot = struct2array(solution);
    X_dot = X_dot(:);
end