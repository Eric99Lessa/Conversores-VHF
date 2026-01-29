clear; clc;

syms t T D real

% Fundamental frequency
omega0 = 2*pi/T;

n_phases = 3; % Number of phases
order_approx = 0; % Approximation order
n_states_eq = 1 + 2*order_approx; % Number of states per equation
n_eq = n_phases + 1; % 1 equation for capacitor and 1 for each inductor

%% PWM
% Define the piecewise function for the PWM signal u(t)
syms d [n_phases 1]
u = piecewise(0 <= t & t < D*T, 1, D*T <= t & t < T, 0);

% Initialize basis
basis = sym(ones(n_states_eq, 1));
u_approx = (1/T)*int(1, t, 0, D*T);

% Preallocate symbolic array for phase-shifted signals
syms un un_approx

% Generate phase-shifted PWM signals
for i = 1:n_phases
    t_delay = (i - 1)*T/n_phases;
    un(i) = subs(u, t, t - t_delay);
    un_approx(i) = subs(u_approx, t, t - t_delay);
end

% Plot the original PWM signals
plota = false; % Bool para plot
if plota
    plot_comp_approx(un, un_approx);
end

%% States variables
n_states = n_states_eq*n_eq; 
syms xn xn_dot [n_states 1]

for i = 1:n_states
    syms(sprintf('x%d(t)', i)) %declare each element in the array as a single symbolic function
    xn(i) = symfun(eval(sprintf('x%d(t)', i)), t); %declare each element to a symbolic "handle"
    xn_dot(i) = diff(xn(i), t);
end

syms In_L IC_u [n_phases 1]
syms LHS RHS [n_eq 1]
syms VC Vin R_ind L C I_load real
assumeAlso(R_ind > 0);
assumeAlso(L > 0);
assumeAlso(C > 0);

%% LHS
% O lado esquerdo das equações do modelo é composto pelas equações das
% correntes em cada indutor e tensão no capacitor de saída

for i = 1:n_eq
    if i <= n_phases
        In_L(i) = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
        LHS(i) = L*diff(In_L(i), t);
    else
        VC = sum(xn((1:n_states_eq) + n_states_eq*(i - 1)).*basis);
        LHS(i) = C*diff(VC, t);
    end
end

%% RHS
% O lado direito das equações inclui a influencia da tensão de entrada,
% resistencia e indutancia dos indutores em cada fase

for i = 1:n_phases
    % se u = 0
    VLi_u0 = Vin - R_ind*In_L(i) - VC;
    IC_u0 = In_L(i);

    % se u = 1
    VLi_u1 = Vin - R_ind*In_L(i);
    IC_u1 = 0;

    VLi_u = VLi_u0*(1 - d(i)) + VLi_u1*d(i);
    IC_u(i) = IC_u0*(1 - d(i)) + IC_u1*d(i);

    RHS(i) = VLi_u;
end
RHS(end) = sum(IC_u) - I_load;

%% Substitui funcoes do tempo
syms Xn Xn_dot [n_states 1] real

old_vars = [xn_dot; xn];
new_vars = [Xn_dot; Xn];

for i = 1:n_eq
    LHS(i) = subsExpression(LHS(i), old_vars, new_vars);
    LHS(i) = collect(LHS(i), basis);
    RHS(i) = subsExpression(RHS(i), old_vars, new_vars);
    RHS(i) = collect(RHS(i), basis);
end

eqs = LHS(:) == RHS(:);

% Solve for X_dot
[solution] = solve(eqs, Xn_dot, 'ReturnConditions', true);
solution = rmfield(solution, intersect(fieldnames(solution), {'parameters','conditions'}));

% Convert struct to symbolic vector
X_dot = struct2array(solution);
X_dot = X_dot(:);

%% Express system in port-Hamiltonian form: x_dot = (J - R)*dH/dx + g*u + xi
% Prepare states and input
X = Xn;
U = d;

% Define Hamiltonian and jacobian
Q = [sqrt(L)*eye(n_phases) zeros(n_phases, 1);
     zeros(1, n_phases) sqrt(C)];
H = 0.5*(Q*X).'*(Q*X);
jac_H = jacobian(H, X); jac_H = jac_H(:);

% Input Matrix
g_x = jacobian(X_dot, U);

X_dot_aux = simplify(X_dot - g_x*U);

% External
xi = subs(X_dot_aux, X, zeros(size(X)));

X_dot_aux = simplify(X_dot_aux - xi);

jac_X = jacobian(X_dot_aux, X);

for i = 1:size(jac_X, 2)
    jac_X(:, i) = jac_X(:, i)/(jac_H(i)/X(i));
end

R = -(jac_X + jac_X')/2; % R é estrita positiva
J = (jac_X - jac_X')/2;

% X_dot reescrito
% test = X_dot - ((J - R)*jac_H + g_x*U + xi);
% simplify(test)
X_dot = (J - R)*jac_H + g_x*U + xi;

%% Closed-loop system

syms Vout_ref IL_ref IL_ref_dot Vout_ref_dot real
syms err [(n_phases + 1) 1] real
syms Kr Kj real

% Define closed-loop Hamiltonian
Hd = 0.5*(Q*err).'*(Q*err);
jac_Hd = jacobian(Hd, err); jac_Hd = simplify(jac_Hd(:));
X_ref = [IL_ref*ones(n_phases, 1); Vout_ref];
X_ref_dot = [IL_ref_dot*ones(n_phases, 1); Vout_ref_dot];
jac_Hd = subs(jac_Hd, err, X - X_ref);

Jc = Kj*J;
Rc = Kr*R;
Jd = simplify(J + Jc); assumeAlso(Kj > 0);
Rd = simplify(R + Rc); assumeAlso(Kr > 0);
Jd = Kj*J; assumeAlso(Kj > 1);
Rd = Kr*R; assumeAlso(Kr > 1);

% Nota: jac_H pode ser escrito como diag([L c])*X, diag([L C]) = D_lc

D_lc = Q.^2;

%% Lei de Controle

% Agora precisamos achar a equação de D que garante as raízes nos lugares
% desejados

% Igualando as derivadas em malha aberta e fechada, isolando o duty cycle:
% g_x*U = (Jd - Rd)*jac_Hd + X_ref_dot - (J - R)*jac_H-xi
% (J - R)*(jac_Hd - jac_H) + (Jc - Rc)*jac_Hd + X_ref_dot - xi

% g_x*U = (J - R)*D_lc*X_ref + (Jc - Rc)*D_lc*(X - X_ref) + X_ref_dot - xi

% como g(x) não é quadrada, precisamos usar a pseudo inversa, que é:
% inv(g_x'*g_x)*g_x' e pode ser obtida pela função pinv

% Jd = subs(Jd, Kj, Kj_sol);
% Rd = Kr*R;

expr_ma = (J - R)*D_lc*X + g_x*d + xi;

expr_mf = (Jd - Rd)*D_lc*(X - X_ref) + X_ref_dot;
expr_mf = subs(expr_mf, Vout_ref_dot, 0);

g_perp = null(g_x.')';
g_perp*g_x

res = simplify(g_perp*(expr_ma - expr_mf), 1000)
res_reg = simplify(subs(res, [IL_ref_dot, Vout_ref_dot], [0,0]));

% Isolando g_x*d
LHS = g_x*d;
RHS = expr_mf - expr_ma + LHS;

pinv_g_x = simplify(collect(pinv(g_x)));
d_sol = pinv_g_x*RHS;

% d_sol = subs(d_sol, Kj, Kj_sol);

syms Vout real
syms IL [1 n_phases] real
d_sol = subs(d_sol, Xn, [IL Vout]');
[d_num, d_den] = numden(d_sol);
% Note d_den is equal for all 3 phases

d_num = simplify(collect(d_num, [C^2 L^2]));

matlabFunction(d_num, 'File', 'function_d_num');
matlabFunction(d_den(1), 'File', 'function_d_den');

%% Plota raizes reais em função do ganho Kr

A_mf = (Jd - Rd)*D_lc;

roots = eig(A_mf);
n_roots = length(roots);

% Vamos separar as raízes que são sempre reais das que podem ser complexas

isreal_eig = false(n_eq, 1);
for i = 1:n_eq
    isreal_eig(i) = isreal(roots(i));
end

real_roots = simplify(roots(isreal_eig));
conj_roots = simplify(roots(~isreal_eig));

%%
real_roots_K = simplify(subs(real_roots, [L C R_ind], [80e-3 12*1e3*1e-6 1e-0]));
conj_roots_K = simplify(subs(conj_roots, [L C R_ind], [80e-3 12*1e3*1e-6 1e-0]));

n_Kj = 10; n_Kr = 500;
Kj_range = linspace(1, 1.5, n_Kj);
Kr_range = linspace(1, 15, n_Kr);

real_roots_val = complex(zeros(length(real_roots_K), n_Kr, n_Kj));
conj_roots_val = complex(zeros(length(conj_roots_K), n_Kr, n_Kj));

for iter_Kr = 1:n_Kr
    for iter_Kj = 1:n_Kj
        real_roots_val(:, iter_Kr, iter_Kj) = double(subs(real_roots_K, ...
            [Kr Kj], [Kr_range(iter_Kr) Kj_range(iter_Kj)]));

        conj_roots_val(:, iter_Kr, iter_Kj) = double(subs(conj_roots_K, ...
            [Kr Kj], [Kr_range(iter_Kr) Kj_range(iter_Kj)]));
    end
end

% ======================================================================
% Plot in complex plane with clickable points showing Kr and Kj
% ======================================================================
figure; ax = axes; hold(ax, 'on'); grid(ax, 'on');
xlabel(ax, 'Real'); ylabel(ax, 'Imag'); title(ax, 'Root-locus-like plot (Kr,Kj grid)');

% We will store Kr/Kj in UserData for each plotted object so datatips can show it.

% ---- Plot trajectories for real roots ----
h = gobjects(0);
for r = 1:size(real_roots_val, 1)
    Z = squeeze(real_roots_val(r, :, :));   % size: n_Kr x n_Kj

    % Lines for varying Kr at fixed Kj (looks like locus families)
    for j = 1:n_Kj
        zline = Z(:, j);
        hh = plot(ax, real(zline), imag(zline), 'b-', 'LineWidth', 1.0, ...
            'PickableParts','all', 'HitTest','on');

        hh.UserData.type   = 'real';
        hh.UserData.rootNo = r;
        hh.UserData.mode   = 'Kr_sweep';
        hh.UserData.Kr     = Kr_range(:);
        hh.UserData.Kj     = Kj_range(j) * ones(n_Kr,1);

        h(end+1) = hh; %#ok<SAGROW>
    end

    % Optional: also plot varying Kj at fixed Kr (uncomment if you want mesh-like locus)
    % for i = 1:n_Kr
    %     zline = Z(i, :).';
    %     hh = plot(ax, real(zline), imag(zline), 'b:', 'LineWidth', 0.75, ...
    %         'PickableParts','all', 'HitTest','on');
    %     hh.UserData.type   = 'real';
    %     hh.UserData.rootNo = r;
    %     hh.UserData.mode   = 'Kj_sweep';
    %     hh.UserData.Kr     = Kr_range(i) * ones(n_Kj,1);
    %     hh.UserData.Kj     = Kj_range(:);
    %     h(end+1) = hh;
    % end
end

% ---- Plot trajectories for complex-conjugate roots ----
for r = 1:size(conj_roots_val, 1)
    Z = squeeze(conj_roots_val(r, :, :));   % n_Kr x n_Kj

    for j = 1:n_Kj
        zline = Z(:, j);
        hh = plot(ax, real(zline), imag(zline), 'r-', 'LineWidth', 1.0, ...
            'PickableParts','all', 'HitTest','on');

        hh.UserData.type   = 'complex';
        hh.UserData.rootNo = r;
        hh.UserData.mode   = 'Kr_sweep';
        hh.UserData.Kr     = Kr_range(:);
        hh.UserData.Kj     = Kj_range(j) * ones(n_Kr,1);

        h(end+1) = hh; %#ok<SAGROW>
    end
end

axis(ax, 'equal'); % often nicer for complex plane

% Enable data cursor + custom callback
dcm = datacursormode(gcf);
set(dcm, 'Enable', 'on', 'UpdateFcn', @localDatatip);

syms tau real
assumeAlso(tau > 0);

[Kr_sol, params, conds] = solve(tau == -1/real_roots(1), Kr, 'ReturnConditions', true);
conj_roots = simplify(subs(conj_roots, Kr, Kr_sol), 1000);

Kj_sol = solve(conj_roots(1) == sum(conj_roots)/2, Kj);
Kj_sol = simplify(Kj_sol);

matlabFunction(Kr_sol, 'File', 'function_Kr');
matlabFunction(Kj_sol, 'File', 'function_Kj');

%%


% ----------------------------------------------------------------------
% Datatip callback (shows Kr/Kj at nearest vertex of the clicked line)
% ----------------------------------------------------------------------
function txt = localDatatip(~, event_obj)
    target = event_obj.Target;
    pos    = event_obj.Position;  % [x y z] typically (z unused)

    txt = {sprintf('Re: %.6g', pos(1)), sprintf('Im: %.6g', pos(2))};

    if isprop(target, 'UserData') && isstruct(target.UserData) ...
            && isfield(target.UserData, 'Kr') && isfield(target.UserData, 'Kj')

        % Find nearest vertex on the clicked line to map to Kr/Kj
        xd = target.XData(:);
        yd = target.YData(:);
        [~, idx] = min((xd - pos(1)).^2 + (yd - pos(2)).^2);

        Kr = target.UserData.Kr(idx);
        Kj = target.UserData.Kj(idx);

        txt{end+1} = sprintf('Kr: %.6g', Kr);
        txt{end+1} = sprintf('Kj: %.6g', Kj);

        if isfield(target.UserData, 'type')
            txt{end+1} = sprintf('Type: %s', target.UserData.type);
        end
        if isfield(target.UserData, 'rootNo')
            txt{end+1} = sprintf('Root #: %d', target.UserData.rootNo);
        end
        if isfield(target.UserData, 'mode')
            txt{end+1} = sprintf('Sweep: %s', target.UserData.mode);
        end
    end
end

% ======================================================================
% Constant damping ratio lines (zeta lines): add to the same axes "ax"
% ======================================================================

% Choose which zeta values you want
zeta_list = [0.1 0.2 0.3 0.4 0.5 0.7 0.9];   % edit as desired

% Decide how far to draw them based on current/expected plot extent.
% If you call this AFTER plotting roots, it will auto-scale nicely.
xl = xlim(ax); yl = ylim(ax);

% Ensure we include some left-half plane span even if current xlim isn't set yet
x_left = min([xl(1), -1]);     % leftmost x to draw to
x0     = 0;                    % start at origin

xline = linspace(x0, x_left, 300);  % goes from 0 to negative values

for zeta = zeta_list
    if zeta <= 0 || zeta >= 1
        continue; % zeta lines are typically 0<zeta<1
    end

    m = sqrt(1 - zeta^2) / zeta;  % slope factor: |w| = m * (-sigma)

    yline =  m * (-xline);        % upper line
    yline2 = -m * (-xline);       % lower line

    % Plot lines
    plot(ax, xline, yline,  'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
    plot(ax, xline, yline2, 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');

    % Label near the upper branch (pick a point not too close to origin)
    label_x = x_left * 0.7;               % negative
    label_y = m * (-label_x);             % positive
    text(ax, label_x, label_y, sprintf('\\zeta=%.1f', zeta), ...
        'Color','k', 'FontSize', 9, 'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom', 'Clipping','on');
end

% Optional: also draw the imaginary axis and real axis lightly
plot(ax, [0 0], ylim(ax), 'k:', 'HandleVisibility','off');
plot(ax, xlim(ax), [0 0], 'k:', 'HandleVisibility','off');

% Restore limits (plot/text can sometimes expand limits depending on settings)
xlim(ax, xl); ylim(ax, yl);