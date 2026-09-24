% shift_PI(): avança a simulação em MALHA FECHADA do conversor half-bridge
% por UM passo de amostragem T, executando as DUAS malhas PI em cascata
% (tensão externa -> corrente interna) já na forma DISCRETA (recursiva),
% e depois integra o modelo de planta (contínuo, via CasADi) por Euler.
%
% É chamada em laço por T3_PI_SC.m, uma vez por passo de tempo, e é onde a
% IMPLEMENTAÇÃO DIGITAL de fato acontece (T3_PI_SC.m só faz o PROJETO dos
% ganhos e a discretização; esta função é o "firmware simulado" que
% qualquer implementação em C/embarcado deveria replicar).
%
% Forma discreta do PI (equação a diferenças, discretização por Tustin/
% trapézio -- ver comentário em T3_PI_SC.m sobre GainA/GainB):
%     u[k] = u[k-1] + GainA*e[k] + GainB*e[k-1]
% Isso é EXATAMENTE o padrão "u_novo = u_antigo + A*erro_atual + B*erro_
% anterior" repetido abaixo para a malha de tensão (con) e para a malha de
% corrente (uPI). Se o Eric for implementar um PI clássico em C (por
% exemplo, para substituir ou complementar duty_cascade() no firmware
% Conversores-VHF), esta é a forma pronta para copiar: guardar e[k-1] (ou
% e_i[k-1]) e a saída anterior a cada ciclo, e aplicar a mesma equação.
%
% Entradas principais:
%   T          - período de amostragem [s] (mesmo T do laço principal)
%   x0         - estado atual da planta [i_L; v_C]
%   con0       - saída anterior do PI de TENSÃO (referência de corrente, "con")
%   uPI_ant    - saída anterior do PI de CORRENTE (contagens PWM, "uPI")
%   e_ant, e_i_ant - erro anterior de tensão e de corrente (memória p/ Tustin)
%   ref        - referência de tensão do capacitor (v_C_ref)
%   i_HESS     - perturbação de corrente de carga somada à referência de
%                corrente (simula um degrau de carga na malha de corrente)
%   Gains      - [GainA_PI_i, GainB_PI_i, GainA_PI_v, GainB_PI_v] (T3_PI_SC.m)
%   K_PWM      - resolução do registrador de comparação PWM (contagens)
%   f          - função CasADi do modelo contínuo da planta (dx/dt = f(x,u,dist))
%
% Saídas: t0 (tempo atualizado), x0 (novo estado), con/uPI (saídas dos dois
% PIs, para o próximo passo), e/e_i (erros deste passo, viram e_ant/e_i_ant
% no próximo passo).

function [t0, x0, con, uPI, e, e_i] = shift_PI(T, t0, x0, con0, uPI_ant, d, f, e_ant, e_i_ant, ref, j, i_HESS, Gains, K_PWM)

% Ki_vC = 0.030; initial gains
% Kp_vC = 0.5441;
% Kp_iL = 80.03;
% Ki_iL = 1.50e3;

st = x0;
% con0 = u(1,:)';
time = j*T;

% GainA_PI_i = (2*Kp_iL + T*Ki_iL) * 0.5;
GainA_PI_i = Gains(1);
% GainB_PI_i = (T*Ki_iL - 2*Kp_iL) * 0.5;
GainB_PI_i = Gains(2);
% GainA_PI_v = (2*Kp_vC + T*Ki_vC) * 0.5;
GainA_PI_v = Gains(3);
% GainB_PI_v = (T*Ki_vC - 2*Kp_vC) * 0.5;
GainB_PI_v = Gains(4);

%================ PI de TENSÃO (malha externa) ===========
% Erro de tensão: referência menos tensão medida do capacitor (v_C = x0(2)).
e = (ref-x0(2,1));
if time > 10
    % [REVISAR] após t=10s o erro é forçado a zero e o PI de tensão é
    % desativado/resetado (ver "con=0" abaixo) -- isto é específico deste
    % CENÁRIO DE TESTE (provavelmente para isolar a resposta da malha de
    % corrente a um degrau de carga aplicado depois de t=10s, ver
    % i_HESS em T3_PI_SC.m), não é um comportamento geral do controlador.
    e = 0; % anula pi de tensão
end
% Equação a diferenças do PI de tensão (Tustin): con[k] = con[k-1] + GainA*e[k] + GainB*e[k-1]
con = GainA_PI_v*e + GainB_PI_v*e_ant + con0;
if con>40
    con=40;                  % saturação da REFERÊNCIA DE CORRENTE gerada pelo PI de tensão (limite físico de i_L)
end
if con<-40
    con=-40;
end
if time > 10
    con = 0; % reseta integrador/saída do PI de tensão (ver nota acima sobre o cenário de teste)
end

% if (time>20 && time<20.05)
%     id = -20;
% else
%     id = 0;
% end

%================ PI de CORRENTE (malha interna) ===========
% Erro de corrente: referência (saída do PI de tensão + perturbação i_HESS) menos corrente medida (i_L = x0(1)).
e_i = (con+i_HESS-x0(1,1));
% Equação a diferenças do PI de corrente (Tustin): uPI[k] = uPI[k-1] + GainA*e_i[k] + GainB*e_i[k-1]
uPI = GainA_PI_i*e_i + GainB_PI_i*e_i_ant + uPI_ant;
% =============================
if uPI>K_PWM
    uPI=K_PWM;              % satura a SAÍDA DO PI (em contagens PWM) no limite superior do registrador de comparação
end
if uPI<0
    uPI=0;                  % satura no limite inferior (sem anti-windup explícito além deste clamp de saída)
end
dis = d(1,:)';               % vetor de distúrbio (não usado neste cenário: d é passado como escalar 0 por T3_PI_SC.m)

uPI_PWM = uPI/K_PWM;          % converte de contagens PWM para duty fracionário [0,1] antes de entrar no modelo de planta

% Integra o modelo CONTÍNUO da planta (função CasADi "f", definida em
% T3_PI_SC.m a partir de A_d/B_d/D_d) por UM passo de Euler explícito:
%   x[k+1] = x[k] + T * f(x[k], duty[k], distúrbio[k])
f_value = f(st,uPI_PWM,dis);
st = st+ (T*f_value);
x0 = full(st);                % converte de tipo simbólico CasADi (SX/DM) para double do MATLAB

t0 = t0 + T;
% u0 = [u(2:size(u,1),:);u(size(u,1),:)];

end
