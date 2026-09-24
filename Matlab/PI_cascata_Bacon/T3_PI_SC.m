% T3_PI_SC.m  (ANOTADO)
% =============================================================================
% Script PRINCIPAL: projeta o controlador PI cascata clássico (C-PI) para o
% conversor half-bridge CC/CC bidirecional do supercapacitor (mesma planta
% do artigo CBA2026, Tabela "Plant and General Parameters"), DISCRETIZA os
% dois PIs (Tustin/trapézio) e SIMULA a malha fechada em tempo discreto
% para validar o projeto antes de qualquer implementação em hardware.
%
% Fluxo (ver README.md nesta pasta para o passo a passo de uso):
%   1) Define os parâmetros físicos da planta (L, r_L, C, V_dc, K_PWM).
%   2) Monta o modelo linear contínuo em espaço de estados (A_d, B_d, D_d)
%      e o empacota numa função CasADi "f" (usada só como um wrapper
%      simbólico conveniente; o modelo em si é linear, não precisa de
%      CasADi para ser avaliado -- é usado por consistência com os outros
%      scripts do projeto que usam CasADi para o MPC).
%   3) Chama PI_design.m DUAS vezes (malha de tensão com a planta Gpv.m,
%      malha de corrente com a planta Gpi.m) para obter os 4 ganhos
%      contínuos: Kp_vC, Ki_vC, Kp_iL, Ki_iL.
%   4) DISCRETIZA os dois PIs por Tustin/trapézio, guardando os 4
%      coeficientes resultantes em Gains(1..4) (ver dedução abaixo).
%   5) Simula a malha fechada em tempo discreto chamando shift_PI.m em
%      laço, um passo T por iteração, com dois eventos de perturbação de
%      corrente de carga (i_HESS) programados no tempo.
%   6) Salva os resultados em .mat e plota tensão/corrente/duty.
%
% Saída principal para uso no firmware: Kp_iL, Ki_iL, Kp_vC, Ki_vC
% (ganhos contínuos) e/ou Gains(1..4) (já discretizados para T=1/fs).
% =============================================================================

% Controle PI

% [CRITICAL] Caminho ABSOLUTO da máquina do Vini. Antes de rodar este
% script em outra máquina (ex.: a do Eric), troque para o caminho local
% onde o CasADi estiver instalado, ou adicione o CasADi ao MATLAB PATH
% permanentemente (Home > Set Path) e comente/remova esta linha.
addpath('C:\Users\vinic\OneDrive\Documentos\GitHub\casadi-3.7.2-windows64-matlab2018b')
import casadi.*

fs = 10e3; % modo fast 0.5e3     -- frequência de amostragem do CONTROLADOR [Hz] (mesma fs do artigo CBA2026)
T = 1/fs; % sampling time [s]    -- período de amostragem usado tanto na discretização dos PIs quanto na simulação
h = T;
N = 10; % prediction horizon    -- [não usado neste script: resquício copiado de um script de MPC; sem efeito aqui]
Ki = 1;                          % [não usado neste script: idem, resquício; não confundir com Ki_vC/Ki_iL abaixo]

% Half-bridge dc-dc converter parameters
% Estes 5 valores são os MESMOS parâmetros nominais da planta usados no
% artigo CBA2026 (Tabela "Plant and General Parameters"): Vdc=300V,
% C=2.3F, L=1mH, rL=0.18 Ohm. Se for adaptar este script para OUTRA planta
% (ex.: o boost do Conversores-VHF), troque estes 5 valores pelos
% parâmetros da nova planta -- o resto do script não precisa mudar.
global L r_L C V_dc K_PWM Kp_vC Ki_vC Kp_iL Ki_iL
L = 1e-3; % filter inductance
r_L = 0.18; % inductor series resistence
C = 2.3; % filter capacitance
V_dc = 300; % DC bus voltage
K_PWM = 5000; % resolução do registrador de comparação PWM (contagens que correspondem a duty=100%)

u_pwm_max = 0.99; u_pwm_min = 0.01; % [não usados neste script: limites de duty fracionário, resquício de outro script]

% Symbolic variables CasADi
%------ states
i_L = SX.sym('i_L'); v_C = SX.sym('v_C');
states = [i_L;v_C]; n_states = length(states);

%------ outputs
u_PWM = SX.sym('u_PWM');
controls = [u_PWM]; n_controls = length(controls);

%------ disturb
i_R = SX.sym('i_R');
disturb = [i_R]; n_disturb = length(disturb);

Ts = SX.sym('Ts'); % [não usado: T (numérico) é o que de fato é usado como passo de integração em shift_PI.m]

% linear model
% Espaço de estados contínuo x_dot = A_d*x + B_d*u + D_d*i_R, com
% x=[i_L; v_C]. Mesmo modelo do artigo CBA2026 (eqs. edo_i_continuos /
% edo_v_continuos): L*di_L/dt = -r_L*i_L - v_C + Vdc*u_PWM,
% C*dv_C/dt = i_L - i_R (i_R = corrente de carga/perturbação).
% [nota de nomenclatura] "A_d"/"B_d"/"D_d" aqui NÃO são as matrizes
% DISCRETAS (a notação "_d" do artigo CBA2026 é reservada para as
% matrizes discretizadas Ad=I+Ts*A, Bd=Ts*B); aqui "A_d"/"B_d"/"D_d" são
% na verdade o modelo CONTÍNUO -- só o nome das variáveis é uma
% coincidência/reaproveitamento de outro script, não confundir com a
% discretização de fato (que acontece só nos ganhos do PI, mais abaixo).
A_d = [-(r_L/L) -(1/L);(1/C) 0];
B_d = [V_dc/L;0];
D_d = [0; -(1/C)];
lm = A_d*[i_L; v_C] + B_d*u_PWM + D_d*i_R;
fprintf('==================\n')
fprintf('Original Model: \n')
print_dense(lm)

f = Function('f',{states,controls,disturb},{lm}); % nonlinear mapping function f(x,u) -- aqui o modelo é linear,
                                                    % mas é empacotado como função CasADi genérica por conveniência
                                                    % (mesma interface usada pelos scripts de MPC do projeto).

s = [];

% ---------------------------------------------------------------------
% PROJETO DOS GANHOS (tempo contínuo) -- ver PI_design.m para o método
% ---------------------------------------------------------------------
% Malha de TENSÃO (externa, lenta): planta Gpv.m (integrador 1/(C*s))
wcv = 2*pi*4;
% [REVISAR] o artigo CBA2026 (Tabela "T3: Classical PI Controller
% Parameters") reporta a frequência de cruzamento da malha de tensão como
% wcv = 2*pi*20 rad/s (fc=20 Hz), não 2*pi*4 (fc=4 Hz) como está aqui.
% Antes de repassar os ganhos gerados por este script para o Eric como
% "os ganhos oficiais do artigo", confirmar com o Vini qual dos dois é o
% valor final: se este script é uma versão anterior de ajuste (fc=4 Hz)
% e o artigo já usa a versão revisada (fc=20 Hz), ou vice-versa. A
% frequência de cruzamento da malha de CORRENTE (wci, logo abaixo) já
% bate exatamente com a tabela do artigo (2*pi*2000 rad/s).
MFv = 85;
[Kp_vC,Ki_vC] = PI_design(wcv,MFv,'Gpv');

% Malha de CORRENTE (interna, rápida): planta Gpi.m (primeira ordem RL)
wci = 2*pi*2e3;
MFi = 85;
[Kp_iL,Ki_iL] = PI_design(wci,MFi,'Gpi');

% ---------------------------------------------------------------------
% DISCRETIZAÇÃO dos dois PIs por Tustin/trapézio (bilinear transform)
% ---------------------------------------------------------------------
% Para um PI contínuo Gc(s)=Kp+Ki/s, a forma a diferenças equivalente por
% Tustin (substituindo s = (2/T)*(z-1)/(z+1)) é:
%     u[k] = u[k-1] + GainA*e[k] + GainB*e[k-1]
%     GainA = Kp + Ki*T/2 = (2*Kp + T*Ki)/2
%     GainB = -Kp + Ki*T/2 = (T*Ki - 2*Kp)/2
% Implementada em shift_PI.m; é a forma diretamente portável para C
% embarcado (guardar e[k-1] e u[k-1] a cada ciclo de amostragem).
% GainA_PI_i
Gains(1) = (2*Kp_iL + T*Ki_iL) * 0.5;
% GainB_PI_i
Gains(2) = (T*Ki_iL - 2*Kp_iL) * 0.5;
% GainA_PI_v
Gains(3) = (2*Kp_vC + T*Ki_vC) * 0.5;
% GainB_PI_v
Gains(4) = (T*Ki_vC - 2*Kp_vC) * 0.5;

% Verificação de viabilidade: ganhos negativos indicam que a combinação
% de wc/margem de fase escolhida não é fisicamente sensata para esta
% planta (o compensador "puxaria" na direção errada). Interrompe a
% execução (via loop vazio ou "quit") para o usuário revisar wc/MF antes
% de simular com ganhos sem sentido.
if (Kp_iL<0)
    disp(['unfeasible controller gains -> negative value /n Kp_iL =',num2str(Kp_iL)])
    s = input("continuar? s/n sim ou não: ","s");
    while(s ~= 's')
    end
end

if (Ki_iL<0)
    disp(['unfeasible controller gains -> negative value /n Ki_iL =',num2str(Ki_iL)])
    s = input("continuar? s/n sim ou não: ","s");
    if (s ~= 's')
        quit;
    end
end

%initial declarations

t_pi = [];
j = 1;
u_cl = [];
u_cl_v = [];
i_L_ref = [];
uPI = 0.333*K_PWM;     % chute inicial da saída do PI de corrente (33,3% de duty em contagens PWM)
e_ant = 0;
e_int_ant = 0;         % [não usado: resquício, o estado do integrador é mantido implicitamente em con/uPI, não aqui]
x0 = [0;90];           % estado inicial da planta: i_L=0, v_C=90V
con = 0;
e_i = 0;
e = 0;
t0 = 0;
xs = [0; 100];         % [não usado diretamente: seria o estado de regime permanente desejado, resquício da
                       % condição de parada comentada mais abaixo ("norm((x0-xs),2) > 1e-2")]

sim_tim = 20; % duração total da simulação [s]

main_loop = tic;
last_print_time = 0;
mpciter = 0;
total_steps = sim_tim/T;
reverseStr = '';
i_HESS = 0;            % perturbação de corrente de carga aplicada à referência da malha de corrente (ver abaixo)
time = 0;
uPI_ant = 0.333*K_PWM;

step_loop_time = zeros(1,total_steps);
xx = zeros(2,total_steps);
u_cl = zeros(1,total_steps);
u_cl_v =  zeros(1,total_steps);
i_L_ref =  zeros(1,total_steps);
t_pi =  zeros(1,total_steps);

% ---------------------------------------------------------------------
% LAÇO DE SIMULAÇÃO EM MALHA FECHADA (tempo discreto, passo T)
% ---------------------------------------------------------------------
while( j < sim_tim / T) %norm((x0-xs),2) > 1e-2 &&
    step_loop = tic;
    d = 0;

    con0 = con;
    uPI_ant = uPI;
    e_ant = e;
    e_i_ant = e_i;
    ref = 100;         % referência de tensão do supercapacitor: 100V constante ao longo da simulação

    % Cenário de teste: dois degraus de perturbação de CORRENTE DE CARGA
    % (i_HESS), simulando o HESS absorvendo (-20A, em t=10.0-10.1s) e
    % depois fornecendo (+20A, em t=12.0-12.4s) energia ao barramento,
    % para verificar a rejeição a distúrbio da malha de corrente (já com
    % o PI de tensão desativado nesse intervalo, ver shift_PI.m).
    if t0 > 10.0 && t0 < 10.1
        i_HESS = -20;
    else
        if t0 > 12.0 && t0 < 12.4
            i_HESS = 20;
        else
            i_HESS = 0;
        end
    end
    [t0, x0, con, uPI, e, e_i] = shift_PI(T, t0, x0, con0, uPI_ant, d, f, e_ant, e_i_ant, ref, j, i_HESS, Gains, K_PWM);
    xx(:,j+1) = x0;

    u_cl(j+1) = uPI;
    u_cl_v(j+1) = con;
    i_L_ref(j+1) = u_cl_v(j)+i_HESS;
    t_pi(j+1) = t0;

    %
    step_loop_time(j) = toc(step_loop);


    j = j+1;

        % --- Barra de Progresso (Atualiza a cada 10 iterações) ---
    if toc(main_loop) - last_print_time > 1 || mpciter == total_steps
        percent = (mpciter / total_steps) * 100;
        bar_len = 100;
        num_bars = floor(percent/100 * bar_len);

        prog_bar = ['[' repmat('=', 1, num_bars) repmat(' ', 1, bar_len - num_bars) ']'];
        msg = sprintf('%s %.1f \n Vo = %.3f \n iL = %.1f \n v_ref = %.1f \n u = %.0f \n', prog_bar, percent,x0(2),x0(1),ref,uPI);

        fprintf("\n")

        % Imprime
        fprintf([reverseStr]);
        fprintf(msg)

        % Prepara para apagar e atualiza o timer
        reverseStr = repmat('\b', 1, length(msg)+1);
        last_print_time = toc(main_loop);

        % Força o MATLAB a desenhar agora (evita buffer overflow)
        % drawnow limitrate;
    end
    % -------------------------------------------

    mpciter=mpciter+1;

end


% ---------------------------------------------------------------------
% RESULTADOS: figuras 5-7 (corrente/referência, tensão, duty) e .mat
% ---------------------------------------------------------------------
figure(5)
plot(t_pi(1:j),xx(1,1:j),t_pi(1:j),i_L_ref(1:j))
figure(6)
plot(t_pi(1:j),xx(2,1:j))
figure(7)
plot(t_pi(1:j),u_cl(1:j))

iL_T3 = xx(1,:);
vSC_T3 = xx(2,:);
u_PWM = u_cl;

% [CRITICAL] Caminho ABSOLUTO da máquina do Vini. Antes de rodar este
% script em outra máquina (ex.: a do Eric), troque para um caminho válido
% local, ou torne relativo (ex.: fullfile(fileparts(mfilename('fullpath')),
% '..', 'results', 'result_T3.mat')), senão o save() abaixo falha com erro
% de caminho não encontrado.
save('C:\Users\vinic\OneDrive\Documentos\GitHub\vscode\research\pos_doc\MATLAB\CBA2026\results\result_T3.mat', 't_pi', 'u_PWM', 'vSC_T3', 'iL_T3', 'Kp_iL', 'Ki_iL', 'Kp_vC', 'Ki_vC', 'wcv', 'wci', 'MFi', 'MFv', 'mpciter','u_cl','u_cl_v')

% [CRITICAL] Bloco morto/resquício: as variáveis "t", "v_T2", "iL_T2" e
% "vSC_T2" usadas nas figuras 1-4 abaixo NUNCA são definidas neste script
% (foram copiadas de outro script, provavelmente um "T2_..." que gerava
% esses resultados). Rodar o script até aqui já produz e SALVA os
% resultados corretamente (a linha save() acima já rodou); o script só vai
% ERRAR nas 4 linhas abaixo com "Unrecognized variable" (ou, se "u_PWM"
% escapar por já existir da linha 191, com erro de índice, pois é um vetor
% linha e aqui é indexado como se fosse coluna: "u_PWM(1:mpciter,1)").
% Comentar/remover este bloco (figuras 1-4) antes de repassar o script ao
% Eric, para não parecer que a simulação falhou.
figure(1)
plot(t(1,1:mpciter),v_T2(1,1:mpciter))
figure(2)
plot(t(1,1:mpciter),iL_T2(1,1:mpciter))
figure(3)
plot(t(1,1:mpciter),vSC_T2(1,1:mpciter))
figure(4)
plot(t(1,1:mpciter),u_PWM(1:mpciter,1))
