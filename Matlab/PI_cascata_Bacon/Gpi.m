%%%%% Definição da função transferência %%%%%
%---------------------------------------
% -> planta de CORRENTE do conversor half-bridge CC/CC bidirecional (HESS,
%    supercapacitor), a mesma topologia e parâmetros do artigo CBA2026
%    (Vdc=300V, C=2,3F, L=1mH, rL=0,18 Ohm, fs=10kHz).
%
% Modelo: a partir de L*di_L/dt = -r_L*i_L + Vdc*d(t) (d = duty, aprox.
% linearizado em torno do ponto de operação), a função de transferência de
% pequenos sinais de i_L(s) por uPI(s) é:
%     Gpi(s) = i_L(s)/uPI(s) = 1 / ( (Vdc/K_PWM) * (r_L + L*s) )
% O fator K_PWM (definido global, ver T3_PI_SC.m) converte o sinal de saída
% do compensador uPI (em contagens do registrador de comparação PWM, não em
% duty fracionário [0,1]) para duty efetivo: d = uPI/K_PWM. É por isso que
% Vdc aparece dividido por K_PWM aqui, e não Vdc puro.
%
% Usada por PI_design.m (chamada como planta_f = str2func('Gpi')) para
% projetar o compensador PI da MALHA DE CORRENTE (a mais interna, mais
% rápida) por resposta em frequência.
%
% Variáveis globais (definidas em T3_PI_SC.m antes de chamar PI_design):
%   L      - indutância do filtro [H]
%   r_L    - resistência série do indutor [Ohm]
%   C      - capacitância do filtro (não usada aqui, só na malha de tensão)
%   V_dc   - tensão do barramento CC [V]
%   K_PWM  - resolução do registrador de comparação PWM (contagens para duty=1)

function f=Gpi(x)
global L r_L C V_dc K_PWM
f=1/((V_dc/K_PWM)*(r_L+L*x));
