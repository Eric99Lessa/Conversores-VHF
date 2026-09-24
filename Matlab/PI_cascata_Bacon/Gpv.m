%%%%% Definição da função transferência %%%%%
%---------------------------------------
% -> planta de TENSÃO do conversor half-bridge CC/CC bidirecional (HESS,
%    supercapacitor), mesma topologia do artigo CBA2026.
%
% Modelo: C*dv_C/dt = i_L (o indutor carrega/descarrega o capacitor de
% saída), então a função de transferência de pequenos sinais de v_C(s) por
% i_L(s) é um integrador puro:
%     Gpv(s) = v_C(s)/i_L(s) = 1/(C*s)
% Esta é a planta da MALHA DE TENSÃO (externa, mais lenta), cuja saída é a
% referência de corrente para a malha interna (Gpi.m). Note que a variável
% de entrada "i_L" nesta malha em cascata é o próprio sinal de controle da
% malha de tensão (análogo ao papel de "u_PWM" na malha de corrente).
%
% Usada por PI_design.m (chamada como planta_f = str2func('Gpv')).
%
% [REVISAR] a função abaixo está declarada como "function f=Gpi(x)", ou
% seja, com o MESMO nome de função usado em Gpi.m, em vez de "Gpv". Isso é
% um resquício de copiar Gpi.m como ponto de partida e não é um erro
% funcional: o MATLAB despacha por NOME DO ARQUIVO (Gpv.m), não pelo nome
% declarado dentro da função, então "str2func('Gpv')" continua chamando
% este arquivo corretamente. Ainda assim, é confuso e vale renomear a
% declaração abaixo para "function f=Gpv(x)" na próxima revisão, para
% evitar erros de copy-paste futuros (ex.: se alguém abrir só este arquivo
% e chamar "Gpi(x)" direto, esperando a planta de corrente, vai na
% verdade rodar a planta de tensão sem perceber).
%
% Variável global (definida em T3_PI_SC.m antes de chamar PI_design):
%   C - capacitância do filtro de saída [F]

function f=Gpi(x)
global C
f=(1/(C*x));
