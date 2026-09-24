%% Projeto de controle por análise da resposta em frequência
%
% PI_design(): projeta um compensador PI contínuo Gc(s) = Kp + Ki/s pelo
% método clássico de resposta em frequência (às vezes chamado de "K-factor"
% ou "magnitude invariance"): dado uma planta G(s), uma frequência de
% cruzamento de ganho desejada wc [rad/s] e uma margem de fase desejada
% fid_graus [graus], calcula Kp e Ki tais que o ganho de malha aberta
% Gc(jwc)*G(jwc) tenha módulo unitário (cruzamento em wc) E fase tal que a
% margem de fase resultante seja exatamente fid_graus.
%
% Esta é a mesma técnica usada para projetar as duas malhas do controlador
% PI cascata clássico (C-PI) do artigo CBA2026 (Tabela T3): a malha de
% tensão (externa, lenta) é projetada chamando esta função com a planta
% Gpv (integrador puro 1/(Cs)), e a malha de corrente (interna, rápida) com
% a planta Gpi (primeira ordem do indutor). Ver T3_PI_SC.m para o uso
% completo, incluindo a discretização das duas malhas.
%
% Entradas:
%   wc         - frequência de cruzamento de ganho desejada [rad/s]
%                (ex.: wc = 2*pi*fc, com fc em Hz)
%   fid_graus  - margem de fase desejada [graus]
%   planta     - nome (string) da função de transferência da planta em s,
%                ex. 'Gpi' ou 'Gpv' (arquivos .m nesta mesma pasta,
%                despachados via str2func/feval)
%
% Saídas:
%   Kp_bode, Ki_bode - ganhos proporcional e integral do PI CONTÍNUO
%                      (ainda não discretizado; ver T3_PI_SC.m para a
%                      discretização por Tustin/trapézio)
%
% Dedução das fórmulas (Kp_bode, Ki_bode):
%   Para Gc(s)=Kp+Ki/s, temos Gc(jwc) = Kp - j*(Ki/wc), ou seja, um número
%   complexo de módulo Mc_real=sqrt(Kp^2+(Ki/wc)^2) e fase fic (o ângulo
%   deste número complexo). Se escrevermos Kp=Mc*cos(fic) e
%   Ki/wc=-Mc*sin(fic), então Gc(jwc) tem exatamente módulo Mc e fase fic,
%   para qualquer par (Mc, fic) escolhido. Basta entao escolher:
%     Mc  = módulo que faz |Gc(jwc)*G(jwc)| = 1  =>  Mc = 1/|G(jwc)|
%     fic = fase que faz a margem de fase = fid_graus
%           (fase total de malha aberta em wc deve ser -180°+MF; como a
%           planta já contribui com fase fig, o compensador deve fornecer
%           a fase restante: fic = (fid - 180°) - fig, em radianos:
%           fic_rad = fid_rad - (fig_rad + pi))

function [Kp_bode,Ki_bode]=PI_design(wc,fid_graus,planta)
% fc=120;             %frequencia de cruzamento
% fid_graus=80;       %Margem de fase desejada
% wc=2*pi*fc/3;       %frequencia de cruzamento
planta_f = str2func(planta);          % handle para a função de transferência (Gpi.m ou Gpv.m nesta pasta)
fid_rad=2*pi*fid_graus/360;           % margem de fase desejada, de graus para radianos
% ws=2*pi*fc;
s=tf('s');                            % variável de Laplace simbólica (Control System Toolbox) -- usada só
                                       % para montar G abaixo com fins de inspeção; não entra nas contas de Kp/Ki.
G=planta_f(s);                        % G(s) simbólica da planta (não usada além desta linha)
Mg=abs(planta_f(1j*wc));              % |G(jwc)|: módulo da planta na frequência de cruzamento desejada
Mc=1/Mg;                              % módulo que o compensador precisa ter para |Gc(jwc)*G(jwc)| = 1
fig_rad=angle(planta_f(1j*wc));       % fase da planta em wc [rad]
fic_rad=fid_rad-(fig_rad+pi);         % fase que o compensador precisa fornecer para atingir a margem de fase desejada
fig_graus=360*fig_rad/(2*pi);         % (só para inspeção/debug em graus)
fic_graus=360*fic_rad/(2*pi);         % (só para inspeção/debug em graus)
Kp_bode=Mc*cos(fic_rad);              % ganho proporcional do PI contínuo
Ki_bode=-Mc*wc*sin(fic_rad);          % ganho integral do PI contínuo
% s=tf('s');
% Gc=(Ki_bode+Kp_bode*s)/s;
% Tmf=feedback(Gc*G,1);
