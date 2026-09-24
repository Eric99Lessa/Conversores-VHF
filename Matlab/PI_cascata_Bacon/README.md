# Projeto de PI cascata (Bode) e discretização: pasta MATLAB

Esta pasta contém os scripts usados para **projetar e validar o controlador PI
cascata clássico (C-PI)** do artigo *"Performance Improvement of Bidirectional
Converters in HESS Using MPC"* (CBA2026), Tabela "T3: Classical PI Controller
Parameters". É a mesma planta descrita na tabela do artigo:

| Parâmetro | Valor |
|---|---|
| Tensão do barramento CC, `V_dc` | 300 V |
| Capacitância do filtro, `C` | 2,3 F |
| Indutância do filtro, `L` | 1 mH |
| Resistência série do indutor, `r_L` | 0,18 Ω |
| Frequência de amostragem, `fs` | 10 kHz |

Conversor: half-bridge CC/CC bidirecional entre o barramento CC e o
supercapacitor do HESS.

## Arquivos

| Arquivo | O que faz |
|---|---|
| `Gpi.m` | Função de transferência (Laplace, `s`) da planta de **corrente** (malha interna): `Gpi(s) = 1 / ((V_dc/K_PWM)*(r_L + L*s))`. |
| `Gpv.m` | Função de transferência da planta de **tensão** (malha externa): `Gpv(s) = 1/(C*s)` (integrador puro). |
| `PI_design.m` | Projeta um PI contínuo `Kp + Ki/s` por resposta em frequência (método de cruzamento de ganho + margem de fase), dada uma planta (`Gpi` ou `Gpv`), a frequência de cruzamento desejada e a margem de fase desejada. |
| `shift_PI.m` | Avança a simulação em malha fechada um passo de amostragem: executa os dois PIs já **discretizados** (forma recursiva) em cascata e integra o modelo da planta. É o "firmware simulado", a lógica que uma implementação em C deveria replicar. |
| `T3_PI_SC.m` | **Script principal.** Define os parâmetros da planta, chama `PI_design.m` duas vezes, discretiza os dois PIs (Tustin/trapézio), simula a malha fechada por 20 s com dois degraus de carga, plota os resultados e salva um `.mat`. |

Todos os 5 arquivos já estão comentados linha a linha (em português), explicando
o processo e as fórmulas usadas; comece por eles se quiser entender os
detalhes, este README é só o guia de uso e os avisos práticos.

## Como rodar (passo a passo)

1. Abra o MATLAB e navegue até esta pasta (`cd` até aqui, ou adicione ao path).
2. **Antes de rodar**, resolva os dois avisos `[CRITICAL]` abaixo (seção
   "Antes de rodar na sua máquina").
3. Rode o script principal:
   ```matlab
   T3_PI_SC
   ```
4. O script vai:
   - imprimir o modelo linear da planta no console;
   - projetar e imprimir (via as variáveis de workspace) os ganhos contínuos
     `Kp_vC`, `Ki_vC` (malha de tensão) e `Kp_iL`, `Ki_iL` (malha de corrente);
   - calcular os 4 coeficientes discretos em `Gains(1..4)`;
   - simular ~20 s de malha fechada (demora alguns segundos, com barra de
     progresso no console) e abrir as Figuras 5, 6 e 7 (corrente de indutor
     e referência, tensão do capacitor, duty aplicado);
   - salvar os resultados em `.mat` (ver caminho no aviso `[CRITICAL]` abaixo).
5. **O script vai terminar com um erro** nas últimas 8 linhas (Figuras 1 a 4);
   isso é esperado, ver aviso abaixo. Os resultados úteis (ganhos + `.mat`) já
   foram gerados e salvos ANTES desse erro; pode ignorar.

## De onde vêm os ganhos que você precisa

Depois de rodar `T3_PI_SC`, os ganhos ficam nestas variáveis do workspace do
MATLAB:

- **Ganhos contínuos** (Kp, Ki de cada PI, unidade "tempo contínuo"):
  `Kp_vC`, `Ki_vC` (tensão) e `Kp_iL`, `Ki_iL` (corrente).
- **Ganhos discretos prontos para implementação em C** (forma recursiva
  `u[k] = u[k-1] + GainA*e[k] + GainB*e[k-1]`, discretizados por
  Tustin/trapézio para o período de amostragem `T = 1/fs`):
  `Gains(1)` = GainA da corrente, `Gains(2)` = GainB da corrente,
  `Gains(3)` = GainA da tensão, `Gains(4)` = GainB da tensão.
  Esta é a forma que `shift_PI.m` já implementa e é a que deve ser copiada
  para um firmware: a cada ciclo de amostragem, guardar o erro e a saída do
  passo anterior e aplicar a mesma equação a diferenças.

## Adaptando para outra planta (ex.: o boost do `Conversores-VHF`)

O método aqui é genérico, não específico deste conversor. Para projetar um PI
para outra planta:

1. Escreva a função de transferência da nova planta em um arquivo próprio
   (copie `Gpi.m`/`Gpv.m` como modelo), com o **nome do arquivo igual ao nome
   usado depois em `PI_design(...)`**.
2. Ajuste os parâmetros físicos globais no topo de `T3_PI_SC.m` (`L`, `r_L`,
   `C`, `V_dc`, `K_PWM`, `fs`) para os da nova planta.
3. Escolha `wc` (frequência de cruzamento) e a margem de fase desejada para
   cada malha, e chame `PI_design.m`.
4. Reaproveite as fórmulas de discretização (linhas "Discretization" em
   `T3_PI_SC.m`) para obter os `Gains(1..4)` da nova planta.

**Atenção ao usar isto para o firmware `Conversores-VHF` (Eric):** a malha de
**tensão** desse firmware (`control.c`, `Kp_v`/`Ki_v`) já tem estrutura de PI
clássico, e este método serve diretamente para projetá-la. Já a malha de
**corrente** (`duty_cascade()`) daquele firmware **não é um PI clássico**: é
uma lei de linearização por realimentação com um único ganho `Kp_i` (ver
`pos_doc/report/Diagnostico_Firmware_Conversores_VHF.pdf`, Seção 4). Este
método de projeto por Bode não se aplica diretamente a `Kp_i` a menos que se
decida trocar aquela lei de controle por um PI clássico convencional, o que,
inclusive, pode ser uma alternativa mais simples de depurar caso a lei atual
continue dando problema.

## Antes de rodar na sua máquina (Eric)

Dois caminhos absolutos estão hard-coded para a máquina do Vini e **vão dar
erro** se você rodar sem ajustar:

1. **`T3_PI_SC.m`, linha do `addpath(...)`**: aponta para a instalação do
   CasADi na máquina do Vini
   (`C:\Users\vinic\...\casadi-3.7.2-windows64-matlab2018b`). Baixe o CasADi
   para MATLAB em <https://web.casadi.org/get/> (escolha a versão compatível
   com sua versão do MATLAB) e troque essa linha para o caminho onde você
   extraiu, ou adicione o CasADi ao MATLAB PATH permanentemente (`Home > Set
   Path`) e remova/comente a linha.
   - CasADi aqui só empacota o modelo linear da planta como uma função
     simbólica conveniente; se preferir não instalar o CasADi, é possível
     reescrever `shift_PI.m` para integrar `A_d*x + B_d*u + D_d*d` direto com
     álgebra de matrizes do MATLAB puro, sem `Function`/`SX.sym`.
2. **`T3_PI_SC.m`, linha do `save(...)`** (perto do fim, antes das Figuras
   1-4): aponta para
   `C:\Users\vinic\...\pos_doc\MATLAB\CBA2026\results\result_T3.mat`, uma
   pasta que só existe na máquina do Vini. Troque para um caminho local seu,
   por exemplo `save('result_T3.mat', ...)` para salvar na pasta atual.

## Problemas conhecidos nestes scripts (não bloqueiam o uso, mas confundem)

- **`Gpv.m` tem a função interna nomeada `Gpi`** (linha `function f=Gpi(x)`),
  em vez de `Gpv`: resquício de copiar `Gpi.m` como ponto de partida. Isso
  **não quebra nada**: o MATLAB despacha pelo nome do ARQUIVO (`Gpv.m`), não
  pelo nome declarado dentro dele, então `PI_design(wcv,MFv,'Gpv')` continua
  chamando o arquivo certo. Mas é confuso se alguém abrir só esse arquivo.
- **As últimas 8 linhas de `T3_PI_SC.m` (Figuras 1 a 4) vão dar erro.** Usam
  variáveis (`t`, `v_T2`, `iL_T2`, `vSC_T2`) que nunca são definidas neste
  script; sobraram de outro script (`T2`) que gerava esses dados. O erro
  acontece **depois** de os resultados úteis (ganhos + `.mat`) já terem sido
  calculados e salvos, então pode ser ignorado, mas o ideal é apagar essas 8
  linhas antes de repassar o script adiante.
- **Discrepância na frequência de cruzamento da malha de tensão**: este
  script usa `wcv = 2*pi*4` (4 Hz), mas a Tabela T3 do artigo CBA2026 lista
  `ωcv = 2π×20` rad/s (20 Hz). A malha de corrente bate exatamente com o
  artigo (`wci = 2*pi*2e3`, ou seja, 2 kHz). Confirmar com o Vini qual valor
  de `wcv` é o definitivo antes de considerar os ganhos de tensão gerados
  aqui como "os do artigo publicado", pode ser uma versão de ajuste anterior
  à revisão final.
