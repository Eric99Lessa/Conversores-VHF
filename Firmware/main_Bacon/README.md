# Firmware `Conversores-VHF` comentado

Cópia comentada dos 16 arquivos de `Firmware/main/` do repositório
[`viniciusbacon/Conversores-VHF`](https://github.com/viniciusbacon/Conversores-VHF)
(firmware ESP32 do conversor CC-CC do Eric). Cada arquivo explica, linha a
linha, o que o código faz, e marca com tags os pontos discutidos no
relatório de diagnóstico:

```
pos_doc/report/Diagnostico_Firmware_Conversores_VHF.pdf
```

**Leia o relatório primeiro.** Ele tem o contexto completo (por que o loop
fechado não funciona, comparação com o PI cascata do artigo CBA2026,
prioridade de cada correção). Este README é só o guia rápido de uso dos
arquivos comentados; os comentários dentro do código têm o detalhe técnico.

## O que NÃO foi mudado

**Nenhuma linha de código foi alterada.** Só foram adicionados comentários
(verificado por *diff* contra um clone independente do repositório original,
ignorando apenas linhas de comentário). Estes arquivos compilam e se
comportam exatamente como os originais do Eric; a única diferença é a
documentação inline.

## Estrutura

```
Firmware/main/
├── config.h              parametros da planta, pinos, macros de compilacao, struct compartilhada
├── main.c                ponto de entrada: decide se o binario e o ESP "de controle" ou "de comunicacao"
├── control.c / control.h laco de controle a 1 kHz (o arquivo mais importante: le o relatorio antes deste)
├── model.c / model.h     modelo medio do conversor (boost/buck) + calculo de corrente de equilibrio
├── observer.c / observer.h  filtro de Kalman escalar (estimacao de estado)
├── optim.c / optim.h     gradiente + Adam (identificador MRAC online, generico)
├── adc.c / adc.h         leitura e calibracao das 6 grandezas medidas
├── communication.c / communication.h  protocolo SPI entre os dois ESP32
└── uart_task.c / uart_task.h          console serial (comandos M/P/V/I/D)
```

Isso é **apenas o conteúdo de `Firmware/main/`**: não inclui `CMakeLists.txt`,
`sdkconfig` nem o resto do projeto ESP-IDF (`Firmware/`). Esta pasta não é um
projeto compilável sozinha.

## Como usar

Estes arquivos servem para **ler e revisar**, não para compilar direto daqui.
Para de fato usá-los no firmware:

1. Abra sua cópia local do projeto ESP-IDF `Conversores-VHF` (a pasta
   `Firmware/`, com `CMakeLists.txt`, `sdkconfig`, etc.).
2. Compare os arquivos aqui com os seus, arquivo por arquivo, para ver os
   comentários e as marcações `[CRITICAL]`/`[REVISAR]` no contexto de cada
   trecho de código.
3. Se quiser manter os comentários no seu projeto de trabalho, copie estes
   arquivos por cima de `Firmware/main/*.c` e `*.h` (eles são
   funcionalmente idênticos aos originais, então não muda o comportamento
   do firmware, só documenta).
4. Aplique as correções descritas no relatório diretamente nos arquivos do
   SEU projeto (`Firmware/main/`), não aqui: esta pasta é só a cópia
   comentada de referência do estado do código em 23/09/2026, e não deve
   virar a fonte de verdade do firmware.

## Legenda das marcações nos comentários

| Tag | Significado |
|---|---|
| `[CRITICAL]` | Item do relatório com prioridade alta ou máxima: candidato a explicar por que o loop fechado não funciona, ou risco real de segurança/comportamento inesperado. |
| `[REVISAR]` | Prioridade média ou baixa: não impede o teste imediato, mas vai incomodar mais cedo ou mais tarde. |

## Os 4 pontos `[CRITICAL]` mais importantes (resumo; detalhe completo no relatório)

1. **`control.c`, `update_duty()` / `duty_cascade()`**: o duty é calculado
   como fração `[0,1]`, mas `mcpwm_set_duty()` (driver legado ESP-IDF)
   espera porcentagem `[0,100]`. Falta multiplicar por 100.
2. **`control.c`, `control_task()`**: `Kp_i`, `Kp_v`, `Ki_v` estão fixados em
   `1.0` (placeholder), não vêm do projeto de ganhos em `Matlab/`.
3. **`control.c`, laço principal**: sem saturação (`clamp`) do duty nem
   anti-windup no integrador de tensão antes de `update_duty()`.
4. **`uart_task.c`, `uart_task()`**: o comando de console `D` (duty manual)
   está implementado (`parse_d_command`) mas nunca é chamado no despacho de
   comandos; digitar `D <valor>` no console sempre retorna erro.

## Ver também

- Relatório completo: `pos_doc/report/Diagnostico_Firmware_Conversores_VHF.pdf`
- Scripts de projeto de PI (Bode + discretização, usados como referência
  metodológica, não os ganhos originais deste firmware):
  `pos_doc/work/VHF/MATLAB/README.md`
