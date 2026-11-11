# Atualização e validação do firmware — 10/11/2026

> Registro da primeira atualização. O filtro por eixo descrito abaixo foi posteriormente substituído por um filtro complementar em quaternion; veja `Correcao_Yaw.md` para a versão atual, testes e nova gravação.

## Resultado

Firmware compilado para STM32F411CEU6 e gravado por ST-LINK, com verificação de Flash bem-sucedida (`Verified OK`). O identificador lido foi `0x10006431`, compatível com STM32F411; a ferramenta detectou 512 KiB de Flash. A placa permaneceu executando o firmware após a gravação.

Alterações implementadas:

- Duas páginas automáticas, alternadas a cada 2 s: atitude e telemetria completa AX/AY/AZ, WX/WY/WZ.
- Horizonte de Roll/Pitch e seta animada de Yaw relativo em ambas as páginas.
- Aquisição prioritária com prazo de 10 ms, Δt medido, atualização do quadro a cada 100 ms e envio de uma página de 128 bytes por etapa. O framebuffer fica congelado até concluir o quadro.
- Correção do modo de endereçamento SSD1306 para modo página, coerente com os comandos B0…B7 usados no driver.
- Calibração inicial do bias XYZ do giroscópio, com 200 amostras e rejeição de janelas com movimento detectado. Repetição automática quando a calibração falha.
- Roll/Pitch alinhados à gravidade na primeira amostra, peso complementar ajustado ao Δt e correção angular curta para Roll em ±180°. A correção gravitacional de Roll é suspensa perto da orientação singular de Pitch ±90°.
- Propagação de erros na configuração da IMU e nas transmissões do OLED; tela de dados inválidos e LED de erro. Intervalos com falha de aquisição não são extrapolados na integração.
- Build reproduzível com Makefile, startup oficial, linker, SysTick, heap e dependências oficiais fixadas por commit.

## Evidências na placa

Os contadores `appDiagnostics` foram lidos por SWD com a CPU em execução. Na primeira janela registrada em `build/runtime-check.log`:

| Contador | Início | Fim | Diferença |
| --- | ---: | ---: | ---: |
| Tick HAL (ms) | 16858 | 21873 | 5015 |
| Leituras válidas | 1451 | 1952 | 501 |
| Quadros concluídos | 145 | 195 | 50 |
| Erros IMU | 0 | 0 | 0 |
| Erros OLED | 0 | 0 | 0 |
| Prazos perdidos | 0 | 0 | 0 |

Taxas observadas: aproximadamente **99,9 leituras/s** e **10,0 quadros/s**. Maior intervalo entre leituras desde aquela inicialização: **11 ms**. Houve uma tentativa de calibração naquela inicialização. Uma captura posterior apresentou cinco tentativas de calibração após outra inicialização; esses contadores reiniciam junto com o firmware e não devem ser somados entre inicializações.

O framebuffer foi capturado em duas páginas distintas e inspecionado visualmente: `build/oled-frame-a.png` (sensores) e `build/oled-frame-b.png` (atitude). Foram conferidos os seis eixos, a legibilidade, o horizonte e a seta de Yaw. São imagens reconstruídas da RAM do firmware, não fotografias do painel físico. As transmissões do OLED não registraram erros nas capturas.

## Testes no computador

`make test` passou com `-Wall -Wextra -Werror`. A comunicação é simulada para exercitar:

- Burst de 14 bytes, reconstrução com sinal e escalas g, °/s e temperatura.
- Falha de leitura e falha em cada escrita de configuração.
- Calibração de três eixos, rejeição de movimento e preservação dos offsets anteriores quando a calibração falha.
- Alinhamento inicial, integração com Δt variável, constante de tempo do filtro, passagem por ±180° e comportamento próximo de Pitch ±90°.
- Envio de exatamente uma página por etapa, proteção do framebuffer durante envio e recuperação após falha de transmissão.
- Alterações visuais para WX, WY e Yaw, tela de dados inválidos e coordenadas da animação em diferentes ângulos.

O ambiente não dispõe da biblioteca de execução AddressSanitizer para o compilador nativo; os testes passaram sem sanitizers. O build ARM passou; os avisos emitidos vieram de parâmetros não usados na HAL oficial, syscalls não usadas da newlib e permissões de segmentos do linker bare-metal. Não houve avisos de compilação nos fontes da aplicação.

## Ferramentas e artefatos

- ARM GNU 14.3.1, disponibilizado pelo STM32CubeIDE.
- HAL STM32F4 v1.8.3, commit `c2e1406d7ea4b73aa42b98ddeb75a8670b1a5a16`.
- CMSIS Device F4 v2.6.10, commit `5f41fb29d22773896c780052bf61e47fc924d524`.
- CMSIS 5.9.0, commit `2b7495b8535bdcb306dac29b9ded4cfb679d7e5c`.
- `build/firmware.elf` e `build/firmware.bin`: executáveis gravados.
- `Backups/stm32f411-before-20261110.bin`: backup integral de 524.288 bytes da Flash anterior.
- `build/flash.log`, `build/runtime-check.log` e `build/display-check.log`: registros da gravação e verificações.

SHA-256 do binário gravado: `cdfcc91fa3bc34b16b83d37e8d9889d83df2cb6da693c8654006882826ef19fb`.

SHA-256 do backup anterior: `6c54d68b68a17decb84687f9fbf2ef4b17c5cc59c51fcfc5178c79f06c701905`.

O uso de Flash estimado pelo build é 33.544 bytes (text + data). BSS: 6.684 bytes, incluindo as reservas estáticas de heap/pilha do linker.

## Limites da validação

As taxas são observações de uma janela curta, baseadas no tick HAL; não são uma medição independente com analisador lógico. Não foi realizada comparação física controlada dos ângulos com uma referência de ±90° nem ensaio de deriva de longa duração. A integração independente por eixo continua sendo uma aproximação para movimentos separados. A aceleração exibida inclui gravidade; Yaw é relativo e pode derivar. A detecção de repouso da calibração não distingue perfeitamente um bias de uma rotação lenta constante em torno de Z.

Antes de demonstrar: reinicie com a montagem parada, aguarde terminar a calibração e faça movimentos separados por eixo, com pausas nas posições de referência.
