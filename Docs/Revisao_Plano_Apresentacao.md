# Revisão do plano e roteiro — Lab I2C STM32

> Registro histórico da versão `444c893`, anterior às correções do firmware. Para o comportamento atualizado e a validação na placa, consulte `Validacao_Firmware.md` nesta pasta.

Revisão de 10/11/2026, baseada na branch `board/stm32f411ceu6`, commit `444c893`, no texto fornecido e na especificação `/home/Rodrigues/Lab I2C STM32.pdf` (página única).

Escopo: inspeção do código, leitura da especificação e cálculos de orçamento temporal. Não houve compilação completa, execução no STM32, medição de barramento ou validação da montagem/vídeo. O firmware não foi alterado.

## 1. Parecer

O plano descreve corretamente a maior parte da implementação, mas o checklist de atendimento integral precisa ser corrigido. Há duas lacunas explícitas em relação à especificação: não são exibidos os três eixos do giroscópio, e a animação gráfica não representa Yaw. A temporização também não sustenta a afirmação de filtro garantido a 100 Hz com integração precisa.

## 2. Correções necessárias

| Afirmação do plano | Verificação e redação recomendada | Evidência |
| --- | --- | --- |
| “Requisitos atendidos” | Atendimento parcial: o OLED mostra aceleração XYZ, Roll/Pitch/Yaw e somente velocidade angular Z. Faltam velocidades angulares X e Y no display. | Especificação, requisitos mínimos; `Core/Src/Ssd1306.c:279–298`. |
| “Requisito avançado atendido” | A linha animada representa Roll e Pitch. Yaw aparece apenas como número; falta sua representação gráfica para declarar atendimento inequívoco à animação dos três ângulos. | Especificação, requisitos avançados; `Core/Src/Ssd1306.c:252–267,286–287`. |
| “Superloop determinístico, filtro a 100 Hz e OLED a 10 FPS” | São metas nominais. O OLED bloqueia o superloop por mais de 10 ms em cada atualização, e a decimação conta iterações, não intervalos reais de 100 ms. | `Core/Src/main.c:61–84`; `Core/Src/Ssd1306.c:155–163`; `Core/Src/Bsp.c:59–64`. |
| “Compensação via HAL_GetTick garante integração precisa” | O código só reduz a espera quando a iteração termina antes de 10 ms. Não recupera atrasos e passa sempre `0.01f` ao filtro. Não há Δt medido entre amostras. | `Core/Src/main.c:19–21,67,79–84`. |
| Equações com `atan2` somado diretamente a graus | Multiplicar os dois ângulos do acelerômetro por `180/π`, como faz o firmware. | `Core/Src/ComplementaryFilter.c:20,61–75`. |
| “R, P, Y pós-filtro complementar” | Roll/Pitch usam fusão; Yaw é somente integração de ωz, sem correção por referência absoluta ou compensação de bias. | `Core/Src/ComplementaryFilter.c:68–75`. |
| “Ausência de drift em Roll/Pitch” | Usar “correção gradual da deriva de Roll/Pitch pela referência gravitacional, em condições aproximadamente estáticas”. Não prometer ausência de erro. | `Core/Src/ComplementaryFilter.c:61–72`. |
| “±90° cravados” e direita/esquerda com sinais fixos | Esperar valores aproximados, após estabilização; definir movimentos pelos eixos do sensor. O sinal físico depende da orientação do módulo. Próximo de Pitch ±90°, Roll pelo acelerômetro fica mal condicionado porque Ay e Az se aproximam de zero. | Equações em `Core/Src/ComplementaryFilter.c:61–64`; ausência de remapeamento em `Core/Src/Mpu6050.c:102–108`. |
| “Aceleração linear” | O firmware apenas aplica a escala em g, sem remover gravidade. Dizer “aceleração medida em g, incluindo a contribuição gravitacional”. | `Core/Src/Mpu6050.c:101–108`. |
| “DLPF em 44 Hz” | Especificar 44 Hz para o acelerômetro e 42 Hz para o giroscópio, com `CONFIG=0x03`. É atenuação de vibrações/ruído, sem garantia de eliminar todo aliasing. | `Core/Src/Mpu6050.c:64–65`; mapa oficial de registradores, seção 4.3. |
| “Driver sem dependências externas” | Usar “driver próprio, sem biblioteca gráfica externa”; há dependência da BSP/HAL, da biblioteca matemática e da biblioteca C. | `Core/Src/Ssd1306.c:13–18`. |
| “Círculo delimitador”, raio 18 | Não há círculo desenhado. O valor 18 define a meia extensão do segmento móvel. | `Core/Src/Ssd1306.c:247–267`. |
| “Clamping do pitchOffset” | O código limita separadamente os dois Y das extremidades a 2…62. O deslocamento não é limitado diretamente; o segmento pode se deformar nos limites. Pitch positivo desloca a linha para baixo, pois Y cresce para baixo. | `Core/Src/Ssd1306.c:254–265`. |

### Orçamento temporal verificável pelo código

Uma atualização completa envia 1.024 bytes de framebuffer. Só esses bytes, com ACK, exigem `1024 × 9 / 400000 = 23,04 ms`.

O driver também envia três comandos por página e os bytes de endereço/controle de cada transação. Considerando oito páginas:

```text
8 × [3 × (endereço + controle + comando)
     + (endereço + controle + 128 dados)] × 9
= 10.008 pulsos SCL
= 25,02 ms a 400 kHz
```

Esse valor exclui START/STOP, leitura da IMU, renderização e overhead de software. Assim, nove iterações de pelo menos 10 ms mais uma atualização de pelo menos 25,02 ms dão limites ideais de aproximadamente 86,9 iterações/s e 8,7 quadros/s. São estimativas superiores para operação normal, não medições. A 100 quadros/s, apenas o payload exigiria 230,4% do barramento; a justificativa para reduzir a frequência do OLED é válida, mas a redução não elimina o bloqueio periódico.

### Inicialização e tratamento de falhas

- A verificação de `WHO_AM_I == 0x68` existe. Porém, `Mpu6050_Init` ignora os retornos das cinco escritas de configuração e pode indicar sucesso mesmo se alguma falhar (`Core/Src/Mpu6050.c:51–74`). Descrever os valores como configurações solicitadas, sem afirmar confirmação por leitura de retorno.
- `Ssd1306_Init` retorna sempre `true`; comandos e atualizações ignoram o resultado das transmissões (`Core/Src/Ssd1306.c:111–147,155–163,307–309`). O teste de `oledOk` em `main` não comprova detecção de OLED ausente.
- Após erro de leitura da IMU, o filtro não é atualizado naquela iteração, mas a tela pode exibir a última amostra. O LED continua sendo alternado nas atualizações do dashboard (`Core/Src/main.c:64–76`). Portanto, ele indica atividade do laço, não validade das medições.

## 3. Afirmações confirmadas

- I2C1 configurado a 400 kHz, PB6/SCL e PB7/SDA; LED PC13 para `STM32F411xE` (`Core/Inc/Bsp.h:30–42`; `Core/Src/Bsp.c:139–159`). Isso comprova configuração em software, não frequência elétrica ou cabeamento medidos.
- Endereços de 7 bits: MPU-6050 `0x68` e SSD1306 `0x3C`. Os headers usam deslocamento de um bit para a API HAL (`Core/Inc/Mpu6050.h:20`; `Core/Inc/Ssd1306.h:22`).
- Separação em BSP, drivers, filtro e aplicação. A redação “estilo inspirado nas diretrizes ASSERT” coincide com os comentários; não há evidência de certificação formal de um padrão ASSERT.
- PLL referenciado ao giroscópio X (`PWR_MGMT_1=0x01`), divisor 9 e DLPF habilitado configuram nominalmente a saída do sensor a 100 Hz, se as escritas forem bem-sucedidas. Essa taxa interna difere da taxa efetiva de leitura do superloop.
- Burst de 14 bytes a partir de `0x3B`: aceleração XYZ, temperatura e giroscópio XYZ. Reconstrução Big-Endian em `s16` e escalas de 16.384 LSB/g e 131 LSB/(°/s) estão implementadas (`Core/Src/Mpu6050.c:84–110`).
- Framebuffer de 1.024 bytes, indexação por páginas de oito pixels verticais, Bresenham inteiro e fonte 5×7 `static const` (`Core/Src/Ssd1306.c:23,31–100,175,183–210`). A localização física da fonte em Flash depende do build/linker, não fornecidos neste checkout.
- Divisória em X=48, origem dos textos em X=52, centro gráfico (24,32), fator de Pitch 0,4 pixel/grau e seis linhas de telemetria coincidem com o plano (`Core/Src/Ssd1306.c:242–300`). Os ângulos são formatados sem casas decimais, embora armazenados como `float`.

## 4. Plano revisado para os dois slides

### Slide 1 — Demonstração de atitude e limites da estimativa

**Layout:** vídeo à esquerda, aproximadamente 60%; à direita, topologia, equações e um checklist curto. Explicações de implementação ficam nas notas de fala ou no segundo slide para preservar a legibilidade.

**Topologia:** STM32F411CEU6; I2C1 configurado a 400 kHz em PB6/PB7; MPU-6050 em 0x68 e SSD1306 em 0x3C; LED de atividade em PC13.

**Equações, com ângulos em graus e velocidades em °/s:**

```text
RollAcc  = atan2(Ay, Az) × 180/π
PitchAcc = atan2(-Ax, sqrt(Ay² + Az²)) × 180/π

Roll[k]  = 0,98 × (Roll[k−1]  + ωx × Δt) + 0,02 × RollAcc
Pitch[k] = 0,98 × (Pitch[k−1] + ωy × Δt) + 0,02 × PitchAcc
Yaw[k]   = Yaw[k−1] + ωz × Δt
```

No código atual, Δt=0,01 s fixo. Roll/Pitch recebem correção gravitacional; Yaw é relativo e sujeito a deriva.

**Checklist fiel à versão atual:** burst e conversão XYZ implementados; fusão Roll/Pitch e integração Yaw implementadas; animação Roll/Pitch implementada; pendentes exibição de ωx/ωy e representação gráfica de Yaw.

**Roteiro de vídeo de 30 segundos:**

| Tempo | Ação e observação |
| --- | --- |
| 00–05 s | Montagem em repouso, Z do sensor apontando para cima. Mostrar R/P próximos de 0°, Ax/Ay próximos de 0 g e Az próximo de +1 g. Yaw começa em zero na inicialização e pode derivar; não exigir zero exato. |
| 05–13 s | Rotacionar aproximadamente +90° e −90° em torno de X, com pequenas pausas nas posições. Mostrar Roll e a rotação da linha. Identificar o sinal pelos eixos do módulo, não apenas por “direita/esquerda”. |
| 13–21 s | Retornar à posição plana e rotacionar aproximadamente ±90° em torno de Y. Mostrar Pitch e deslocamento da linha até as bordas. Avisar que Roll pode oscilar próximo dessa orientação limite. |
| 21–30 s | Retornar ao plano, girar em torno de Z e mostrar W e a variação de Y. Terminar em repouso e observar a estabilização de R/P, sem afirmar erro zero ou ausência de deriva. |

O vídeo de 30 s é uma síntese visual. Para validar quantitativamente, usar posições de referência, pausas maiores e registrar valores/erros. A especificação permite até cinco minutos; não exige um vídeo de 30 s. Não usar um retorno brusco como prova de precisão: movimentos rápidos introduzem acelerações que contaminam temporariamente a referência do acelerômetro.

### Slide 2 — Arquitetura, aquisição e custo do OLED

1. **Organização modular:** BSP concentra HAL/hardware; drivers cuidam da IMU e OLED; filtro calcula a estimativa; `main` coordena o superloop sem RTOS explícito. Estilo de código inspirado no laboratório ASSERT.
2. **Aquisição e fusão:** burst de 14 bytes, conversão em g e °/s, α=0,98; sensor configurado nominalmente a 100 Hz. Não há sincronização por data-ready/FIFO, compensação de bias ou Δt real entre leituras.
3. **Taxas e limitação atual:** meta de período de 10 ms; atualização do OLED a cada dez iterações. O envio bloqueante de um quadro custa pelo menos cerca de 25 ms a 400 kHz, reduzindo as taxas efetivas. Apresentar 100/10 Hz como metas, não resultados medidos.
4. **Gráfico próprio:** framebuffer de 1.024 bytes, fonte 5×7 e Bresenham; Roll gira um segmento, Pitch desloca seus extremos, e o marcador da aeronave permanece fixo. Yaw aparece numericamente.

## 5. Roteiro de fala revisado para o OLED

“Dividimos a tela de 128 por 64 pixels na coluna 48. À esquerda, o centro do indicador fica em (24,32). Uma linha móvel, com meia extensão de 18 pixels, gira segundo o Roll. O Pitch adiciona um deslocamento de 0,4 pixel por grau, e limitamos os Y das duas extremidades entre 2 e 62. O marcador central fica fixo. Essa é uma representação simplificada; não desenhamos um círculo nem animamos Yaw nesta versão.

À direita, as três primeiras linhas mostram Roll, Pitch e Yaw sem casas decimais. Roll e Pitch combinam giroscópio e acelerômetro; Yaw é a integração da velocidade angular Z. As duas linhas seguintes mostram a aceleração XYZ em g, incluindo gravidade. A última mostra apenas ωz em graus por segundo. Para cumprir integralmente a especificação, ainda precisamos exibir ωx e ωy e acrescentar uma representação gráfica de Yaw.

A tela é atualizada a cada dez iterações. Embora a meta seja filtro a 100 Hz e tela a 10 Hz, a transmissão bloqueante do framebuffer ultrapassa o período de 10 ms. Por isso, as frequências reais e a precisão dinâmica precisam de medição e ajuste.”

## 6. Prioridades antes de declarar atendimento integral

1. Exibir os três eixos de velocidade angular, por exemplo com páginas alternadas ou revisão do layout.
2. Adicionar representação gráfica de Yaw, por exemplo um indicador de direção relativa; não apresentá-lo como bússola absoluta.
3. Medir o tempo entre leituras válidas e usá-lo na integração. Para sustentar aquisição regular a 100 Hz, reduzir ou tornar assíncrono o envio do OLED e considerar data-ready/FIFO; apenas corrigir Δt não recupera amostras perdidas.
4. Propagar falhas nas escritas da IMU e na inicialização/atualização do display; indicar dados obsoletos.
5. Validar em bancada sinais, escalas, estabilização, erros nos incrementos de 90°, tempos do laço e taxas reais. Calibrar bias e restringir as conclusões ao comportamento observado.

Referência externa usada para parâmetros do sensor: [InvenSense, MPU-6000/MPU-6050 Register Map and Descriptions, revisão 4.2](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf), especialmente seções 4.2 e 4.3. Para `CONFIG=3`, a tabela informa 44 Hz no acelerômetro e 42 Hz no giroscópio.
