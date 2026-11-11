# Lab 3: Fusão Sensorial e Estimação de Pose (MPU-6050 + OLED SSD1306 via I²C)
**Formação em Sistemas Embarcados Virtus-CC — Curso 2: Periféricos**  
**Prof. D.Sc. Rafael Bezerra Correia Lima**

---

> [!NOTE]
> **Estilo de Codificacao:** A arquitetura de firmware, nomenclatura de identificadores, utilizacao de BSP, struct de dados locais e padronizacao Doxygen deste repositorio foram inspiradas nas diretrizes de boas praticas do **Laboratorio ASSERT (IFPB)**.

---

## 1. Descricao da Solucao

Este projeto realiza a **estimacao da pose tridimensional (Roll, Pitch e Yaw)** de uma aeronave utilizando **fusao sensorial** entre acelerometro triaxial e giroscopio triaxial (IMU MPU-6050, 6-DOF). Os dados sao processados por um **filtro complementar** em tempo real e apresentados em um display OLED SSD1306 de 128x64 pixels, incluindo uma **animacao gráfica de horizonte artificial** (requisito avancado).

### Painel de voo no OLED

O painel permanece visível continuamente em 128×64 pixels:

- **Centro:** horizonte artificial com céu escuro, solo pontilhado e símbolo fixo da aeronave. O desenho usa diretamente a gravidade prevista pelo quaternion, acompanhando inclusive a atitude invertida.
- **Laterais:** Roll e Pitch em graus; NZ (aceleração no eixo Z do sensor em g) e WZ (velocidade angular Z em °/s).
- **Parte inferior:** fita circular de rumo relativo, 000…359, com indicação central `000R`. O sufixo R indica referência relativa à inicialização, sem bússola absoluta.
- **Parte superior:** alternância a cada 2 segundos entre AX/AY/AZ em g e WX/WY/WZ em °/s. As unidades aparecem no centro inferior como `A G` ou `W D/S`.

`INV` identifica atitude invertida. Perto da vertical, `VERT` e os campos de Roll/rumo sem valor indicam a ambiguidade dos ângulos de Euler; o horizonte e o quaternion continuam atualizando. Uma recuperação de inclinação trata o equilíbrio antipodal após uma inversão não adquirida, sem forçar o Yaw a zero.

![Prévia do painel nas diferentes atitudes](Docs/Preview_Painel_Voo.png)

Na inicialização, `KEEP STILL - CAL` solicita que a montagem fique parada por aproximadamente 2 segundos. A calibração estima o bias dos três eixos do giroscópio; movimento detectado ou erro de leitura produzem `CAL RETRY - STILL` e uma nova tentativa. Os offsets são recalculados a cada reinicialização.

O superloop prioriza a leitura da IMU a cada 10 ms, mede o Δt real e envia o framebuffer congelado uma página de 128 bytes por etapa, quando restam pelo menos 6 ms para a próxima aquisição. A meta de atualização do painel é 10 quadros/s.

Falhas de aquisição produzem `IMU ERROR / DATA STALE`. O LED alterna a cada 500 ms em operação normal e a cada 100 ms quando há erro. A integração não extrapola intervalos com falhas. Consulte [o painel e sua validação](Docs/Painel_Voo.md).

---

## 2. Pinagem de Conexão

| Módulo / Periférico | Black Pill (F411) | STM32H723 | Nucleo-F767ZI | Conexão / Função |
| :--- | :---: | :---: | :---: | :--- |
| **VCC / VDD**        | `3.3V` | `3.3V` | `3.3V` | Alimentação dos módulos |
| **GND**              | `GND`  | `GND`  | `GND`  | Referência de terra comum |
| **I2C SCL**          | `PB6`  | `PB6`  | `PB6`  | `I2C1_SCL` (Clock com pull-up de 4.7k) |
| **I2C SDA**          | `PB7`  | `PB7`  | `PB7`  | `I2C1_SDA` (Dados com pull-up de 4.7k) |
| **MPU-6050 AD0**     | `GND`  | `GND`  | `GND`  | Fixa o endereço I²C do MPU em `0x68` |
| **LED Heartbeat**    | `PC13` | `PC13` | `PB0`  | Indicador visual de ciclo de leitura |

### Diagrama Esquemático do Circuito
![Esquemático I2C MPU6050 e OLED](Docs/esquematico_circuito.png)

> O arquivo vetorial completo está disponível em: [Esquematico_Placa3_I2C_MPU6050_OLED.pdf](Docs/Esquematico_Placa3_I2C_MPU6050_OLED.pdf)

---

## 3. Equacoes de Fusao Sensorial (Filtro Complementar)

### 1. Angulos calculados pelo Acelerometro:
$$\text{Roll}_{\text{acc}} = \text{atan2}(A_y, A_z) \times \frac{180}{\pi}$$
$$\text{Pitch}_{\text{acc}} = \text{atan2}\left(-A_x, \sqrt{A_y^2 + A_z^2}\right) \times \frac{180}{\pi}$$

### 2. Atualização em quaternion:

O firmware integra as três velocidades angulares de forma acoplada. Isso evita o erro de tratar a velocidade no eixo Z do sensor como taxa de Yaw quando a placa está inclinada. A referência de gravidade corrige a inclinação por um filtro complementar não linear, com realimentação proporcional:

$$\hat{g}_b = [2(q_xq_z-q_wq_y),\ 2(q_yq_z+q_wq_x),\ 1-2(q_x^2+q_y^2)]$$
$$e = \frac{a}{\|a\|}\times\hat{g}_b,\qquad K_p = -\frac{\ln(0,98)}{0,01}$$
$$\omega_c = \omega\frac{\pi}{180} + K_p e,\qquad \beta=\frac{\|\omega_c\|\Delta t}{2}$$
$$\delta q = \left[\cos\beta,\ \frac{\sin\beta}{\|\omega_c\|}\omega_c\right],\qquad q_k = \operatorname{normalize}(q_{k-1}\otimes\delta q)$$

Para velocidade nula, o fator seno usa seu limite Δt/2. Roll/Pitch/Yaw são extraídos do quaternion; os ângulos do acelerômetro acima são usados no alinhamento inicial. A correção por gravidade só é aplicada quando a norma medida fica entre 0,85 e 1,15 g, reduzindo a influência de acelerações externas evidentes. O ganho nominal corresponde a aproximadamente 0,5 s de constante de tempo para pequenos erros.

A formulação é inspirada no [filtro complementar explícito de Mahony, Hamel e Pflimlin (2008)](https://researchportalplus.anu.edu.au/en/publications/nonlinear-complementary-filters-on-the-special-orthogonal-group/), usando correção proporcional e a calibração inicial de bias, sem magnetômetro ou termo integral de bias.

Yaw continua relativo à inicialização e sujeito à deriva residual. Próximo de Pitch ±90°, heading e Roll de Euler ficam indeterminados: o painel mostra `VERT` e oculta os valores de Roll e rumo. O quaternion continua sendo integrado e a indicação retorna ao sair dessa região, com histerese para evitar alternâncias rápidas. O filtro não força um retorno artificial do Yaw a zero.

---

## 4. Roteiro de Testes em Incrementos de 90 Graus

1. Após a calibração, mantenha o sensor com Z apontando para cima: Roll/Pitch próximos de 0°, Ax/Ay próximos de 0 g e Az próximo de +1 g.
2. Faça rotações separadas de aproximadamente ±90° em torno de X e observe Roll e a linha do horizonte. Defina os sinais pelos eixos do módulo instalado.
3. Retorne ao plano antes de fazer rotações de aproximadamente ±90° em torno de Y. Observe Pitch e o deslocamento do horizonte; Roll e heading ficam indeterminados próximo de Pitch ±90°; a fita de rumo ficará sem valor e será exibido `VERT`.
4. Retorne ao plano, gire em torno de Z e observe a fita de rumo relativo e WZ. Yaw tem referência na inicialização e pode derivar ao longo do tempo.
5. Aguarde a alternância da faixa superior e confira AX/AY/AZ e WX/WY/WZ. Pause nas posições de referência para comparar os ângulos com uma referência física.

## 5. Compilar, testar e gravar (STM32F411CEU6)

Requer Git, Make, compilador ARM GNU e OpenOCD com ST-LINK. O Makefile detecta automaticamente o compilador do STM32CubeIDE instalado em `/opt/st`; para outra instalação, informe `TOOLCHAIN=/caminho/arm-none-eabi-`.

```sh
sh Tools/fetch_dependencies.sh
make -j4
make test
make flash
```

As dependências oficiais têm versões e commits fixados no script e ficam em `Drivers/`, ignorado pelo Git. O build inclui startup CMSIS, linker para 512 KiB de Flash/128 KiB de RAM, SysTick e suporte ao heap da newlib-nano para formatação de floats. `make test` executa testes em C no computador, com comunicação simulada; não grava a placa. Se o compilador local tiver os runtimes de sanitizers, é possível usar `make test TEST_SANITIZERS=-fsanitize=address,undefined`.

Os executáveis são `build/firmware.elf` e `build/firmware.bin`. `make clean` remove a pasta `build/`, incluindo os logs. O backup preservado em `Backups/` permanece intacto. `make flash` grava e verifica o firmware, mas não cria backup automaticamente.

A gravação realizada em 10/11/2026 preservou a Flash anterior em `Backups/stm32f411-before-20261110.bin`. Consulte [o painel atual](Docs/Painel_Voo.md), [a correção do erro de Yaw](Docs/Correcao_Yaw.md) e [o registro da primeira validação](Docs/Validacao_Firmware.md) para evidências, versões e limites dos testes. A revisão anterior do plano em `Docs/Revisao_Plano_Apresentacao.md` descreve a versão histórica anterior às correções.
