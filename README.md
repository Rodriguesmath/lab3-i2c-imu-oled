# Lab 3: Fusão Sensorial e Estimação de Pose (MPU-6050 + OLED SSD1306 via I²C)
**Formação em Sistemas Embarcados Virtus-CC — Curso 2: Periféricos**  
**Prof. D.Sc. Rafael Bezerra Correia Lima**

---

> [!NOTE]
> **Estilo de Codificacao:** A arquitetura de firmware, nomenclatura de identificadores, utilizacao de BSP, struct de dados locais e padronizacao Doxygen deste repositorio foram inspiradas nas diretrizes de boas praticas do **Laboratorio ASSERT (IFPB)**.

---

## 1. Descricao da Solucao

Este projeto realiza a **estimacao da pose tridimensional (Roll, Pitch e Yaw)** de uma aeronave utilizando **fusao sensorial** entre acelerometro triaxial e giroscopio triaxial (IMU MPU-6050, 6-DOF). Os dados sao processados por um **filtro complementar** em tempo real e apresentados em um display OLED SSD1306 de 128x64 pixels, incluindo uma **animacao gráfica de horizonte artificial** (requisito avancado).

### Requisito Avançado Atendido: Animação Gráfica do Horizonte Artificial
Conforme solicitado nos requisitos avançados do roteiro:
1. **Representação Gráfica Contínua:** Além de exibir os valores numéricos de aceleração linear, velocidade angular e ângulos de Euler (Roll, Pitch e Yaw), o display OLED renderiza um **Horizonte Artificial Aeronáutico (*Attitude Indicator*)**.
2. **Dinâmica do Horizonte:**
   - A linha de referência inclina proporcionalmente ao ângulo de **Roll** ($\phi$).
   - O centro de referência desloca-se verticalmente proporcionalmente ao ângulo de **Pitch** ($\theta$).
   - Um marcador central em cruz (*crosshair*) simboliza a aeronave/placa em relação ao solo.
3. **Taxa de Quadros Suave (10 FPS):** Atualização periódica a 10 Hz desacoplada da taxa do filtro (100 Hz), permitindo animação fluida sem sobrecarregar o barramento I²C.

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

### 2. Equacao de Atualizacao do Filtro Complementar ($\alpha = 0,98$):
$$\text{Roll}[k] = \alpha \cdot \left(\text{Roll}[k-1] + \omega_x \cdot \Delta t\right) + (1 - \alpha) \cdot \text{Roll}_{\text{acc}}$$
$$\text{Pitch}[k] = \alpha \cdot \left(\text{Pitch}[k-1] + \omega_y \cdot \Delta t\right) + (1 - \alpha) \cdot \text{Pitch}_{\text{acc}}$$
$$\text{Yaw}[k] = \text{Yaw}[k-1] + \omega_z \cdot \Delta t$$

---

## 4. Roteiro de Testes em Incrementos de 90 Graus

1. Mantenha a placa perfeitamente plana sobre a bancada: Roll e Pitch devem marcar aproximadamente $0^\circ$, e o eixo Z deve marcar $+1.0\text{ g}$.
2. Incline a montagem $90^\circ$ para a direita: Roll deve atingir $+90^\circ$ e a linha do horizonte na tela inclina em $90^\circ$.
3. Incline $90^\circ$ para a esquerda: Roll atinge $-90^\circ$.
4. Aponte o nariz da placa $90^\circ$ para cima: Pitch atinge $+90^\circ$.
5. Aponte $90^\circ$ para baixo: Pitch atinge $-90^\circ$.
6. Gire a placa horizontalmente sobre a mesa: O angulo de Yaw responde proporcionalmente a velocidade angular $\omega_z$.
