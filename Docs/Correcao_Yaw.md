# Correção de erro acumulado na seta — 10/11/2026

> Registro da versão de referência preservada em `Backups/versao-quaternion-estavel-20261110.tar.gz`. O painel e as proteções de inversão posteriores estão descritos em `Painel_Voo.md`.

## Diagnóstico e alteração

A seta usa um pivô fixo em (24,51). A versão anterior integrava `gyroZDps` diretamente como Yaw, enquanto calculava Roll e Pitch separadamente. Quando o sensor está inclinado, a velocidade angular no seu eixo Z não coincide, em geral, com a taxa de Yaw no mundo. Rotações combinadas, mesmo com retorno à atitude inicial, podiam acumular um erro geométrico além da deriva física do giroscópio.

O filtro foi substituído por um filtro complementar não linear em quaternion:

- Integração conjunta dos três eixos, por multiplicação de incrementos de rotação e normalização do quaternion.
- Correção proporcional pela gravidade, com ganho `-ln(0,98)/0,01`, preservando aproximadamente a constante de tempo nominal para pequenos erros. A correção fica suspensa se a norma do acelerômetro estiver fora de 0,85…1,15 g.
- Inicialização de inclinação pelo acelerômetro e referência inicial de Yaw igual a zero.
- Extração de Roll/Pitch/Yaw a partir da mesma orientação tridimensional, em vez de integrar os três ângulos independentemente.
- Quando o eixo de referência aponta quase na vertical, Roll/heading de Euler ficam indeterminados: a UI mostra `R:--`, `Y:--`, `Y VERTICAL` e `?` no lugar da seta. A integração interna continua. Há histerese para evitar alternância rápida nessa região.

Referência da formulação de correção gravitacional: [Mahony, Hamel e Pflimlin, Nonlinear Complementary Filters on the Special Orthogonal Group (2008)](https://researchportalplus.anu.edu.au/en/publications/nonlinear-complementary-filters-on-the-special-orthogonal-group/), DOI 10.1109/TAC.2008.923738. A implementação usa apenas realimentação proporcional e a calibração inicial de bias; não inclui magnetômetro nem estimação integral de bias.

Yaw absoluto permanece não observável com esta IMU: a correção não inventa uma referência de Norte, não força o ângulo a zero em repouso e não elimina bias residual, saturação ou amostras perdidas.

## Testes

`make test` passou com avisos tratados como erro, incluindo:

- Sensor inclinado em Roll=90° e giro em torno de seu Y: resulta corretamente em Yaw de 90° no mundo, mesmo com velocidade Z do sensor nula.
- Vinte trajetórias fechadas em três eixos, cada uma com `Rx(+90°), Ry_corpo(+90°), Rx_corpo(-90°), Rz_corpo(-90°)`. Após cada ciclo, os três ângulos voltam a menos de 0,1° da identidade em dados ideais simulados. O integrador anterior somava −90° de Yaw por ciclo nessa trajetória.
- Δt variável, convergência da correção gravitacional, travessia de ±180°, rejeição de aceleração evidente de 2 g e quaternion finito ao atravessar Pitch=90°.
- UI não desenha uma seta dependente do ângulo quando o heading está marcado como inválido.
- Regressões de comunicação, calibração, seis eixos e envio incremental do OLED.

Esses testes validam a geometria e o software; não são ensaios físicos de precisão.

## Gravação e execução na placa

O build ARM passou e a nova imagem foi gravada no STM32F411, com `Verified OK` em `build/quaternion-flash.log`. Assinatura de diagnóstico: `0x49324333`.

Em `build/quaternion-runtime.log`, os contadores em execução registraram **502 leituras válidas e 50 quadros em 5.019 ms**, aproximadamente 100,0 Hz e 10,0 Hz. Erros de IMU/OLED e prazos perdidos permaneceram em zero; o maior intervalo entre leituras desde aquela inicialização foi 11 ms. A calibração foi aceita na primeira tentativa daquela inicialização.

As normas dos quaternions nas duas capturas foram 1,000000077 e 1,000000028. O framebuffer em `build/quaternion-display.png` foi inspecionado visualmente. Isso confirma execução e representação coerentes, sem demonstrar retorno físico preciso a uma direção de referência.

SHA-256 da imagem atual: `14ba156b9afb4ad0533652ece999e12f093923fa4fad4baf8971f83b977a226b`.

Uso do build: 33.856 bytes de Flash (text + data); BSS de 6.700 bytes, incluindo as reservas do linker. A imagem anterior à correção foi preservada em `Backups/stm32f411-before-quaternion-20261110.bin`; o backup original anterior a todas as alterações continua preservado.

Para testar fisicamente: reinicie com a placa parada, aguarde terminar a calibração, faça rotações combinadas suaves e retorne a uma referência de direção marcada na bancada. Compare o erro antes/depois e observe separadamente eventual deriva em repouso. Perto da vertical, `?` indica heading indefinido, não falha de comunicação.
