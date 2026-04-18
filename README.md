# ACE - Autonomous Charging Environment

O projeto **ACE (Autonomous Charging Environment)** é um ecossistema de hardware e software desenhado para explorar o voo autónomo e sistemas de carregamento sem fios em drones. O sistema baseia-se numa arquitetura de comunicação de baixa latência entre uma aeronave (drone) e uma unidade de controlo (comando).



# 📂 Código do Drone (`M5StampFly/`)

## Estrutura do Código-Fonte Drone (`M5StampFly/src/`)

Esta secção descreve a função de cada módulo no firmware do drone, facilitando a navegação e o desenvolvimento.

### 🧠 Lógica de Controlo e Voo
* **`main.cpp`** O ponto de entrada do firmware. Inicializa todos os sistemas (`setup`) e executa o ciclo principal de 400Hz (`loop`).
* **`flight_control.cpp`** O "cérebro" do drone. Gere a máquina de estados (Voo, Aterragem, Flip) e mistura os comandos das manetes com os PIDs para calcular a potência de cada motor.
* **`pid.cpp`** Implementação da classe matemática do controlador **PID** e de filtros de sinal. Inclui proteção contra *windup* integral e suavização de derivadas.
* **`alt_kalman.cpp`** Filtro de Kalman para estimativa de altitude. Combina dados do acelerómetro com o sensor ToF para uma leitura de altura precisa e sem ruído.

### 📡 Comunicação e Telemetria
* **`rc.cpp`** Gere a receção de comandos via **ESP-NOW**. Interpreta os sinais enviados pelo comando rádio (manetes e botões).
* **`telemetry.cpp`** Envia dados do estado do drone em tempo real (bateria, ângulos, altitude) de volta para o comando, permitindo a monitorização durante o voo.

### ⚖️ Sensores e Fusão de Dados
* **`sensor.cpp`** Gestor central de sensores. Implementa o filtro **Madgwick** para calcular a orientação espacial (Roll, Pitch, Yaw) e monitoriza a saúde da bateria.
* **`imu.cpp`** Interface de baixo nível (SPI) com o sensor inercial **BMI270**. Configura e extrai os dados brutos de aceleração e rotação.
* **`tof.cpp`** Controlador para os sensores laser **VL53LX**. Mede a distância ao chão e deteta obstáculos frontais via I2C.

### 🛠️ Periféricos e Feedback
* **`led.cpp`** Controlo visual através de LEDs RGB (biblioteca **FastLED**). Indica modos de voo, avisos de bateria e estado da ligação.
* **`buzzer.cpp`** Gere alertas sonoros e melodias de sistema para feedback auditivo ao utilizador.
* **`button.cpp`** Gere as pressões no botão físico do drone (cliques curtos ou longos) para funções como o reinício do sistema.

## 📡 Sensores a Bordo

O drone ACE utiliza um conjunto robusto de sensores para garantir estabilidade, perceção espacial e gestão de energia. A lógica destes sensores está modularizada nos seguintes ficheiros:

### 1. IMU (Unidade de Medida Inercial)
* **Sensor:** Bosch BMI270 (Acelerómetro e Giroscópio).
* **Função:** Medir taxas de rotação e forças de aceleração para manter o drone nivelado e estável.
* **Ficheiros associados:** `src/imu.cpp` e `src/imu.hpp`.

### 2. ToF (Sensores de Distância Laser Time-of-Flight)
* **Sensor:** STMicroelectronics VL53LX (x2).
* **Função:** * **Inferior (`ToF_bottom`):** Mede a altitude exata em relação ao solo para controlo de altura e aterragem automática.
  * **Frontal (`ToF_front`):** Mede a distância a objetos à frente do drone para evitar obstáculos.
* **Ficheiros associados:** `src/tof.cpp` e `src/tof.hpp`.

### 3. Monitorização de Energia
* **Sensor:** Texas Instruments INA3221.
* **Função:** Monitorizar a tensão da bateria em tempo real para ativar sistemas de segurança de baixa voltagem (aviso via LEDs e telemetria).
* **Ficheiros associados:** Integrado em `src/sensor.cpp`.

### 🧠 Fusão de Dados (Sensor Hub)
* **Ficheiros associados:** `src/sensor.cpp` e `src/sensor.hpp`.
* **Função:** Atua como o núcleo de processamento sensorial. Recolhe os dados brutos de todos os sensores acima, aplica filtros passa-baixo e utiliza o algoritmo **Madgwick AHRS** para fundir os dados e calcular a orientação precisa (Roll, Pitch, Yaw) que será alimentada ao controlador de voo.


## 📋 TODO - Roadmap de Desenvolvimento (Drone)

Para atingir o objetivo do projeto ACE (voo autónomo num percurso predefinido e retorno automático para carregamento), as seguintes tarefas estão planeadas para o firmware do drone.

### 1. Integração de Acelerómetro Externo (Opcional)
**Objetivo:** Adicionar e validar um acelerómetro externo I2C (M5Stack ACCEL de 3 Eixos) para redundância ou maior precisão, caso o BMI270 interno não seja suficiente para a navegação por estimação (*dead reckoning*).
* **O que fazer:** * Configurar a comunicação I2C no barramento existente para o endereço do novo acelerómetro.
  * Ler os eixos X, Y e Z e aplicar filtros passa-baixo aos dados brutos.
  * Comparar/integrar os dados com o sistema atual (filtro Madgwick).
* **Ficheiros a alterar:** * Criar `src/accel_externo.cpp` e `.hpp` (recomendado para manter o código limpo).
  * Editar `src/sensor.cpp` (para inicializar no barramento I2C e integrar os dados na rotina `sensor_read()`).

### 2. Deteção de Tensão Crítica (Battery Threshold)
**Objetivo:** Monitorizar continuamente a bateria e acionar o modo de regresso e aterragem assim que a tensão cair abaixo de um limite seguro (`V_threshold`), indicando a necessidade de carregamento por indução.
* **O que fazer:**
  * O sensor INA3221 já está integrado. É preciso definir constantes claras para o `V_threshold`.
  * Criar uma lógica no ciclo principal: `if (Voltage < V_threshold) { Mode = RETURN_TO_BASE_MODE; }`.
* **Ficheiros a alterar:**
  * `src/flight_control.hpp` (para adicionar as constantes do limiar de tensão e o novo estado `RETURN_TO_BASE_MODE`).
  * `src/flight_control.cpp` (na máquina de estados principal, forçar a mudança de modo quando a bateria falha).

### 3. Rotina de Voo Autónomo (Ex: Trajeto em Quadrado)
**Objetivo:** Permitir que o drone percorra um trajeto pré-definido (ex: um quadrado de 2x2 metros) de forma autónoma.
* **O que fazer:**
  * **Navegação por Estima (Dead Reckoning):** Usar os dados do acelerómetro (integração dupla da aceleração para calcular deslocamento) e do IMU (Pitch/Roll) para estimar a distância percorrida.
  * **Máquina de Estados de Trajeto:** Criar uma lógica de "Waypoints" (Pontos de Passagem). Ex: Avançar 2m -> Virar 90º Yaw -> Avançar 2m, etc.
  * Injetar ângulos de referência (`Pitch_angle_reference`, `Roll_angle_reference`) nos PIDs no lugar dos comandos humanos.
* **Ficheiros a alterar:**
  * `src/flight_control.cpp` (Criar a função de controlo de posição autónoma e substituir a entrada do rádio).
  * `src/sensor.cpp` (Para processar a integração matemática da aceleração em distância percorrida).

### 4. Protocolo e Lógica de Comunicação (ESP-NOW)
**Objetivo:** Expandir a comunicação atual entre o comando e o drone. O drone tem de aceitar ordens como "Iniciar Percurso Autónomo" e "Regressar à Base", e enviar o seu estado (ex: "Bateria Fraca, a iniciar RTL").
* **O que fazer:**
  * Estudar a estrutura atual de pacotes recebidos (`OnDataRecv`).
  * Adicionar novos "comandos" virtuais no array `Stick` que o comando envia. (Ex: `Stick[START_AUTON_MODE]`).
  * O drone também terá de enviar alertas específicos via telemetria para o comando.
* **Ficheiros a alterar:**
  * `src/rc.cpp` e `src/rc.hpp` (Mapear novos índices para os pacotes de dados).
  * `src/telemetry.cpp` (Para avisar o comando das decisões tomadas de forma autónoma).
  * *(Nota: Isto também implicará alterações significativas no código do Comando para enviar as rotinas).*



# 🎮 Código do Comando / Controlador (`Atom-JoyStick/`)

## Estrutura do Código-Fonte Comando (`src/` e `examples/`)

Esta secção descreve a função de cada módulo no firmware do comando remoto, responsável por interagir com o utilizador, processar os inputs físicos e enviar comandos precisos para o drone.

### 🧠 Lógica Principal e Comunicação
* **`main.cpp`** O "cérebro" do comando remoto. Coordena o sistema inteiro: gere o emparelhamento e comunicação de baixa latência (**ESP-NOW**) com o drone, processa as máquinas de estados dos modos de voo, converte a posição mecânica do joystick nos sinais de voo (Pitch, Roll, Yaw, Throttle) e gere tarefas paralelas (*multithreading* com FreeRTOS).

### 🕹️ Hardware e Sensores de Input
* **`AtomJoyStick.cpp`** Driver de comunicação I2C dedicado ao módulo M5Stack Atom Joystick. Encarrega-se de ler os valores analógicos exatos (em 8 ou 12-bit) dos quatro eixos do comando, regista os cliques nos botões físicos integrados e monitoriza a tensão elétrica (voltagem) das duas baterias do comando 18650.
* **`GetValue.ino`** (Pasta de exemplos) Código de diagnóstico e demonstração. É utilizado isoladamente para calibrar e testar o hardware, imprimindo as leituras cruas dos eixos e baterias diretamente para o monitor série.

### 🖥️ Interface Gráfica (UI) e Ecrã
* **`app_lvgl.cpp`** Motor visual do comando. Utiliza a biblioteca gráfica **LVGL** para desenhar e gerir os dois ecrãs de interface (o menu de Setup e o painel de Voo). É aqui que são processadas as animações dos mostradores (como o horizonte artificial) e a atualização em tempo real da telemetria recebida do drone (bateria, altitude, aviso de radar frontal).
* **`lvgl_porting.cpp`** A ponte (*driver*) entre a interface gerada pelo LVGL e o ecrã físico do M5AtomS3. Transfere os píxeis calculados para o display (via biblioteca M5GFX), traduz cliques nos botões físicos para interações táteis virtuais e gere o "relógio" interno (Tick) numa tarefa isolada para garantir que o ecrã atualiza a alta frequência sem encravar o rádio.

### 🔊 Feedback e Alertas
* **`buzzer.cpp`** Gestor de áudio via PWM (LEDC). Controla o altifalante interno para fornecer feedback tátil e de sistema ao piloto. Inclui funções para *beeps* de clique, melodias de arranque da UI e um alerta sonoro especializado que avisa quando a bateria atinge a voltagem exata ideal para armazenamento prolongado (*Storage Voltage*).


## 📋 TODO - Roadmap de Desenvolvimento (Comando / Controlador)

Para o comando acompanhar as novas capacidades autónomas do drone ACE, teremos de expandir a sua interface, os alertas e o protocolo de comunicação.

### 1. Atualização do Protocolo de Comunicação (ESP-NOW)
**Objetivo:** Permitir que o comando envie as ordens de "Iniciar Voo Autónomo" e "Regressar à Base" (RTL), e seja capaz de interpretar as novas mensagens de estado enviadas pelo drone.
* **O que fazer:**
  * Adicionar novos índices virtuais aos dados transmitidos (ex: uma flag específica para ativar o quadrado e outra para forçar a aterragem na base).
  * Descodificar os novos dados de telemetria recebidos (ex: ler se o drone entrou em modo RTL automaticamente devido à bateria).
* **Ficheiros a alterar:**
  * `src/main.cpp` (Para alterar a construção do pacote de dados (o array de comandos) enviado para o drone e a leitura do pacote de telemetria recebido).

### 2. Mapeamento de Botões para Modos Autónomos
**Objetivo:** Atribuir ações físicas aos novos comandos do projeto para que o piloto possa acionar as rotinas com o Joystick.
* **O que fazer:**
  * Escolher um botão físico (ou criar uma combinação de botões / pressão longa) no módulo Atom Joystick para acionar a rotina do quadrado.
  * Escolher/Mapear um botão de emergência para forçar o "Return to Base" a qualquer momento.
* **Ficheiros a alterar:**
  * `src/AtomJoyStick.cpp` (Para ler a nova combinação/botão, se necessário).
  * `src/main.cpp` (Para traduzir esse clique na variável que será enviada via ESP-NOW).

### 3. Atualização da Interface Gráfica (Ecrã LVGL) (Totalmente Opcional (só fazer no fim))
**Objetivo:** Informar visualmente o piloto de que o drone assumiu o controlo e mostrar alertas críticos.
* **O que fazer:**
  * Adicionar indicadores visuais no ecrã de voo (ex: Mudar um texto para "AUTO" ou "RTL" com cores chamativas quando os modos estiverem ativos).
  * Criar um alerta visual crítico (ex: um *pop-up* ou piscar o ecrã a vermelho) quando a bateria do drone atingir o `V_threshold` e a aterragem de emergência começar.
* **Ficheiros a alterar:**
  * `src/app_lvgl.cpp` (Para desenhar e animar os novos elementos visuais da interface baseados nas variáveis atualizadas no `main.cpp`).

### 4. Alertas Sonoros Críticos (Buzzer) (Totalmente Opcional (só fazer no fim))
**Objetivo:** Garantir que o piloto percebe que o drone está com bateria fraca ou a regressar, mesmo que não esteja a olhar para o ecrã.
* **O que fazer:**
  * Criar novos padrões de beeps (ex: 3 beeps rápidos para confirmar que a rotina do quadrado começou; um alarme contínuo para "Bateria Crítica / A Iniciar Regresso").
  * Chamar estes beeps no ciclo principal quando as flags de telemetria correspondentes dispararem.
* **Ficheiros a alterar:**
  * `src/buzzer.cpp` (Para criar as novas funções de padrões sonoros).
  * `src/main.cpp` (Para chamar as funções do buzzer na lógica de telemetria).