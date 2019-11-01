# Home Guardian PT

## CubeMXの設定

- ACCESS TO MCU SELECTOR
- STM32F446REを選択
- Start Projectをクリック

### 1. Pinout & Configuration

**SYS**

JTAG (5 pins)

System Wake-Up 0

**TIM1**

- Clock Source: Internal Clock
- Channel1: PWM Generation CH1
- Channel2: PWM Generation CH2

Parameter Settings

- Prescaler: 89
- Counter Period: 999

**TIM2**

- Combined Channels: Encoder Mode

Parameter Settings

- Counter Period: 65535
- Encoder Mode: Encoder Mode TI1 and TI2

**TIM3**

- Combined Channels: Encoder Mode

Parameter Settings

- Counter Period: 65535
- Encoder Mode: Encoder Mode TI1 and TI2

**TIM7**

- Activated

Parameter Settings

- Prescaler (PSC - 16 bits value): 899
- Counter Period: 999
- auto-reload preload: Enable

NVIC Settings

- TIM7 global interrupt: Enabled


**I2C1**

- I2C: I2C

**USART2**

- Mode: Asynchronous
- Baud Rate: 115200

**GPIO**

Left

- PC0: GPIO_Input: SW1
- PC1: GPIO_Input: SW2
- PC2: GPIO_Output: MD_STBY
- PC3: GPIO_Input: BAT_VOL

Bottom

- PC5: GPIO_Output: MUX_RESET

Right

- PB13: GPIO_Output: MUX1
- PB14: GPIO_Output: MUX2
- PB15: GPIO_Output: MUX3
- PC6: GPIO_Output: DRL_IN1
- PC7: GPIO_Output: DRL_IN2
- PC8: GPIO_Output: DRR_IN1
- PC9: GPIO_Output: DRR_IN2
- PA10: GPIO_Output: LED_BAT

### 2. Clock Configuration

HCLKが180MHzとなるように設定

- PLLM: /8
- PLLN: x180
- System Clock Mux: PLLCLK
- APB1 Prescaler: /4
- APB2 Prescaler: /4

### 3. Project Manager


Project ManagerタブのFirmware Package Name and Versionで以下を選択する。

- Toolchain / IDE: SW4STM32
- User latest available version (STM32Cube FW_F4 V1.24.1)
- Use Default Firmware Location

## SW4STM32の設定

### CubeMXでGenerate Codeを実行した後の操作

CubeMXはCでコードを生成するので、SW4STM32にてC++に変換する必要がある。

#### 手順
 - Project ExplorerでHGPTを右クリックし、Clean Projectを実行
 - Project ExplorerでHGPTを右クリックし、Convert to C++を実行
 - main.cをmain.cppに変換

 
まず空のプロジェクトを作成してGenerate Code、SW4STM32で書き込み。
次にピン設定してGenerate Code、C++に変換して書き込み。
 