# Home Guardian PT

## CubeMXの設定

### 1. Pinout & Configuration

**SYS**

JTAG (5 pins)

System Wake-Up 0

**TIM1**

- Clock Source: Internal Clock
- Channel1: PWM Generation CH1
- Channel2: PWM Generation CH2
- Prescaler: 89
- Counter Period: 999

**TIM2**

- Combined Channels: Encoder Mode
- Counter Period: 65535
- Encoder Mode: Encoder Mode TI1 and TI2

**TIM3**

- Combined Channels: Encoder Mode
- Counter Period: 65535
- Encoder Mode: Encoder Mode TI1 and TI2

**TIM7**

- Activated
- TIM7 global interrupt: Enabled
- Prescaler (PSC - 16 bits value): 899
- Counter Period: 999
- auto-reload preload: Enable

**I2C1**

- I2C: I2C

**USART2**

- Mode: Asynchronous
- Baud Rate: 115200

**GPIO**

Left

- PC0: SW1
- PC1: SW2
- PC2: MD_STBY
- PC3: BAT_VOL

Bottom

- PC5: MUX_RESET

Right

- PB13: MUX1
- PB14: MUX2
- PB15: MUX3
- PC6: DRL_IN1
- PC7: DRL_IN2
- PC8: DRR_IN1
- PC9: DRR_IN2
- PA8: PWM_L(TIM1_CH1)
- PA9: PWM_R(TIM1_CH2)
- PA10: LED_BAT

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
 