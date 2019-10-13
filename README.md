# Home Guardian PT

## CubeMXの設定

Project ManagerタブのFirmware Package Name and Versionで以下を選択する。

STM32Cube FW_F4 V1.24.0

## SW4STM32の設定

### CubeMXでGenerate Codeを実行した後の操作

CubeMXはCでコードを生成するので、SW4STM32にてC++に変換する必要がある。

#### 手順
 - Project ExplorerでHGPTを右クリックし、Clean Projectを実行
 - Project ExplorerでHGPTを右クリックし、Convert to C++を実行
 - main.cをmain.cppに変換
 