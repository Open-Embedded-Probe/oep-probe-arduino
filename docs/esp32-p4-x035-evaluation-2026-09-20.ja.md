# ESP32-P4 / CH32X035 prototype評価（2026-09-20）

## 結論

現状のP4 prototypeは、X035の保全、書込み、全域照合と、RAMを介した最小限の実行確認には使用できる。
高速化後の別imageへの更新は全工程80.00秒で、初期実装の約13分から約9.7倍短縮した。一方、UARTと
fixture GPIOを公開していないため、ArduinoCore-CH32の既存selftestをこのprobeだけで完走する段階には
まだない。

## 実機確認

- probe固定名: `/run/board-identify/by-id/esp32-series-30eda0e31108`
- target: CH32X035F8U6、flash 63,488 byte
- 初期image保全SHA-256: `17ad3777ba42af0bd8d61ae5521ab5a4d5f10057e148d22b3fae5b8fbc235988`
- 診断imageの書込み後SHA-256: `5449097718257fb15ebcf7deb17fe039b9e75eac8de39f23e62d7ee583bb8bac`
- 全域read: 24.80秒、hash一致
- 109 logical pageの比較・program・全域verify・reset: 80.00秒、109 attempts、hash一致
- 元imageへの復元と全域verify: 成功

初回の診断ではRAM reportが更新されず、旧imageの値を新imageの結果と誤認した。原因は
`normalizeUser()`がresetではなく、旧imageをhaltしたPCへのdebug resumeだったことである。PFIC
system-reset payloadをRAMで実行するよう修正後、診断imageの`version=2`と実行phaseを確認した。
`delayMicroseconds(25000)`の前後は`millis()`が0から25、`micros()`が36から25,039となり、SysTick
compareも47,999（48 MHz / 1,000 - 1）だった。X035 coreの時刻基盤はこの試験では正常である。

## 良かった点

- 物理port番号に依存しない固定aliasで一貫して操作できる。
- 書込み前backupとSHA-256により、prototypeであっても元状態へ戻せる。
- 64-byte要求をX035の256-byte erase単位へread-modify-writeするため、隣接内容を保存できる。
- program失敗時の再送回数が結果に現れ、今回の109 pageはすべて初回成功した。

## 改善した点

- SWDIOはopen-drainのHIGHで解放できるため、各bitの`pinMode()`を除去した。
- 現fixtureでは追加half-periodを1 µsから0へ変更し、全域hash一致で信号成立を確認した。
- 96-byte OEP messageに収まるmemory readを32 byteから88 byteへ拡張した。
- byte数とpage数を混同していた進捗条件を、全体の1/16境界表示へ変更した。
- `normalizeUser()`をdebug resumeからPFIC system-reset payloadへ変更し、書換え後に必ず新imageの
  reset vectorとC runtimeを通るようにした。

## 残る使い勝手と機能の不足

1. P4 X035 exampleはTargetControl / TargetMemory / TargetFlashだけを宣言する。UART出力を読めないため、
   `serial_println`やtest command方式の既存suiteを判定できない。
2. PC15の1信号だけでは双方向UARTにならない。既定Serial1のPB10(TX) / PB11(RX)をP4のUARTへ配線し、
   FixtureUart capabilityとして公開する必要がある。
3. GPIO、ADC電圧印加、SPI/I2C slave、波形計測機能がないため、周辺機能の電気的検証はできない。
4. 型番、flash容量、保護状態を自動識別しない。誤targetへの破壊的操作を防ぐpreflightが必要である。
5. 64-byte要求ごとに256-byte物理pageを更新するため、同一物理pageを最大4回erase/programする。
   256-byte transactionまたはprobe内batch化が次の速度改善候補である。
6. 操作途中のprobe電源断ではRAM上の回復cacheが失われる。完全backupを用いた復元が必須である。
7. stableなmachine-readable resultがなく、Coreのupload recipeへ直接組み込むには早い。

## GPIO配線対応表（2026-09-21）

X035が各padに2〜45 Hzの固有矩形波を出し、P4のread-only GPIO bank snapshotを12秒間
（約595 sample/s）採取して求めた。PC18/PC19（RVSWD）とPC10/PC11（内部bondの非駆動側）は
意図的に駆動していない。ここで「未観測」はchipのpadではなく、このdevelopment boardのheaderへ
出ていないpadである。

| X035 board pad | ESP32-P4 GPIO | 備考 |
|---|---:|---|
| PA0, PA1, PA2, PA3, PA4 | 46, 47, 48, 49, 53 | PA2/PA3 はUSART2 |
| PA5, PA6, PA7 | 4, 11, 5 | PA6/PA7 はTIM3 PWM |
| PB0, PB1, PB3 | 12, 6, 13 | PB0/PB1 はUSART4 |
| PB11, PB12 | 9, 14 | PB10はboard headerに未露出 |
| PC14, PC15, PC16, PC17 | 10, 15+45, 52, 50 | PC15はP4の2入力へ分岐 |

P4 GPIO2/54はRVSWD、GPIO24〜27はUSB PHYなのでfixture GPIOから予約除外する。P4のGPIO bankは
利用可能maskと値maskを一度に返すread-only operationであり、USB Serial/JTAGのGPIO24/25を
`pinMode()`してCDCを切断する初期問題を回避済みである。

## UART実機確認

USART4のPB0(TX)→P4 GPIO12(RX)、PB1(RX)←P4 GPIO6(TX)をFixtureUartとして公開した。
`CH32_SERIAL_DEFAULT=4`でbuildした既存`serial_println`は、115200 baudでREADY、PING/PONG、RUN、
`hello from ch32`、`hex=BEEF`、`done failures=0`を全て確認した。

USART2のPA2/PA3も物理対応は確認済みだが、長い連続出力時にX035のTX割込みringが満杯になる所見が
あった。X035のみTXE pollingへフォールバックして送信停止は解消した。

## Core自己試験の現状

`core_api`をUSART4経由で再実行し、`analogWrite(PA1, 0/128/255)`を含む18項目すべてが
`PASS`、`core_api done failures=0`となった。`CriticalSection` exampleも連続実行し、
`noInterrupts()` / `interrupts()`の前後で1秒周期の`millis()`が継続して増加することを確認した。

初期の停止はTIM2/PWM回路ではなかった。X035/X033のsketchはU modeで動作するため、通常の
`mstatus` CSRを読むタイマー資源管理のcritical sectionがillegal instructionになっていた。WCH EVTの
core headerと同様、U modeからアクセスできるQingKe CSR `0x800`の割込みenable bits `0x88`で
保存・マスク・復帰するようCoreを修正した。Arduino APIの`interrupts()` / `noInterrupts()`も同じCSRを使う。
RAM phase markerでPWMの0→128→255を連続完走してから、通常の自己試験で確認した。

2026-09-21に、P4がESIG `0x1ffff704`から読んだchip-id
`0x035e0601`（CH32X035F8U6）へ、同じF8U6 variantで新規にbuildした
`core_api`を再書込みした。buildは隔離したArduino CLIのuser/data/download directory内で行い、
`ARDUINO_CH32X035F8U6`と`CH32_SERIAL_DEFAULT=4`をcompile commandから確認した。imageは9,780 byte、
OEPのF8U6 preflight（63,488 byte flash、OBR `0x03fffffc`、WPR `0xffffffff`）後に39 physical pageを
programし、全域verifyまで成功した（program 14.808338 s、verify 5.108294 s）。

FixtureUart（PB0 TX→P4 GPIO12 RX、PB1 RX←P4 GPIO6 TX、115200 baud）で起動bannerを待ち、
`RUN\\n`を送った結果、`core_api done failures=0`を受信した。PASSした19項目は`millis`、`micros`、
digital I/O、pin encoding、`analogRead`、ADC channel、`analogWrite`、attach/detach interrupt、
`shiftOut`、`pulseIn_timeout`、乱数、`availableForWrite`、port API群である。従って、ここでの
Core自己試験はC8T6 binaryの代用ではなく、実装対象CH32X035F8U6のHIL evidenceである。

## X035 core検証の次段階

P4 prototypeは少なくとも書込み、reset、UART command/response、GPIO出力の入力観測、X035の時刻・ADC・
PWM・EXTIを含むCore自己試験に利用できる。次の未検証範囲は、fixtureが能動的に供給するADC
0/1.65/3.3 V、I2C/SPI peer、PWM周波数/デューティの波形計測である。

## 能動GPIO / ADC三状態試験（2026-09-21）

FixtureGpioに構成operationを追加した。予約済みのRVSWD/USB GPIOを除くallowed pinだけについて、
floating input、pull-up、pull-down、両pull、push-pull LOW/HIGH、open-drain LOW/releaseを選べる。
この表面はGPIOを直接駆動するので、targetがoutputのときは競合させず、検証終了時は必ずfloating inputへ
戻す。

PA5（X035 ADC ch5、P4 GPIO4）をADC入力にして実測した結果は次の通りである。

| P4 GPIO4の状態 | X035 ADC値（10 bit） | 判定 |
|---|---:|---|
| push-pull LOW | 2 | 0 V端点として使用可 |
| input pull-up + pull-down | 345 | 約1.1 V。中点ではない |
| push-pull HIGH | 1001 | 3.3 V端点として使用可 |

P4の内部pull-up/pull-downは抵抗値が対称でないため、両方有効にしても1.65 Vにはならなかった。
従って「ADCの0/1.65/3.3 Vをrelease基準で試す」プローブは、GPIO pullだけでは不足である。専用probeの
capability宣言には、校正済みDACまたは抵抗分圧（少なくとも0 V、Vref/2、Vref）を含める必要がある。
PA0→P4 GPIO46は入力観測には使えるがP4側output非対応であり、能動駆動用pinとして除外する。
