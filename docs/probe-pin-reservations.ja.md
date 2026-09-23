# プローブ MCU 別・配線に使わない方がよい pin（2026-09-23）

DUT を繋ぐとき、**プローブ側のどの pin を避けるか**の一覧。目的は 3 つ。

1. **プローブが起動しなくなるのを防ぐ**（boot strap を DUT が引っ張る）。
2. **DUT を意図せず動かさないようにする**（プローブの boot ROM が console へ文字を吐く、LED を叩く）。
3. **黙って壊れる配線を避ける**（flash / PSRAM / USB の専用線）。

各表の「確度」は: **実測** = このベンチで動かして確かめた、**確立** = 一般に確立した事実、**要確認** = datasheet を見ること。

## 原則: ブートストラップと既定 UART は避けるべきか

- **ブートストラップ: 避ける。** reset 解除の瞬間だけ読まれるので、DUT 側が同じ線を low/high に保持していると
  プローブが download mode などで起動する。**DUT が pull を持つかは配線するまで分からない**ので、最初から使わないのが安全。
  ESP32 系は実害が大きい（GPIO0 low で download mode）。**RP2040 / RP2350 には GPIO の strap が無い**ので、この心配は無い
  （BOOTSEL は QSPI_SS 側の専用ボタン）。
- **既定 UART: SoC による。**
  - **ESP32 系は避ける。** boot ROM が reset のたびに UART0（GPIO1/3 等）へログを出すため、繋いだ DUT の受信に
    毎回ゴミが入る。console でもあるので二重に困る。
  - **RP2040 / RP2350 は避けなくてよい。** 「既定」は core の慣習（`Serial1` = GP0/GP1）にすぎず、`setRX/setTX` で
    移動できる。実際このベンチでは **CH32L103 の RVSWD が Pro Micro の GP0/GP1**（= Serial1 既定）に来ており、
    問題なく動いている。ただし同じ pin を `fixture.uart` には使えないので、**申告して避ける**ことだけ必要。

## ESP32-P4（fixture probe `esp32-series-30eda0e31108`）

| pin | 用途 | 確度 |
|---|---|---|
| GPIO24, GPIO25 | **USB-Serial/JTAG**（OEP の transport そのもの） | 実測（probe が予約） |
| GPIO2, GPIO54 | 現 fixture の RVSWD（SWDIO / SWCLK） | 実測 |
| GPIO13, GPIO14 | SD card 用 pad。high 駆動後の復帰が遅く、pull の判定に使えない | 実測（`oep_gpio_matrix` で除外） |
| GPIO9 | 同上（復帰が遅い） | 実測 |
| boot strap | P4 の strapping pin 一覧 | **要確認**（datasheet。手元の SoC ヘッダには無い） |
| flash / PSRAM | 内蔵 flash・PSRAM の専用線 | **要確認**（module 依存） |

P4 は「適当に繋いでよい」のが長所で、実際 55 本中 51 本を fixture channel として使えている。

## ESP32（classic、`esp32-d0wd-v3-0070070d9394`、V003 jig のプローブ）

| pin | 用途 | 確度 |
|---|---|---|
| GPIO6–GPIO11 | 内蔵 SPI flash。触ると即クラッシュ | 確立 |
| GPIO1（TX）, GPIO3（RX） | UART0 = console 兼 boot ROM ログ | 確立 |
| GPIO0 | boot strap（low で download mode） | 確立 |
| GPIO2, GPIO5, GPIO12, GPIO15 | boot strap（GPIO12 = MTDI は flash 電圧を決めるので特に危険） | 確立 |
| GPIO34–GPIO39 | **入力専用**。出力も内部 pull も無い | 確立 |
| GPIO16 | 現 jig の SWIO（V003 PD1） | 実測 |
| GPIO23 | 現 jig の NRST（V003 PD7）。普段は Hi-Z | 実測 |

実際に使える pin は `4, 5, 13, 14, 17, 18, 19, 21, 22, 25, 26, 27, 32, 33` と入力専用の `34–39`。

## ESP32-S3（`esp32-s3-e4b063b4a81c`、現在は USB 経路の実験用）

| pin | 用途 | 確度 |
|---|---|---|
| GPIO0, GPIO3, GPIO45, GPIO46 | boot strap | 確立 |
| GPIO26–GPIO32 | 内蔵 flash（octal 品は GPIO33–37 も） | 確立 |
| GPIO19, GPIO20 | native USB D−/D+ | 確立 |

## RP2040（Waveshare RP2040-Zero、`E0C9125B0D9B`）

| pin | 用途 | 確度 |
|---|---|---|
| GPIO16 | 基板上の WS2812 | 実測（probe が予約） |
| GPIO0, GPIO1 | 現配線で Pro Micro の SWD ヘッダへ（GP0 = SWCLK, GP1 = SWDIO） | 実測 |
| — | **boot strap は無い。** BOOTSEL は QSPI_SS の専用ボタン | 確立 |

`GP0–GP15` と `GP26–GP29` が castellated edge に出ている。GP17–GP25 は出ていない。

## RP2350（SparkFun Pro Micro RP2350、`9489dd2ae0953650`）

| pin | 用途 | 確度 |
|---|---|---|
| GPIO19 | 基板の PSRAM chip select | 確立（variant が宣言） |
| GPIO25 | 基板の NeoPixel | 確立（variant が宣言） |
| GPIO0, GPIO1 | 現配線で CH32L103 の RVSWD（GP0 = SWDIO, GP1 = SWCLK） | 実測 |
| — | **boot strap は無い。** SWD は GPIO ではなく専用ヘッダ | 確立 |
| — | **エラッタ E9**: 解放した入力が high に張り付く。pin 探索では必ず pad 側の pull を当てて読む | 実測 |

## 配線するときの実務

- **PIO を使う日のために、同じ機能の線は連番に寄せる**（SWD の 2 本、UART の 2 本、SPI の 4 本）。RP2 の PIO は
  連続した pin 群を前提にする。今は bit-bang なので順序自由だが、後から効く。
- **使わない pin は起動時に Hi-Z にする。** RP2 の pad は reset 直後 **pull-down 有効**で、ESP32 にも同種の既定がある。
  半結線の治具ではそれが DUT のリセット系を引きっぱなしにしうる。実際 CH32L103 治具では、この pull-down だけで
  「DM には繋がるのに hart が halt しない」状態になった（2026-09-23）。probe sketch は service 起動前に
  `oep::platformParkPins()` を呼ぶこと。プローブは頼まれていない線を動かさない、が原則。
- **プローブ側で予約した pin は `probe.identity` の reserved mask に必ず入れる。** host は fixture channel を
  そこから決めるので、宣言さえすれば誤用は起きない。
- **DUT 側にも同じ注意がある**（CH32 の SWDIO/SWCLK、USB pad、BOOT pin）。DUT 側の制約は
  ArduinoCore-CH32 の `docs/upload-and-fixture.ja.md` の errata 節にまとまっている。
