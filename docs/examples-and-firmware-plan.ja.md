# examples と配布する firmware の見直し（2026-09-30。方向はユーザー了承済み）

今の `examples/` は、手元の治具ごとの firmware（「ボード + 治具の target の chip」、例: `Esp32P4X035Probe`）で、治具の配線を
焼き込んでいる。治具を持たない人には名前も中身も意味を持たない。ここでは、今の治具のためではなく、**これから OEP を使う人・作る人が
要る sample と binary** から並びを決め直す。

## 1. 誰が何を要るか

| 人 | したいこと | 要るもの |
|---|---|---|
| 使う人（Arduino を触らない） | CH32 の書き込み・デバッグ（RVSWD / SWIO）、ARM の SWD、pytest の治具（GPIO / UART）、ロジックのキャプチャ、target のコンソール | **手持ちの定番ボードに焼くだけの binary**。配線は焼いた後に決める |
| probe を作る人 | 自分の基板で probe を作る、独自のインターフェースを足す、別の経路で運ぶ | **1 つずつのことを示す短い sketch**（学ぶ順に並ぶ） |
| host を作る人 | 道具、IDE、試験の枠組み | firmware は要らない: oep-client-python の偽の probe（`oep_client.fake_serve`）。実機では上の binary |

治具（自分の試験台）は、上の binary に**設定**（probe.config の slot / bind / label / idle、`oep config`）を入れたものとして表す。
治具ごとの firmware は作らない。

## 2. 原則

1. **binary は 1 ボード 1 本**。そのボードでできることを全部入れ、どのピンを使うかは host が決める（oep-if-debug §1 のピンの組:
   describe の role_channels、scan の count = 0 はすべての組、attach の pins。fixture は plan）。仕様はすでにこれを許している。
2. **binary を出すのは、ベンチで確かめられるボードだけ**: ESP32-P4、classic ESP32、RP2350、RP2040。RP2040 / RP2350 は、ピンの
   番号が同じ他のボード（Pico、Pico 2、RP2040-Zero、Pro Micro RP2350 など）でも動く 1 本にする。**ベンチの無いボード**
   （ESP32-S3、C3、C6 など）には binary を出さず、自分でビルドする手引き（§3.1）を用意する。確かめていない binary は配らない。
3. **名前**: binary はボード（`OepProbe-rp2040.uf2`）、sketch はやること（`SwdDebugProbe`）。target の chip の名前は入れない。
4. **sketch は 1 つのことだけを示す**。ピンは冒頭の定数。読んで写して変えるためのもの。
5. **治具に固有の事情は firmware から出す**。今 sketch に焼き込んでいるもの（§5）は、ボードの事実、target の事実、治具の設定の
   どれかで、それぞれの置き場所がある。

## 3. binary（Release に付ける firmware）

| binary | ボード | 経路 | 線 | fixture | その他 |
|---|---|---|---|---|---|
| `OepProbe-rp2040` | Pico、RP2040-Zero ほか | USB CDC | rvswd、swd | gpio、uart | console |
| `OepProbe-rp2350` | Pico 2、Pro Micro RP2350 ほか | USB CDC | rvswd、swd | gpio、uart | console |
| `OepProbe-esp32` | ESP32 DevKitC ほか | UART bridge | swio | gpio、uart | console、probe.config |
| `OepProbe-esp32p4` | ESP32-P4 | HS vendor bulk、HID、USB CDC、USB-Serial/JTAG | rvswd | gpio、uart、capture | console、probe.config、I2C / SPI target |

どれも iProduct は `OEP` で始め、PID が割り当てられたら `1209:4F45`（PID-USE.md）。describe の probe の名前はボード
（`raspberrypi-pico` など）で、治具の profile（`io.github.ch32-riscv-ug.rp2350-l103` など）は付けない。

Firmware の workflow は、`examples/Firmware/` の各 sketch の sketch.yaml の profile ごとに作って付ける
（`<sketch>-<profile>-<version>.uf2` / `.merged.bin`）。他の example は CI でビルドが通るかだけを確かめる。

### 3.1 ベンチの無いボードの手引き

`Firmware/OepProbe` を自分のボードでビルドする手順を書く（docs か README）: board の選び方（arduino-cli の FQBN）、sketch.yaml に
profile を足す方法、ボードの予約ピン（§6 f）の足し方、焼いた後に `oep dump` で確かめること。ボードが動いたと報告があり、ベンチで
確かめられるようになったら §3 に足す。

## 4. sketch（学ぶ順）

状態（2026-09-30）: 作った。03.Transports は MultipleTransports の 1 本（UartBridge は 01.Basics の classic ESP32、SharedConsolePort は 06.Settings/ProbeConfig に含めた）。LogicCapture は旧 Esp32P4CaptureProbe、SwdPinSurvey は旧 PicoDebugPortSurvey を移したもの。

```
examples/
  01.Basics/
    MinimalProbe        oep.core だけ。host の `oep dump` に見える最小の probe。各行に説明
    FixtureProbe        gpio と uart: pytest から動かす治具
  02.Interfaces/
    CustomInterface     独自のインターフェース（io.github.<you>.<name>、revision、describe、plan のピン）
  03.Transports/
    UartBridge          UART bridge の 1 本（classic ESP32 など）: DTR / RTS、固定の速さ
    SharedConsolePort   シリアルの口を OEP と target のコンソールで共用する（bind、core §3.4）
    HsVendorBulk        ESP32-P4 の HS vendor bulk（direct build、DirectBulkStream）
  04.Debug/
    RvswdDebugProbe     CH32（RVSWD 2 線）: wire + riscv-dm + console
    SwioDebugProbe      CH32V00x（SWIO 1 線）
    SwdDebugProbe       ARM（SWD）: wire + arm-adi
  05.Capture/
    LogicCapture        ESP32-P4 の PARLIO で 160 Msps（host/stream_test.py 付き）
  06.Settings/
    ProbeConfig         slot / bind / label / idle を保存して起動時から使う
  Tools/
    SwdPinSurvey        どのピンが debug port か探す（立ち上げ用）
  Firmware/
    OepProbe            §3 の binary（profile: rp2040、rp2350、esp32s3、esp32、esp32p4）
```

## 5. 今の example の行き先

| 今 | 焼き込んでいるもの | 行き先 |
|---|---|---|
| `Esp32P4X035Probe` | RVSWD の組、4 経路、スロット 1 つ、capture | `Firmware/OepProbe` の esp32p4 + 治具の設定（slot、bind） |
| `Esp32V003Probe` | SWIO GPIO16、NRST GPIO23 | `Firmware/OepProbe` の esp32 + 設定（label NRST）。reset 線は host が明示する（§6 d） |
| `Rp2350L103Probe` | RVSWD GP0/1、NRST GP2、ch12 の pull-up、bus を low で休ませる、半周期 500 ns、GP19（PSRAM の CS） | `Firmware/OepProbe` の rp2350 + 設定（label、idle）。bus と速さは host が持つ（§6 e）、GP19 はボードの事実（§6 f） |
| `Rp2040ZeroProbe` | SWD GP0/1 | `Firmware/OepProbe` の rp2040 |
| `Esp32P4CaptureProbe` | capture、試験の信号、303a:4021 | `05.Capture/LogicCapture`（USB の ID は他と同じにする） |
| `PicoDebugPortSurvey` | 走査 | `Tools/SwdPinSurvey` |

状態（2026-09-30）: X035 の治具（esp32p4）と V003 の治具（esp32）は、0.0.8 の `Firmware/OepProbe` + 設定でベンチを通ったので、
`Esp32P4X035Probe` と `Esp32V003Probe` は消した。RP2 の 2 本は、RP2350 のベンチに給電されてから。

ベンチ（ArduinoCore-CH32 の `tests/benches/*.toml`）は、example の名前の代わりに binary と設定のファイルを持つ。切り替えは、
各治具で新しい binary と設定がベンチの確認を通った版で、古い example を消して行う（途中の二重持ちはしない）。

## 6. ライブラリに足りないもの（binary を作る前に要る順）

a. **線がピンの組を host から受ける**（済: oep-spec 29990da / dfa46d2 / ad43e25、firmware。`Firmware/OepProbe` の rp2040 / rp2350 / esp32p4 / esp32。SWIO も実行中のピン。実機の確認は未）: 今は決まった 1 組だけ（`fixedPairScan` / `fixedPairPins`）。describe の role_channels で
   使えるピンを出し、scan / attach の pins で組を受ける（oep-if-debug §1）。RVSWD（ESP32 の dedicated GPIO、RP2 の SIO）、SWIO、SWD の
   それぞれで、ピンを実行中に替えられるようにする。
b. **RP2040 / RP2350 の probe.config の保存**（済: flash の最後の領域、arduino-pico の EEPROM）（今は ESP32 の NVS だけ）。flash の最後の領域など。
c. **USB の名乗り**（RP2 は済: `Firmware/OepProbe` が `USB.setProduct`）: RP2 と ESP32-S3 でも iProduct `OEP…` と個体ごとの serial を出す（今は ESP32-P4 だけ）。
d. **reset 線に既定は無い**（済: oep-spec 5bfe052、firmware 620594c）: attach_under_reset の channel は必須。probe は reset に使ってよい
   channel を describe の role_channels の role 3 で宣言し、plan が持つ channel は断る。
e. **線の設定は probe の中に持たない**（済: 同上）: 休ませ方は rvswd の attach の idle_clock、速さの上限は max_speed で host が渡す。
   host 無しで attach するスロットは、スロットの項目の max_speed / idle_clock に同じ値を持つ。target 系統ごとの値（LinkE も
   L103 / V203 は SWCLK low、X035 は両線 high）と「reset 直後の遅いクロックの間は遅く」の手順は host が持つ（2026-09-30、
   ベンチと wch-protocols と相談）。
f. **ボードの予約ピン**（Pro Micro RP2350 の PSRAM の CS GP19、RP2040-Zero の WS2812 GP16）は、ボードごとの表を
   `Firmware/OepProbe` に持つ（Arduino の board のマクロで選ぶ）。
g. 後で: RP2 の SWIO（PIO）、ESP32 の SWD。§3 の表の空いたところを埋める。

## 7. 決めること

- binary の名前（`OepProbe-<board>`）と、ベンチが読む asset 名の規則・治具の設定ファイルの形（ベンチと合わせる）。
