# Pico bench（RP2040-Zero + Pro Micro RP2350）2026-09-23

## 機材と配線（暫定）

| 役割 | board | 識別 | 配線 |
|---|---|---|---|
| ordinary ARM SWD の対抗 | Waveshare RP2040-Zero（"PICO Zero"） | BOOTSEL serial `E0C9125B0D9B`、`2e8a:0003` | GP0 / GP1 → Pro Micro RP2350 の **SWD ヘッダ**（どちらが SWCLK / SWDIO かは未記録） |
| OEP probe（CH32L103 向け） | SparkFun Pro Micro RP2350（"PICO2"） | flash uid `9489dd2ae0953650`、BOOTSEL `2e8a:000f` | header の**右半分ぐらい**が CH32L103 へ。RVSWD の pair は未特定 |

利用者の方針（2026-09-23）: **配線は暫定で入れ替える可能性がある**。Pico は PIO のために pin の並びが効くが、
**当面 PIO は使わず低速の bit-bang**。まずは **ordinary SWD と単純な GPIO** を今の配線でできる範囲だけ確認し、
**フルテストは CH32V003 を決めた pinout で PICO Zero か PICO2 に繋いでから**行う。

## firmware

| sketch | profile（`sketch.yaml` で版を固定） | 内容 |
|---|---|---|
| `examples/Rp2350L103Probe` | `rp2040:rp2040 (6.1.1)` / `sparkfun_promicrorp2350:usbstack=picosdk` | OEP v0 endpoint を USB CDC に出す。probe.identity / target.control・memory・flash（RVSWD、既定 GP2=SWDIO, GP3=SWCLK、`-DOEP_RVSWD_SWDIO=` で変更）/ fixture.gpio / fixture.uart |
| `examples/Rp2040SwdSurvey` | `rp2040:rp2040 (6.1.1)` / `waveshare_rp2040_zero:usbstack=picosdk` | ordinary ARM SWD の bring-up。GP0/GP1 の両向き × half period × multidrop TARGETSEL 候補を掃いて DPIDR が返る組を報告する（read only） |

core は **global に入れない**。`arduino-cli compile --profile <name>` が `~/.arduino15/internal/` へ pin 止めで展開する。

## 実装の構成

- `src/OepRp2BitBang.h` — RP2040/RP2350 の SIO bit-bang pin 操作。RVSWD と ARM SWD が共有する。PIO は使わない（配線が暫定のため）。
- `src/OepRvswdFrame.h` — CH32 RVSWD の frame（1 実装。ESP32 dedicated GPIO backend と RP2 SIO backend が同じ frame を使う）。
- `src/OepSwdFrame.h` — ordinary ARM SWD（ADIv5）の frame。request / turnaround / ACK / parity / line reset / multidrop TARGETSEL。
- `src/OepPlatform.h` — core 間の差（pin mode、UART の pin 指定とバッファ）。

## 書き込み

BOOTSEL の RP2 は libusb 経由なので、WSL では udev rule が要る（1 回だけ）。

```sh
sudo tee /etc/udev/rules.d/99-rp2-picotool.rules >/dev/null <<'RULE'
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0666", TAG+="uaccess"
RULE
sudo udevadm control --reload-rules && sudo udevadm trigger
```

その後は WSL から `arduino-cli upload --profile <name> --input-dir <dir> <sketch>`。
**VID/PID が変わる遷移（MicroPython → BOOTSEL → 自作 firmware）では usbipd の bind をやり直す**必要がある（管理者権限）。

## 2026-09-23 の実測

### ordinary ARM SWD は通った

RP2040-Zero → Pro Micro RP2350 の SWD ヘッダ。**SWCLK = GP0、SWDIO = GP1**（利用者の記憶どおり「0 と 1」、向きはこの通り）。

| 項目 | 値 |
|---|---|
| ACK | OK（`0b001`） |
| DPIDR | **`0x4c013477`** = Arm designer、DP **version 3**（ADIv6）、RP2350 の SW-DP |
| 起動系列 | `jtag->swd` / `dormant->swd` / line reset のいずれでも応答 |
| multidrop TARGETSEL | **不要**。line reset 直後の DPIDR read に答える（RP2040 と違う） |
| half period | 500 ns（bit-bang、SIO 直叩き） |

同じ線で CH32 RVSWD を試すと無応答（正しい陰性対照）。

### pin survey: 何が繋がっているか

released input を読むだけでは RP2350 の**エラッタ E9（浮いた入力が high に張り付く）**で全 pin が「pull-up」に見える。
pad 側の pull を当てて読むと切り分けられる。

| board | 外部 pull-up | 外部 pull-down | それ以外 |
|---|---|---|---|
| RP2040-Zero | GP0, GP1（SWD の 2 本、≤10 kΩ 級） | なし | 18 本すべて free |
| Pro Micro RP2350 | **GP24** | **GP23**, GP29 | 他は free |

GP24 / GP23 は CH32 の SWDIO(PA13, pull-up) / SWCLK(PA14, pull-down) の signature と一致するので、L103 の RVSWD pair の
**候補**として example の既定にした（未確認）。

### L103 は応答しない = 無給電

Pro Micro から ARM SWD 380 組・CH32 RVSWD 420 組を総当たりして、どれも無応答。**CH32L103 基板に電源が来ていない**
（利用者確認済み、2026-09-23。WCH-LinkE `0E028F0692F1` も挿さっていない）。したがってこの結果は配線の否定にはならない。

給電後の再測は 1 コマンドで済む:

```sh
cd <scratchpad>
./picoflash.sh 9-1 /dev/serial/by-id/usb-SparkFun_ProMicro_RP2350_9489DD2AE0953650-if00 \
    $PWD/surveyp/PicoDebugPortSurvey.ino.uf2 2e8a:000f
# console を読むと pull signature と RVSWD pair sweep が出る。確定したら
# examples/Rp2350L103Probe の既定 pin を直して probe firmware に戻す。
```

### 両機の GPIO は互いに繋がっていない

両方を OEP probe にして片側を駆動・他側で bank read（E143 と同じ方法）→ 1 本も動かない。
Zero の GP0/GP1 は Pro Micro の **GPIO ではなく SWD ヘッダ**に行っている、という配線の裏付け。

## L103 に給電したあと（同日、利用者が電源を接続）

### RVSWD の pair が確定

Pro Micro の **SWDIO = GP0、SWCLK = GP1**。全 420 組の総当たりでこの 1 組だけが応答し、DMSTATUS = `0x00000c82`
（version 2、authenticated、allrunning）。GP0/GP1 は Serial1 の既定 pin でもあるので、`fixture.uart` には別の組を割り当てる。

### 半周期の実測（この配線・flying wire）

| half | 冷えた bus の初回応答 | 連続 1000 read |
|---:|---|---|
| 0 / 25 ns | 応答なし | — |
| 50 ns | 応答するが内容が壊れる | identical 0 / differed 793 / parity fail 207 |
| **100 ns** | 安定 | **1000 / 1000** |
| 200 / 500 ns | 安定 | 1000 / 1000 |

`attach()` は 100 ns を選ぶ。1 DMI read は約 26 µs。

### `attach()` の修正 2 点（どちらも実測が根拠）

1. **半周期ごとに bus を張り直す**。追従できない半周期を試すと DM が歩調を崩し、そのままだと後続の遅い候補まで巻き添えになる。
2. **冷えた DM は最初の wake に答えない**。500 ns で「5 回目の試行」で初めて応答し、以後は 95/100。よって候補ごとに
   wake を最大 8 回試してから判定する。これを入れる前は、温まっていない L103 が「ターゲット無し」に見えていた。

修正後、OEP の `target.control.read_dmi` が probe 経由で DMSTATUS を返す。

### 未解決: halt が効かない（原因は 2 候補まで絞り込み、特定は未）

| 試したこと | 結果 |
|---|---|
| `DMCONTROL` へ 0x80000001 を書いて**読み戻す** | `0x80000001`（haltreq=1）。**書き込みは届いている** |
| その直後の DMSTATUS | `0x00000c82` のまま = allrunning。halt しない |
| haltreq のまま 10〜80 ms 追跡 | `0xfffffdbe` / `0xfffffffe` など**ほぼ全 1 の壊れた語**に劣化する |
| 半周期ごとの bus 再初期化（`wakeBus()`）を挟んで再読 | 同じ。allrunning か壊れた語 |
| haltreq + ndmreset → ndmreset だけ解除（reset 経由の halt） | 返る語がやはり全 1 系。`allhalted=1` に見える回も内容が壊れており**信用できない** |

つまり **halt を要求した後だけリンクが壊れる**。hart が走り続けているのか、halt はしているが読めていないのかを
区別できていない。候補は 2 つ:

1. **DMCONTROL への書き込みが時々化ける**。RVSWD の write は ACK も検証も無いので、bit 0（dmactive）が落ちれば
   DM 自体が寝てバスは全 1 になる。読みが 1000/1000 clean でも write が同じ品質とは限らない。
   対策候補: 書き込みのたびに読み戻して検証する、write だけ遅い半周期にする。
2. **L103 側が低消費電力モード**で core clock が止まっており halt を受け付けない（`DBGMCU` の stop/standby
   debug enable 未設定）。この基板に何の firmware が入っているか不明なのが効いている。

X035（PCB 治具）では同じ `Ch32Dm::halt()` が通るので、PHY や DM 手順そのものの誤りではない。
**次の一手**: L103 に既知の firmware を入れてから再測する。L103 のフル結線検証を P4 で行う際に一緒に切り分けるのが早い。
### 現在の配線状態（2026-09-23 終了時点）

- **Pro Micro ↔ CH32L103**: 生きている。GP0 = SWDIO / GP1 = SWCLK。
- **Pico Zero ↔ Pro Micro の SWD ヘッダ**: **外れている**。Zero の GP0/GP1 は pull 判定で「何も付いていない」に戻り、
  DPIDR も返らない。SWD が通った記録（DPIDR `0x4c013477`）は線が生きていた時点のもので、有効。
  繋ぎ直せば `PicoDebugPortSurvey` の「ARM SWD write path」節（DP CTRL/STAT を書いて power-up ack を読む）まで進める。

## 書き込みと USB bind の実際

- udev rule を入れた後は **picotool が WSL から使える**。ただし **複数の RP2 を同時に列挙すると picotool 2.3.0 は
  segfault する**ので、`--bus N --address M` で必ず 1 台を指定する。
- 再書き込みは `scratchpad/picoflash.sh <busid> <by-id node> <uf2> <BOOTSEL VID:PID>`（1200 bps touch → BOOTSEL →
  picotool → アプリ復帰 → attach）。
- **usbipd の bind は 4 つの identity すべてで取得済み**（各機のアプリ側と BOOTSEL 側）。bind は永続するので、
  以後の書き換えに管理者権限は要らない（2026-09-23 実測）。attach は非管理者で可。

## 未了

- **L103 の halt が効かない**理由の切り分け（書き落ち / 低消費電力モード / DBGMCU）。読みと attach は通る。
  L103 に既知の firmware を入れてから再確認するのが早い。
- ordinary SWD を OEP の service にするか決める（今は frame + survey sketch まで。`target.control` は CH32 DM 専用）。
- V003 を決めた pinout で繋いだ後のフルテスト（GPIO / UART / I2C / SPI / ADC / reset）。配線を決める際は、
  PIO を使う日のために **SWD/UART/SPI の各組を連番ピンに**寄せておくと後が楽。
- Pico の `fixture.capture`（PIO + DMA なら P4 級が狙えるが、配線確定まで着手しない）。
