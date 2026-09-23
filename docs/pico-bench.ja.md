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

Pro Micro から ARM SWD 380 組・CH32 RVSWD 420 組を総当たりして、どれも無応答。**CH32L103 の WCH-LinkE
（`0E028F0692F1`）が挿さっておらず**、あの基板に電源が来ていない。給電してから再測する。

### 両機の GPIO は互いに繋がっていない

両方を OEP probe にして片側を駆動・他側で bank read（E143 と同じ方法）→ 1 本も動かない。
Zero の GP0/GP1 は Pro Micro の **GPIO ではなく SWD ヘッダ**に行っている、という配線の裏付け。

## 書き込みと USB bind の実際

- udev rule を入れた後は **picotool が WSL から使える**。ただし **複数の RP2 を同時に列挙すると picotool 2.3.0 は
  segfault する**ので、`--bus N --address M` で必ず 1 台を指定する。
- 再書き込みは `scratchpad/picoflash.sh <busid> <by-id node> <uf2> <BOOTSEL VID:PID>`（1200 bps touch → BOOTSEL →
  picotool → アプリ復帰 → attach）。
- **usbipd の bind は 4 つの identity すべてで取得済み**（各機のアプリ側と BOOTSEL 側）。bind は永続するので、
  以後の書き換えに管理者権限は要らない（2026-09-23 実測）。attach は非管理者で可。

## 未了

- **L103 に給電して**（WCH-LinkE `0E028F0692F1` を挿す）RVSWD pair を確定する。候補は SWDIO=GP24 / SWCLK=GP23。
- ordinary SWD を OEP の service にするか決める（今は frame + survey sketch まで。`target.control` は CH32 DM 専用）。
- V003 を決めた pinout で繋いだ後のフルテスト（GPIO / UART / I2C / SPI / ADC / reset）。配線を決める際は、
  PIO を使う日のために **SWD/UART/SPI の各組を連番ピンに**寄せておくと後が楽。
- Pico の `fixture.capture`（PIO + DMA なら P4 級が狙えるが、配線確定まで着手しない）。
