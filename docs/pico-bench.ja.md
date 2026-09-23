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

## 未了

- RP2350 ↔ L103 の RVSWD pair 特定（`fixture.gpio` の pull 掃引で「何かに繋がっている pin」を出すところから）。
- ordinary SWD の TARGETSEL 実測（RP2350 の multidrop instance id は datasheet 未確認のため候補掃引）。
- V003 を決めた pinout で繋いだ後のフルテスト（GPIO / UART / I2C / SPI / ADC / reset）。
