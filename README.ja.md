# OEP development probe firmware (v0)

破壊的変更を前提とする Arduino 向け OEP probe 実装。wire 上の数値は
[oep-spec](../oep-spec) の `registry/oep-v0.yaml` が唯一の定義で、`tools/sync_codec.sh` が生成 codec
（`src/oep_v0.h` / `src/oep_v0.c`）を取り込む。設計の指針は oep-spec `docs/development-guidelines.ja.md`、
進行は `docs/rebuild-plan-2026-09-22.ja.md`。

## 構成

| ファイル | 役割 |
|---|---|
| `src/OepFrame.*` | length-prefixed frame（reliable byte stream 用） |
| `src/OepEndpoint.*` | request の routing、core（confirm / list / describe / ping）、result の送出、watchdog |
| `src/OepService.h` | service（offered function）の interface |
| `src/OepProbeIdentity.h` | probe.identity |
| `src/OepRvswdPhy.*` | RVSWD PHY。ESP32 dedicated GPIO、push-pull + 明示 turnaround、session ごとの half period margin check、bounded retry |
| `src/OepCh32Dm.*` | CH32（QingKe V4、X035）debug module: halt / resume / reset、autoexec reader、autoexec flash writer |
| `src/OepTargetServices.*` | target.control / target.memory / target.flash。program_page は erase → program → read-back を 1 transaction にする |
| `examples/Esp32P4X035Probe/` | ESP32-P4 + CH32X035F8U6 fixture の firmware（USB-Serial/JTAG、1 KiB frame、4 KiB window） |
| `tests/` | HIL 回帰試験（build → upload → test。`tests/README.ja.md`） |

## 使い方

firmware は**使う前に転送する**。

```sh
arduino-cli compile --clean --profile esp32p4 examples/Esp32P4X035Probe
arduino-cli upload --profile esp32p4 --port /run/board-identify/by-id/esp32-series-30eda0e31108 examples/Esp32P4X035Probe
```

host は oep-client-python の `oep_client.v0`（`Client`、`services.TargetMemory.read_range`、`services.TargetFlash.program_pages`）。
USB-Serial/JTAG の port は open / close で P4 が reset するので、pytest からは harness の serial 実体を借りる。

## 2026-09-22 の実測（HIL `tests/hil/probe`）

| 操作 | 新 stack | 旧 prototype（E145） |
|---|---:|---:|
| 62 KiB full read（USB 越し、host CRC = probe CRC） | 0.154 s（414 kB/s） | 4.94 s |
| 256-byte page program（erase + program + read-back）pipelined | 7.3 ms/page | 440 ms/page |
| 500 B ping × 200 pipelined | 283 kB/s 片方向 | — |

まだ無いもの: fixture service（GPIO / UART / I2C target / capture）と lease、host 側 `program_image`、ArduinoCore-CH32 の sketch runner、
worklist P0 の reliability gate（verify 20 回、差分 program 20 回、中断 5 回）の新 stack での再取得。

## 既知の罠

- **X035 の ndmreset 後に hart が走り出さない回がある**（2026-09-22）。DMSTATUS は allrunning を返すが sketch は動かず、
  次に線を放して再 attach（初期化列 + dmactive 0→1）すると走る。halt してから多数の autoexec read を行った後の reset は
  20/20 走った。`Ch32Dm::reset()` は halt → ndmreset → 解除の読み返し → dmactive 再有効化 → running 待ち → ack →
  線解放 → 再 attach → 解放、の順で、20 + 24 サイクルで 43/44。残る 1 件は台帳候補 `x035-ndmreset-hart-not-running`。

- Arduino.h は `word(...)` を `makeWord(...)` の macro にしている。lambda や関数を `word` と名付けると引数がそのまま返る。
- pytest-embedded の port を close → 再 open すると P4 が reset する。
