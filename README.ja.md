# OEP development probe firmware

Open Embedded Probe（OEP）の probe を Arduino で書くためのライブラリと、各 probe のファームウェア（`examples/`）。
v1（oep-spec の `docs/oep-core.ja.md` と標準インターフェースの `docs/oep-if-*.ja.md`、固める候補の形）を話す。破壊的変更を前提とする実験段階で、互換は約束しない。

wire 上の数値は oep-spec の `registry/oep-v1.toml` が唯一の定義で、その生成物を `src/OepV1Registry.h` に写している。
OEP を初めて読む人は oep-spec の `docs/review-guide.ja.md`（どこに何が書いてあるか）から。

## 構成

| PATH | 中身 |
|---|---|
| `src/OepV1.h`、`src/OepV1Endpoint.*`、`src/OepV1Registry.h` | v1 の本体（oep-core）: フレーム、名前で探すインターフェース、ロック、複数の経路（describe の transport）、シリアルの口の共用（core §3.4）、plan、通知 |
| `src/OepV1Bind.*` | シリアルの口に流すもの（bind: last-reset / manual / mixed、セッション中の停止と最後の reset からの再開） |
| `src/OepV1Stream.h`、`src/OepV1Debug.h` | 標準インターフェースの共通部品（位置つきのストリーム、線と target の status とピンの組） |
| `src/OepV1Target.*` など | 標準インターフェース: 線と target（`oep.wire.rvswd` / `swio` / `swd`、`oep.target.riscv-dm` / `arm-adi`）、コンソール、fixture（gpio / uart / capture）、`oep.probe.config`（スロット、bind、NVS への保存。ESP32）。各ファイルの冒頭に対応する仕様の節がある |
| `src/OepP4I2cTarget.*`、`src/OepP4SpiTarget.*` | 独自インターフェース `io.github.ch32-riscv-ug.esp32.i2c-target` / `spi-target`（revision 1、ESP-IDF の I2C / SPI スレーブ） |
| `src/OepCh32Dm.*`、`src/OepRvswdPhy.*`、`src/OepSwioPhy.*`、`src/OepDmConsole.*`、`src/OepPinTable.h`、`src/OepPlatform.h`、`src/OepFrame.*` など | 部品（CH32 のデバッグモジュール、線の物理層、コンソールの framing、ピンの表と空きの状態、Arduino の core の差、フレーム） |
| `examples/` | probe のファームウェア（ESP32-P4 + X035、classic ESP32 + V003、RP2350 / RP2040、P4 HS のキャプチャ `Esp32P4CaptureProbe` と `host/stream_test.py`） |
| `tests/host/` | 移植できる部分（シリアルの口の読み、endpoint の共用の規則、bind）の host の試験: `tests/host/run.sh`（g++） |
| `tools/sync_registry.sh` | oep-spec の `generated/oep-v1/oep_v1_registry.h` を `src/OepV1Registry.h` に写す |
| `docs/` | 日付入りの作業記録（経緯） |

## 使い方

firmware は**使う前に転送する**（ボードには何が入っているか分からない）。例: X035 の治具。

```sh
arduino-cli compile --clean examples/Esp32P4X035Probe
arduino-cli upload -p /run/board-identify/by-id/esp32-series-30eda0e31108 examples/Esp32P4X035Probe
```

host は oep-client-python の `oep_client.v1`（`link.open_host()` / `link.open_usb_host()`）。シリアルの口（USB-Serial/JTAG、
USB CDC、UART bridge）は OEP のフレーム（`0x00 <COBS> 0x00`）と bind の生のバイトを 1 本で運ぶ。host は排他（TIOCEXCL）で開き、
フレームの外は雑音として捨てる（oep-spec の host 開発ガイド §1.6、§2）。

## 既知の罠

- Arduino.h は `word(...)` を `makeWord(...)` の macro にしている。lambda や関数を `word` と名付けると引数がそのまま返る。
- ESP32-P4 の `RvswdPhy::begin` の後に同じピンへ `pinMode` / `digitalWrite` を使うと、dedicated GPIO の束から外れて戻らない（chip の reset が要る）。
- direct build（`build_opt.h` で EspUsbDevice の vendor を直接書く形）のスケッチは、`build_opt.h` を変えたら `--clean` でビルドする。
