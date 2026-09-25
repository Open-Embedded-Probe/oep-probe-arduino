# OEP development probe firmware

Open Embedded Probe（OEP）の probe を Arduino で書くためのライブラリと、各 probe のファームウェア（`examples/`）。
v1（oep-spec `docs/v1-core-wire-delta.ja.md`、固める候補の形）を話す。破壊的変更を前提とする実験段階で、互換は約束しない。

wire 上の数値は oep-spec の `registry/oep-v1.toml` が唯一の定義で、その生成物を `src/OepV1Registry.h` に写している。
OEP を初めて読む人は oep-spec の `docs/review-guide.ja.md`（どこに何が書いてあるか）から。

## 構成

| PATH | 中身 |
|---|---|
| `src/OepV1*.h` / `src/OepV1*.cpp` | v1 の本体: endpoint（フレーム、名前で探すインターフェース、ロック、複数の経路）、線と target（`oep.wire.rvswd` / `swio` / `swd`、`oep.target.riscv-dm` / `arm-adi`）、コンソール、fixture（gpio / uart / capture）、`oep.probe.config`（試作）。各ファイルの冒頭に対応する仕様の節がある |
| `src/OepCh32Dm.*`、`src/OepRvswdPhy.*`、`src/OepSwioPhy.*`、`src/OepDmConsole.*` など | 世代によらない部品（CH32 のデバッグモジュール、線の物理層、コンソールの framing） |
| `src/OepEndpoint.*`、`src/OepService.h`、`src/oep_v0.*` など | v0 の endpoint と codec（経緯。v1 の probe は使わない） |
| `examples/` | probe のファームウェア（ESP32-P4 + X035、classic ESP32 + V003、RP2350 / RP2040、P4 HS）と試作（`Esp32P4HsPrototype`、`Esp32P4X035ConsolePrototype`） |
| `docs/` | 日付入りの作業記録（経緯） |
| `tests/hil/` | v0 の HIL 試験（古い。v1 の実機の回帰は ArduinoCore-CH32 の `tests/manual/oep_smoke/`） |

## 使い方

firmware は**使う前に転送する**（ボードには何が入っているか分からない）。例: X035 の治具。

```sh
arduino-cli compile --clean examples/Esp32P4X035Probe
arduino-cli upload -p /run/board-identify/by-id/esp32-series-30eda0e31108 examples/Esp32P4X035Probe
```

host は oep-client-python の `oep_client.v1`（`link.open_host()` / `link.open_usb_host()`）。

## 既知の罠

- Arduino.h は `word(...)` を `makeWord(...)` の macro にしている。lambda や関数を `word` と名付けると引数がそのまま返る。
- ESP32-P4 の `RvswdPhy::begin` の後に同じピンへ `pinMode` / `digitalWrite` を使うと、dedicated GPIO の束から外れて戻らない（chip の reset が要る）。
- direct build（`build_opt.h` で EspUsbDevice の vendor を直接書く形）のスケッチは、`build_opt.h` を変えたら `--clean` でビルドする。
