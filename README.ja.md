# OpenEmbeddedProbe（OEP の probe の Arduino ライブラリ）

[English](README.md)

## OEP とは

Open Embedded Probe（OEP）は、**probe**（開発中のチップにつなぐ小さな基板）と、PC の **host** のソフトウェアの間の、オープンな
プロトコルです。1 つの probe が、デバッガ（WCH の CH32 の RVSWD / SWIO、ARM の SWD）、target のコンソール、試験の治具（GPIO、
UART、ロジックのキャプチャ）を兼ね、どの host（書き込みの道具、IDE のモニタ、pytest）も同じ方法で話します。

- probe は**自分にできることを宣言する**（名前で探すインターフェースと、そのピンや上限）ので、host はボードの表を持たなくてよい。
- **セッションのロック**で、2 つのプログラムが同時に probe を動かすのを防ぐ。
- **USB の vendor bulk、HID、USB CDC、USB-Serial/JTAG、UART bridge** のどれでも運べる。シリアルの口は、OEP のフレームと target の
  コンソールを 1 本で運ぶ。
- probe は線のことだけを知り、**target が何か**（flash の配置、ローダー）は **host が持つ**。

このライブラリは、ESP32-P4、classic ESP32、RP2350、RP2040 をその probe にします。`examples/` は、手元の治具のための、そのまま
使える firmware です。

- 仕様: [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec) — まず [レビューの手引き](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/docs/review-guide.ja.md) から。プロトコルの本体は
  [docs/oep-core.ja.md](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/docs/oep-core.ja.md)、番号の表は [registry/oep-v1.toml](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/registry/oep-v1.toml)。
- host のライブラリ: [oep-client-python](https://github.com/Open-Embedded-Probe/oep-client-python)（`pip install oep-client-python`、
  `oep` の命令、試験のための偽の probe）。
- USB の VID:PID: pid.codes の PID が割り当てられるまで、参照の firmware は `303a:0002` と `OEP` で始まる iProduct を使う。割り当て後に
  OEP の PID を使ってよい条件は [PID-USE.ja.md](PID-USE.ja.md)。

## このライブラリ

Open Embedded Probe（OEP）の probe を Arduino で書くためのライブラリと、各 probe のファームウェア（`examples/`）。
v1（oep-spec の `docs/oep-core.ja.md` と標準インターフェースの `docs/oep-if-*.ja.md`、固める候補の形）を話す。破壊的変更を前提とする実験段階で、互換は約束しない。

wire 上の数値は oep-spec の `registry/oep-v1.toml` が唯一の定義で、その生成物を `src/OepRegistry.h` に写している。
OEP を初めて読む人は oep-spec の `docs/review-guide.ja.md`（どこに何が書いてあるか）から。

## 構成

| PATH | 中身 |
|---|---|
| `src/Oep.h`、`src/OepEndpoint.*`、`src/OepRegistry.h` | v1 の本体（oep-core）: フレーム、名前で探すインターフェース、ロック、複数の経路（describe の transport）、シリアルの口の共用（core §3.4）、plan、通知 |
| `src/OepBind.*` | シリアルの口に流すもの（bind: last-reset / manual / mixed、セッション中の停止と最後の reset からの再開） |
| `src/OepStream.h`、`src/OepDebug.h` | 標準インターフェースの共通部品（位置つきのストリーム、線と target の status とピンの組） |
| `src/OepTarget.*`、`src/OepSwd.*`、`src/OepConsole.*`、`src/OepFixture.*`、`src/OepCapture.*`、`src/OepSampler.*`、`src/OepConfig.*` | 標準インターフェース: 線と target（`oep.wire.rvswd` / `swio` / `swd`、`oep.target.riscv-dm` / `arm-adi`）、コンソール、fixture（gpio / uart / capture）、`oep.probe.config`（スロット、bind、NVS への保存。ESP32）。各ファイルの冒頭に対応する仕様の節がある |
| `src/OepP4I2cTarget.*`、`src/OepP4SpiTarget.*` | 独自インターフェース `io.github.ch32-riscv-ug.esp32.i2c-target` / `spi-target`（revision 1、ESP-IDF の I2C / SPI スレーブ） |
| `src/OepCh32Dm.*`、`src/OepRvswdPhy.*`、`src/OepSwioPhy.*`、`src/OepDmConsole.*`、`src/OepPinTable.h`、`src/OepPlatform.h`、`src/OepFrame.*` など | 部品（CH32 のデバッグモジュール、線の物理層、コンソールの framing、ピンの表と空きの状態、Arduino の core の差、フレーム） |
| `examples/` | probe のファームウェア（ESP32-P4 + X035、classic ESP32 + V003、RP2350 / RP2040、P4 HS のキャプチャ `Esp32P4CaptureProbe` と `host/stream_test.py`） |
| `tests/host/` | 移植できる部分（シリアルの口の読み、endpoint の共用の規則、bind）の host の試験: `tests/host/run.sh`（g++） |
| `tools/sync_registry.sh` | oep-spec の `generated/oep-v1/oep_v1_registry.h` を `src/OepRegistry.h` に写す |
| `tools/bump_version.py`、`tools/sync_release_assets.py`、`.github/workflows/release.yml` | リリース（arduino-library-release-toolkit のものをそのまま使う。編集しない） |
| `docs/` | 日付入りの作業記録（経緯） |

## 使い方

firmware は**使う前に転送する**（ボードには何が入っているか分からない）。例: X035 の治具。

```sh
arduino-cli compile --clean examples/Esp32P4X035Probe
arduino-cli upload -p /run/board-identify/by-id/esp32-series-30eda0e31108 examples/Esp32P4X035Probe
```

host は [oep-client-python](https://github.com/Open-Embedded-Probe/oep-client-python)（`pip install oep-client-python`、
`oep_client.link.open_host()`）。シリアルの口（USB-Serial/JTAG、
USB CDC、UART bridge）は OEP のフレーム（`0x00 <COBS> 0x00`）と bind の生のバイトを 1 本で運ぶ。host は排他（TIOCEXCL）で開き、
フレームの外は雑音として捨てる（oep-spec の host 開発ガイド §1.6、§2）。

シリアルの口に target のコンソールを流すには、スロットと bind を登録する（`oep.probe.config`、ESP32 は NVS に保存）:

```sh
oep config slot <probe> --name x035 --wire rvswd --pins 2,54 --attach at-boot --retry 1 --mechanism dmseq
oep config bind <probe> --port 1 --mode last-reset --stream slot:x035 --save
oep config show <probe>
```

## 既知の罠

- Arduino.h は `word(...)` を `makeWord(...)` の macro にしている。lambda や関数を `word` と名付けると引数がそのまま返る。
- ESP32-P4 の `RvswdPhy::begin` の後に同じピンへ `pinMode` / `digitalWrite` を使うと、dedicated GPIO の束から外れて戻らない（chip の reset が要る）。
- direct build（`build_opt.h` で EspUsbDevice の vendor を直接書く形）のスケッチは、`build_opt.h` を変えたら `--clean` でビルドする。
- vendor bulk の OUT は packet ごとに受ける（`CFG_TUD_VENDOR_RX_NEED_ZLP=0`）。16 KiB の転送で受けて ZLP で終わらせる形
  （`=1`）では、ちょうど 512 byte の倍数で終わる要求（1024 byte の write_block など）の完了が、P4 では次の OUT が来るまで
  遅れ、probe は 3 s 答えなかった（2026-09-30、X035 の治具）。

## リリース

[arduino-library-release-toolkit](https://github.com/tanakamasayuki/arduino-library-release-toolkit) の共通の仕組みをそのまま使う。
変更は `CHANGELOG.md` の `## Unreleased` に (EN) / (JA) で書き足し、GitHub Actions の Release（workflow_dispatch）を起動すると、
`library.properties` の版を上げ、`src/openembeddedprobe_version.h` を作り、`release` ブランチで example の `sketch.yaml` の `dir: ../..` を
`OpenEmbeddedProbe (<版>)` に書き換え、`tests/` を除いた ZIP、tag、GitHub Release を作る。続けて、別の `.github/workflows/firmware.yml`（toolkit のものではない）が
tag から各 example をビルドし、`<Example>-<version>.merged.bin`（ESP32、0x0 に書く）/ `.uf2`（RP2040 / RP2350）と
`firmware-<version>.json`（sha256）を Release に付ける。probe は同じ版を describe の firmware の文字列で返す。
