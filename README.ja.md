# OpenEmbeddedProbe（OEP の probe の Arduino ライブラリ）

[English](README.md)

## OEP とは

Open Embedded Probe（OEP）は、**probe**（開発中のチップにつなぐ小さな基板）と、PC の **host** のソフトウェアの間の、オープンな
プロトコルです。1 つの probe が、デバッガ（WCH の CH32 の RVSWD / SWIO、ARM の SWD）、target のコンソール、試験の治具（GPIO、
UART、SPI / I2C のデバイス、ロジックのキャプチャ）を兼ね、どの host（書き込みの道具、IDE のモニタ、pytest）も同じ方法で話します。

**プロトコルは決まっていて、probe の能力は決まっていません。** OEP が定めるのは、要求と結果、ロック、発見の仕組みです。
何ができるかは開いています。

- probe は**自分にできることを宣言する**（名前で探すインターフェースと、そのピンや上限）。host はボードの表を持たず、probe が
  持つものに合わせて動く。
- 標準のインターフェース（`oep.wire.*`、`oep.target.*`、`oep.fixture.*` など）は oep-spec が定める。それとは別に、**誰でも
  自分のインターフェースを足せる**。自分の逆 DNS の名前（`io.github.<you>.<name>`、revision も自分で持つ）で出せば、誰の
  許可も要らない。知らない host は使わないだけである。このライブラリの ESP32 の SPI / I2C デバイスは、その拡張の例。
- **セッションのロック**で、2 つのプログラムが同時に probe を動かすのを防ぐ。
- **USB の vendor bulk、HID、USB CDC、USB-Serial/JTAG、UART bridge** のどれでも運べる。シリアルの口は、OEP のフレームと target の
  コンソールを 1 本で運ぶ。
- probe は線のことだけを知り、**target が何か**（flash の配置、ローダー、debug の線をどう動かすか）は **host が持つ**。

このライブラリは、ESP32-P4、classic ESP32、RP2350、RP2040 をその probe にします。

- 仕様: [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec) — まず [レビューの手引き](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/docs/review-guide.ja.md) から。プロトコルの本体は
  [docs/oep-core.ja.md](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/docs/oep-core.ja.md)、番号の表は [registry/oep-v1.toml](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/registry/oep-v1.toml)。
- host のライブラリ: [oep-client-python](https://github.com/Open-Embedded-Probe/oep-client-python)（`pip install oep-client-python`、
  `oep` の命令、試験のための偽の probe）。
- USB の VID:PID: pid.codes の PID が割り当てられるまで、参照の firmware は `303a:0002` と `OEP` で始まる iProduct を使う。割り当て後に
  OEP の PID を使ってよい条件は [PID-USE.ja.md](PID-USE.ja.md)。

## 利用例: ESP32-P4 で CH32L103 を試験する

<!-- ここに ESP32-P4 と CH32L103 のフル結線の写真を入れる。 -->

ESP32-P4 の基板 1 枚を、見たいピンをすべて CH32L103 につないでおけば、それだけで試験台になります。

- **target の書き込みとデバッグ**: RVSWD で書き込み、止める・走らせる・1 命令ずつ進める、メモリを読み書きする
  （`oep.wire.rvswd`、`oep.target.riscv-dm`）。[ArduinoCore-CH32](https://github.com/ch32-riscv-ug/ArduinoCore-CH32) なら、Arduino IDE から
  probe 経由で書き込める（`oep://...` のポート）。
- **コンソールを読む**: debug module 経由（`oep.target.console`、UART 不要）か UART（`oep.fixture.uart`）。
- **target のコードがピンで何をしているかを確かめる**: P4 が相手側のデバイス（SPI のデバイス、I2C のデバイス）として受けるので、
  target のドライバが本当に送ったバイトを試験で見られる。GPIO や UART で target に返すこともできる（`oep.fixture.gpio` / `uart`、
  `io.github.ch32-riscv-ug.esp32.spi-target` / `i2c-target`）。
- **最大 16 ピンを同時にキャプチャ**: SPI のデバイスとして動かしながらでも取れる（`oep.fixture.capture`、P4 の PARLIO）。

  | ch 数 | サンプリング |
  |---:|---:|
  | 2 | 160 Msps |
  | 8 | 40 Msps |
  | 16 | 20 Msps |

  どのピンをキャプチャするかは要求ごとに選ぶ（plan）。試験項目ごとに必要なピンだけを取れ、試験の間に配線も firmware も変えなくて
  よい。取ったサンプルは sigrok のセッションファイル（`LogicCapture.to_sr`、PulseView で開ける）や、
  [WireSkein](https://github.com/Open-Embedded-Probe/wireskein) のような実行記録に渡せるので、結果だけでなく信号のレベルで試験できる。

上のことはすべて、実行中に host が選びます。同じ firmware が、どの試験にも使えます。

## 電気的な注意

このライブラリは信号のレベルに関与しません。probe のピンは MCU のピンそのものです。

- ESP32-P4 は 3.3 V 系。5 V 系の target につなぐときは、間に**双方向で高速に動くレベルコンバータ**などを入れる。
- 受けるだけの線（キャプチャなど）は、間に**高速なバッファ**を入れれば 5 V 系とつなげる。
- 入力の前に**高速なコンパレータ**を置けば、レベルの判定を可変にできる。ESP32-P4 は可変の LDO を持つので、その判定の電圧に
  外付けの部品は要らない。

## このライブラリ

Open Embedded Probe（OEP）の probe を Arduino で書くためのライブラリと、各 probe のファームウェア（`examples/`）。
v1（oep-spec の `docs/oep-core.ja.md` と標準インターフェースの `docs/oep-if-*.ja.md`、固める候補の形）を話す。破壊的変更を前提とする実験段階で、互換は約束しない。

wire 上の数値は oep-spec の `registry/oep-v1.toml` が唯一の定義で、その生成物を `src/OepRegistry.h` に写している。

## 構成

| PATH | 中身 |
|---|---|
| `src/Oep.h`、`src/OepEndpoint.*`、`src/OepRegistry.h` | v1 の本体（oep-core）: フレーム、名前で探すインターフェース、ロック、複数の経路（describe の transport）、シリアルの口の共用（core §3.4）、plan、通知 |
| `src/OepBind.*` | シリアルの口に流すもの（bind: last-reset / manual / mixed、セッション中の停止と最後の reset からの再開） |
| `src/OepStream.h`、`src/OepDebug.h` | 標準インターフェースの共通部品（位置つきのストリーム、線と target の status とピンの組） |
| `src/OepTarget.*`、`src/OepSwd.*`、`src/OepConsole.*`、`src/OepFixture.*`、`src/OepCapture.*`、`src/OepSampler.*`、`src/OepConfig.*` | 標準インターフェース: 線と target（`oep.wire.rvswd` / `swio` / `swd`、`oep.target.riscv-dm` / `arm-adi`）、コンソール、fixture（gpio / uart / capture）、`oep.probe.config`（スロット、bind。ESP32 は NVS、RP2040 / RP2350 は flash に保存）。各ファイルの冒頭に対応する仕様の節がある |
| `src/OepP4I2cTarget.*`、`src/OepP4SpiTarget.*` | 独自インターフェース `io.github.ch32-riscv-ug.esp32.i2c-target` / `spi-target`（revision 1、ESP-IDF の I2C / SPI スレーブ）。OEP を拡張する例 |
| `src/OepCh32Dm.*`、`src/OepRvswdPhy.*`、`src/OepSwioPhy.*`、`src/OepDmConsole.*`、`src/OepPinTable.h`、`src/OepPlatform.h`、`src/OepFrame.*` など | 部品（CH32 のデバッグモジュール、線の物理層、コンソールの framing、ピンの表と空きの状態、Arduino の core の差、フレーム） |
| `examples/Firmware/OepProbe` | ボードの firmware。チップごとに 1 本（profile rp2040、rp2350、esp32p4）で、ピンはすべて host が選ぶ。RP2 は RVSWD、SWD、gpio、uart。ESP32-P4 は RVSWD、gpio、uart x2、capture、SPI / I2C デバイス。設定は保存できる |
| `examples/` | probe のファームウェア（ESP32-P4 + X035、classic ESP32 + V003、RP2350 / RP2040、P4 HS のキャプチャ `Esp32P4CaptureProbe` と `host/stream_test.py`）。使う人が要るものから並べ直し中: [docs/examples-and-firmware-plan.ja.md](docs/examples-and-firmware-plan.ja.md) |
| `tests/host/` | 移植できる部分（シリアルの口の読み、endpoint の共用の規則、bind）の host の試験: `tests/host/run.sh`（g++） |
| `tools/sync_registry.sh` | oep-spec の `generated/oep-v1/oep_v1_registry.h` を `src/OepRegistry.h` に写す |
| `tools/bump_version.py`、`tools/sync_release_assets.py`、`.github/workflows/release.yml` | リリース（arduino-library-release-toolkit のものをそのまま使う。編集しない） |
| `docs/` | 日付入りの作業記録（経緯） |

## 使い始める

1. **firmware**: [Releases](https://github.com/Open-Embedded-Probe/oep-probe-arduino/releases) のビルド済みを使う
   （どの RP2040 / RP2350 のボードにも `OepProbe-rp2040-<version>.uf2` / `OepProbe-rp2350-<version>.uf2` を BOOTSEL のボードに
   コピー。ESP32-P4 は `OepProbe-esp32p4-<version>.merged.bin`。ピンは host が選ぶ。ESP32 は `<Example>-<profile>-<version>.merged.bin` を `esptool.py write_flash 0x0 <file>`。sha256 は
   `firmware-<version>.json`）。自分でビルドするなら、Arduino のライブラリマネージャーで
   **OpenEmbeddedProbe** を入れ、`ファイル > スケッチ例 > OpenEmbeddedProbe` を開き、その `sketch.yaml` の profile（core の版と
   ライブラリを固定してある）でビルドする:

   ```sh
   arduino-cli compile --clean examples/Esp32P4X035Probe
   arduino-cli upload -p <port> examples/Esp32P4X035Probe
   ```

   firmware は**使う前に転送する**（ボードに何が入っているかは分からない）。
2. **host**: `pip install oep-client-python` の後、probe が何を持つかを見る:

   ```sh
   oep dump --port <port>
   ```

   Python からは `oep_client.link.open_host(<port>)` でセッションを得る。シリアルの口（USB-Serial/JTAG、USB CDC、UART bridge）は
   OEP のフレーム（`0x00 <COBS> 0x00`）と bind の生のバイトを 1 本で運ぶ。host は排他（TIOCEXCL）で開き、フレームの外は雑音として
   捨てる（oep-spec の host 開発ガイド §1.6、§2）。
3. **設定**: 起動時から target のコンソールをシリアルの口に流すには、スロットと bind を登録する（`oep.probe.config`、ESP32 は
   NVS、RP2040 / RP2350 は flash に保存）。線の設定は target のもので、host が渡す（CH32L103 は、休ませる間は SWCLK を low、reset 直後は 1 MHz まで）:

   ```sh
   oep config slot <probe> --name l103 --wire rvswd --pins 2,54 --attach at-boot --retry 1 --mechanism dmseq \
       --max-speed 1000000 --idle-clock low
   oep config bind <probe> --port 1 --mode last-reset --stream slot:l103 --save
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
tag から各 example の各 profile をビルドし、`<Example>-<profile>-<version>.merged.bin`（ESP32、0x0 に書く）/ `.uf2`（RP2040 / RP2350）と
`firmware-<version>.json`（sha256）を Release に付ける。probe は同じ版を describe の firmware の文字列で返す。

## ライセンス

MIT（[LICENSE](LICENSE)）。各ソースファイルにも表記がある（`SPDX-License-Identifier: MIT`）。USB の VID:PID はこれに含まれない:
[PID-USE.ja.md](PID-USE.ja.md)。
