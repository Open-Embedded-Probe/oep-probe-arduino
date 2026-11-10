# firmware の機能を選ぶ、自分のものを足す

[English](firmware-features.md)

配布している firmware は、ライブラリの部品 `OepFirmware.h` です。そのチップでできることを既定で全部入れています。
スケッチは `begin()` を呼ぶだけで、ライブラリの版を上げれば firmware の本体も新しくなります。この手引きでは、次の 2 つを
説明します。

- 要らない機能を外す、要るものだけで組む（小さいチップに収める、使わないものを出さない）
- 自分のインターフェースを足す（firmware の残りはそのまま）

## 1. いちばん短いスケッチ

```cpp
#include <OepFirmware.h>

void setup() { oep::firmware.begin(); }   // そのチップの既定の機能を全部

void loop() { oep::firmware.loop(); }
```

配布の [OepProbe](../../examples/Firmware/OepProbe/OepProbe.ino) も、この形です。

- `OepFirmware.h` は、スケッチの `.ino` の 1 か所だけで含めます。2 つのファイルで含めると、リンクで止まります。
  ほかのファイルからは `OepFirmwareParts.h`（機能の名前と、下の `add` / `onSetup` / `onLoop`）を含めます。
- ESP32-P4 / P4X では、スケッチの `sketch.yaml` に EspUsbDevice のライブラリを入れます（配布の OepProbe の profile と同じ）。

## 2. 機能を選ぶ

機能は `begin<F>()` の F（コンパイル時の定数）で選びます。F に入れなかった機能は、コードが何もリンクされません。

```cpp
oep::firmware.begin<oep::kDefaults & ~oep::kSwd & ~oep::kPwm>();          // 既定から SWD と PWM を外す
oep::firmware.begin<oep::kDefaults | oep::kCdc>();                         // 既定に、既定で外れている USB CDC を足す
oep::firmware.begin<oep::kNone | oep::kBulk | oep::kRvswd | oep::kConsole>(); // 全部なしから、これだけ
```

| 名前 | 中身 |
|---|---|
| `kBulk`、`kHid`、`kCdc`、`kUsj`、`kBridge`、`kWifi` | 経路: USB vendor bulk、HID、CDC、USB-Serial/JTAG、UART ブリッジ、Wi-Fi の TCP |
| `kDfu`、`kPortSpeed` | ESP32-P4 の DFU（OEP の外）、UART ブリッジの port_speed |
| `kRvswd`、`kSwio`、`kSwd` | debug の線（と、その target） |
| `kConsole`、`kSwioConsole` | target のコンソール（RVSWD の線のもの、SWIO の線のもの） |
| `kGpio`、`kUart`、`kI2cTarget`、`kSpiTarget` | fixture: GPIO、UART、I2C / SPI の target |
| `kI2cController`、`kSpiController`、`kPwm`、`kCounter`、`kDac` | fixture: I2C / SPI の controller、PWM、エッジの計数、DAC |
| `kCapture`、`kLogic`、`kAnalog`、`kGroup` | キャプチャ: ロジック（`kCapture` は ESP32-P4 / classic ESP32、`kLogic` は RP2 / CH32）、アナログ、同時に始める group |
| `kSettings`、`kLink`、`kMemo` | core の設定（保存）、`oep.probe.link`（線の試験）、`oep.probe.memo` |

- `oep::kDefaults` は配布の firmware の機能、`oep::kAvailable` はそのチップが持つ機能です。チップごとの中身は
  [ボード](boards.ja.md) にあります。
- 次の組はコンパイル時に止まり、理由が出ます: そのチップが持たない機能、経路が 1 つも無い、`kConsole` に `kRvswd` が無い、
  `kSwioConsole` に `kSwio` が無い、`kPortSpeed` に `kBridge` が無い。
- 外すと、それより後のインターフェースの fn の番号が変わります。host は名前で探すので困りませんが、保存した設定は
  そのインターフェースの並びのときにだけ読み戻せます。外したら設定を入れ直してください。

## 3. 実行時の値

名前は `FirmwareConfig` で変えられます。書かなかったものは配布の firmware の値です。

```cpp
oep::FirmwareConfig cfg;
cfg.model = "my-jig";              // fn 0 の describe の model
cfg.product = "My jig";            // USB の製品名（人が見る名前。host はこれでプローブを見分けない）
oep::firmware.begin(cfg);          // begin<F>(cfg) も同じ
```

## 4. 自分のインターフェースを足す

`begin()` の前に登録します。どのファイルからでも呼べます（`OepFirmwareParts.h`）。

```cpp
#include <OepFirmware.h>

class Hello final : public oep::Interface { /* はじめてのインターフェースの Hello */ };
static Hello hello;

void setup() {
  oep::firmware.onSetup([](oep::Endpoint &endpoint, oep::PinTable &pins) {
    pins.claim(4, oep::owners::kSketch);   // 自分が使うピン
  });
  oep::firmware.add(hello);                // firmware の部品の後ろに足す
  oep::firmware.onLoop([] { /* loop() で進める仕事があれば */ });
  oep::firmware.begin();
}

void loop() { oep::firmware.loop(); }
```

- `add` したインターフェースは、firmware の部品をすべて足した後に、登録の順に足されます。保存した設定はその後で適用されます。
- `onSetup` は `begin()` の中で、自分のインターフェースを足す直前に呼ばれます。ピンは渡された `pins` から取り、持ち主の番号は
  `oep::owners::kSketch`（0x80）から上を使います。
- `onLoop` は毎回の `loop()` で、firmware 自身の仕事の後に呼ばれます。待たないでください。
- 登録はヒープに数 byte ずつ取ります（数の上限はありません）。
- インターフェースの書き方は [はじめてのインターフェース](your-first-interface.ja.md) を見てください。

例: [extras/LibraryTest](../../extras/LibraryTest/README.ja.md) は、試したいライブラリの呼び出しをプローブの上で動かし、そのログ
（`OepProbeLog.h`: RAM に書いてプローブの時計を付ける）をキャプチャの波形に紐付ける例です。

## 5. ビルドする

- **Arduino IDE**：新しいスケッチに上の形を書きます（`ファイル > スケッチ例 > OpenEmbeddedProbe > Firmware > OepProbe` から始めても
  同じです）。
- **arduino-cli**：配布の `OepProbe` の `sketch.yaml` を自分のスケッチに写し、`libraries: - dir:` をこのライブラリの場所に直します
  （元は `../../..`）。そのうえで `arduino-cli compile --profile <profile> <フォルダ>` を実行します。
- 大きさに気を付けてください。CH32X035 の 62 KiB は既定の構成でほぼ埋まっています。何かを外して空けてください。

## 6. 確かめる

`oep dump --port <port>` で、足したインターフェースが list に載り、外した部品が消えていることを確かめます。
