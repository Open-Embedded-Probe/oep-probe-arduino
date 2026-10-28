# firmware の機能を選ぶ、自分のものを足す

[English](firmware-features.md)

配布している firmware（`examples/Firmware/OepProbe`）は、そのチップでできることを既定で全部入れています。この手引きでは、
次の 2 つを説明します。

- 要らない機能を外す（小さいチップに収める、使わないものを出さない）
- 自分のインターフェースを足す（firmware の残りはそのまま）

どちらも `OepProbe` のフォルダに自分の `.cpp` を 1 つ置くだけで、配布元のファイルは書き換えません。

## 1. 仕組み

firmware の部品（経路とインターフェース）は、それぞれが 1 つの選択です。

```cpp
// チップのヘッダ（Rp2.h など）の中
void oepAddPwm() { static oep::Slot<oep::FixturePwm> pwm; endpoint.add(pwm.make(pins)); }   // 部品を作って足す
__attribute__((weak)) void oepWithPwm() { oepAddPwm(); }                                       // 既定: 足す
```

- `oepAddX()` だけが部品 X を名指します。呼ばなければ、X のコードは何もリンクされません。
- `oepWithX()` は weak です。スケッチが同じ名前の関数を定義すると、そちらが使われます。

一覧は [OepFeatures.h](../../examples/Firmware/OepProbe/OepFeatures.h) にあります。チップごとに何が既定で入っているかは、
[OepProbe.ino](../../examples/Firmware/OepProbe/OepProbe.ino) の冒頭と [ボード](boards.ja.md) にあります。

## 2. 機能を外す・足す

`OepProbe` のフォルダに、たとえば `MyChoices.cpp` を置きます。

```cpp
#include "OepFeatures.h"

void oepWithSwd() {}                 // oep.wire.swd / oep.target.arm-adi を外す
void oepWithPwm() {}                 // oep.fixture.pwm を外す
void oepWithCdc() { oepAddCdc(); }   // 既定で外れている USB CDC を足す（そのチップに oepAddCdc があるとき）
```

| X | 中身 |
|---|---|
| `Bulk`、`Hid`、`Cdc`、`Usj`、`Bridge`、`Wifi` | 経路: USB vendor bulk、HID、CDC、USB-Serial/JTAG、UART ブリッジ、Wi-Fi の TCP |
| `Dfu`、`PortSpeed` | ESP32-P4 の DFU（OEP の外）、fn 0 の port_speed |
| `Rvswd`、`Swio`、`Swd` | debug の線（と、その target） |
| `Console`、`SwioConsole` | target のコンソール（RVSWD の線のもの、SWIO の線のもの） |
| `Gpio`、`Uart`、`I2c`、`Spi` | fixture: GPIO、UART、I2C / SPI の target |
| `I2cController`、`SpiController`、`Pwm`、`Counter`、`Dac` | fixture: I2C / SPI の controller、PWM、エッジの計数、DAC |
| `Capture`、`Logic`、`Analog`、`Group` | キャプチャ: ロジック（P4 / classic ESP32 / RP2 / CH32）、アナログ、同時に始める group |
| `Settings`、`Link` | core の設定（保存）、`oep.probe.link`（線の試験） |

- チップのヘッダに `oepAddX` が無い X は、そのチップでは足せません（リンクで失敗します）。
- 外すと、それより後のインターフェースの fn の番号が変わります。host は名前で探すので困りませんが、保存した設定は
  そのインターフェースの並びのときにだけ読み戻せます。外したら設定を入れ直してください。

## 3. 自分のインターフェースを足す

同じ `.cpp` に `oepUserSetup` と `oepUserLoop` を定義します。どちらも定義しなければ何もしません。

```cpp
#include <OepEndpoint.h>
#include <OepPinTable.h>
#include "OepFeatures.h"

class Hello final : public oep::Interface { /* はじめてのインターフェースの Hello */ };
static Hello hello;

void oepUserSetup(oep::Endpoint &endpoint, oep::PinTable &pins) {
  endpoint.add(hello);   // firmware の部品の後ろに足す
}
void oepUserLoop() {
  // hello に loop() で進める仕事があれば、ここで（待たないこと）
}
```

- `oepUserSetup` は `setup()` の中で 1 回、firmware の部品をすべて足した後に呼ばれます。保存した設定はその後で適用されます。
- `oepUserLoop` は毎回の `loop()` で、firmware 自身の仕事の後に呼ばれます。
- ピンを使うなら、渡された `pins` から取ります。持ち主の番号は `oep::owners::kSketch`（0x80）から上を使い、firmware の部品と
  重ならないようにします。
- インターフェースの書き方は [はじめてのインターフェース](your-first-interface.ja.md) を見てください。

## 4. ビルドする

- **Arduino IDE**：`ファイル > スケッチ例 > OpenEmbeddedProbe > Firmware > OepProbe` を開き、名前を付けて保存してから、自分の
  `.cpp` をタブで足します（保存したスケッチは、入れたライブラリを使います）。
- **arduino-cli**：`OepProbe` のフォルダを写して `.cpp` を足し、写した `sketch.yaml` の `libraries: - dir:` をこのライブラリの
  場所に直します（元は `../../..`）。そのうえで `arduino-cli compile --profile <profile> <フォルダ>` を実行します。
  ESP32-P4 は `--clean` を付けます（`build_opt.h`）。
- 大きさに気を付けてください。CH32X035 の 62 KiB は既定の構成でほぼ埋まっています。
  何かを外して空けてください。

## 5. 確かめる

`oep dump --port <port>` で、足したインターフェースが list に載り、外した部品が消えていることを確かめます。
