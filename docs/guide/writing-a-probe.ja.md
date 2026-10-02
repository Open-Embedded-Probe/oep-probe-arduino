# probe を書く

[English](writing-a-probe.md)

ライブラリの組み立てと、それを使って自分の probe や自分のインターフェースを作る方法。example を開いて読んでください。
`01.Basics/MinimalProbe` が骨組み、`02.Interfaces/CustomInterface` がインターフェース、`Firmware/OepProbe` が全部を組み合わせた
ものです。

## 1. probe の形

```text
 host  ==フレーム==>  経路（Stream） --> Endpoint --> インターフェース（fn 1）   oep.wire.rvswd
                      経路 2 .......        |     --> インターフェース（fn 2）   oep.target.riscv-dm
                                             |     --> インターフェース（fn 3）   io.github.you.thing
                                     oep.core（fn 0）: confirm、list、describe、ロック、plan、購読
```

- **Endpoint** がプロトコルを持ちます。各経路からフレームを読み、セッションとロックを確かめ、fn でインターフェースを探し、
  要求が来た経路に結果を返します。oep.core（fn 0）は組み込みです。
- **インターフェース**は、名前と操作を持つクラスです。endpoint は `add()` した順に番号を振ります（fn 1、2、...）。host は番号
  ではなく名前で探します。
- スケッチはこれらの配線です。オブジェクトを置き、`setup()` で足し、`loop()` で `poll()` を呼びます。

## 2. endpoint

```cpp
static uint8_t rx[1100], tx[1024];
static oep::Endpoint endpoint(Serial, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, oep::Endpoint::kUsbCdc);
```

- **経路**はどの `Stream` でもよい。**種類**がフレームの形を決めます。シリアルの口（`kUsbCdc`、`kUsbSerialJtag`、
  `kUartBridge`）は CRC 付きの `0x00 <COBS> 0x00` のフレームを運び、その間に生のバイト（bind、§7）も運べます。メッセージの
  経路（`kVendorBulk`、`kHid`、`kTcp`）は長さを前に付けたメッセージを運びます。経路を足すのは
  `endpoint.addTransport(stream, rx, sizeof rx, kind, usb_interface)`（`03.Transports/MultipleTransports`）。
- **上限** `{max_frame, window, max_inflight}` は、confirm で host に約束する値です。シリアルの口の rx は、符号化したフレームを
  入れるので max_frame より少し大きくします（`cobsFrameMax`）。
- oep.core の **describe** は自分で書きます。`describeCore(w, model, unit_id, ...)` が、firmware の版、model、どの経路でも同じ
  unit id（`platformUnitId`）、channel の数、予約の channel を書きます。`setProbeDescription` で渡します。経路の一覧と
  transport、`discoverable`、`plan_roles`、`max_op_ms` は endpoint が足します。describe は宣言だけです（core §7.3）:
  動いている間に変わるものは入れません。
- `setBootId(platformRandom32())` を 1 度。host は probe が起動し直したことを知ります。
- `loop()` から `poll()`。長く止まりません。インターフェースも止まってはいけません。

## 3. インターフェース

```cpp
class Blink final : public oep::Interface {
 public:
  const char *name() const override { return "io.github.you.blink"; }   // 自分の逆 DNS の名前。oep.* は使わない
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }                      // 固定部分の形
  bool lockFree(uint8_t op) const override { return op == 0x03; }      // 何も変えない読み出し
  size_t describe(uint8_t *out, size_t capacity) override;             // 使う前に host が読むもの
  oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
};
```

- **名前**で host が探します。標準のインターフェースは `oep.*`（oep-spec が定める）で、自分のものは自分が持っている名前の
  逆 DNS（`io.github.<you>.<name>`）にします。誰の承認も要らず、知らない host は使わないだけです。これが OEP の拡張の仕方です。
- **revision** は、すべての payload の固定部分の形を決めます。形を変えたら上げます。任意の TLV を足すだけなら変えません。
- **handle** は要求を 1 つ受け、結果の payload を `out` に書きます。
  - `completed(n)`: 実行して成功した。payload は n バイト。
  - `failed(n)` / `partial(n)`: 実行したが、うまくいかなかった（全部 / 一部）。payload は成功のときの形。
  - `rejected(reason)`: 実行していない。`kRejectMalformed`、`kRejectUnavailable`、`kRejectUnsupported`、`kRejectUnknownOperation`。
- **TLV の後ろの部分**: どの要求も最後に TLV を付けられます。`plainTail(tail, payload, length, fixed, out, capacity)` が固定部分の
  後ろを読みます。知らない critical の TLV は要求を断り（`refused()` で分かる）、それ以外の知らない TLV は覚えておき、
  `tail.finish(result, out, capacity)` が ignored として返します。知っている tag があるときは `tail.parse(...)`、
  `tail.find(tag, len, &critical)`、守れない値には `tail.refuse(tag, critical, ...)`。
- **ロック**は endpoint が見ます。`lockFree` でない op は、ロックを持つセッションにだけ実行されます。そのセッションのリースが
  切れると `sessionLapsed()` が呼ばれるので、残したもの（線の接続など）を片付けます。
- **describe** は `TlvWriter` で書きます。共通の tag（`roleChannels`、`u32(kTagMaxClockHz, ...)`、`u32(kTagFeatures, ...)`）と、
  自分の tag（0x40 から）。

## 4. ピン: 表と plan

- `oep::PinTable pins(mask)` は、スケッチがインターフェースに渡す channel（GPIO の番号）の集まりです。インターフェースは使う
  ものを自分の持ち主の番号で `claim()` し、`release()` で返します。持たれている channel はほかには断られるので、2 つの
  インターフェースが同じピンを駆動することはありません（core §8.1）。返した channel は空きのときの状態（Hi-Z、または設定の
  とおり）になります。
- **plan** は、host が実行中にピンを割り当てる方法です（core §8）。`plan_apply` が (fn, role, channel) を挙げると、endpoint は
  各インターフェースに `planCheck()`（副作用なし: 0 か断る理由）、次に `planApply()` を聞き、`planRelease()` で返させます。
  役とその候補のピンは describe で宣言します（`roleChannels`）。
- 1 つの表を共有するインターフェースの持ち主の番号は、重ならないようにします（Firmware のスケッチに一覧があります）。

## 5. このライブラリの標準インターフェース

| クラス | インターフェース | 備考 |
|---|---|---|
| `WireRvswd` + `DebugPort` + `Ch32Dm` + `RvswdPhy` | `oep.wire.rvswd` | RP2 の SIO、または ESP32 の dedicated GPIO で RVSWD |
| `WireRvswd`（名前は `oep.wire.swio`）+ `SwioPhy` | `oep.wire.swio` | 1 本線。classic ESP32 |
| `TargetRiscvDm` | `oep.target.riscv-dm` | halt / resume / step / reset、block の読み書き、run。target を走らせる前に、target のレジスタと DATA0/1 を戻す |
| `TargetConsoleStream` + `DmConsole` | `oep.target.console` | debug module を通した target のコンソール（dmseq、DMDATA、SDI） |
| `WireSwd` + `SwdPort`、`TargetArmAdi` | `oep.wire.swd`、`oep.target.arm-adi` | ARM の SWD。RP2040 / RP2350 |
| `FixtureGpio`、`FixtureUart` | `oep.fixture.gpio` / `uart` | plan で決めたピンで |
| `LogicCapture` / `SamplerCapture` | `oep.fixture.logic` | ESP32-P4 の PARLIO / classic ESP32 の GPIO の sampler。レベル / エッジのトリガ |
| `AnalogCapture` | `oep.fixture.analog` | ADC のチャネルを順に、ワンショット、生の値、しきい値のトリガ（ESP32 は連続変換、RP2 は FIFO + DMA）。frontend、基準電圧、出荷時の較正 |
| `CaptureGroup`（+ `GroupTrack`） | `oep.fixture.capture-group` | トラックを一緒に始める。`GroupTrack` を持つ capture を束ねられる |
| `P4I2cTarget`、`P4SpiTarget` | `oep.fixture.i2c-target` / `spi-target` | ESP-IDF の I2C / SPI スレーブで実装 |
| `ProbeConfig` + `Binds` | `oep.probe.config` | flash に保存し、起動時に行う設定（§7） |

各ソースファイルの冒頭に、従う仕様の節があります。

## 6. debug の線: 決まった組か、host が選ぶピンか

- 決まった組: `DebugPort port{dm, swdio, swclk}` と `phy.begin(swdio, swclk)`（`04.Debug/*`）。
- host が選ぶピン: 組を決めずに `port.pin_choice = mask; port.pins = &pins;`。線は channel を role_channels で宣言し、scan / attach
  で host が名指した空いている組ならどれでも受けて、PHY をそこへ動かし（`usePins`）、生きている接続はそのピンを表で持ちます。
  `reset_allowed` は attach の reset TLV が引いてよい channel で、いつも host が名指します。
- 線の設定（休ませ方、速さの上限）は target のもので、host が attach で渡します。probe はチップごとの既定を持ちません。

## 7. シリアルの口、bind、設定

- シリアルの口は OEP のフレームと、その間の生のバイトを運びます。生のバイトは **bind** のとおり（スロットのコンソール、fixture の
  UART、名前の印つきの複数）です。`endpoint.setRawPorts(&binds)` で有効になります。セッションがロックを持つ間、それが使う口の
  生の流れは止まり、終わった後に target の最後の reset から続きます（core §3.4）。
- `ProbeConfig` はスロット、bind、plan、label、空きのときの状態、fixture UART の設定（uart の項目）、無効にした channel
  （disable の項目）を持ち、保存し（ESP32 は NVS、RP2 は flash の最後の領域）、起動時に行います。`state`（op 0x06）がスロットと
  bind の状態を、`unset`（0x05）がキーでの削除です。最後に `add(config)` し、`addPlace(wire, console)`、`addUart(uart)`、
  `setPins(&pins)`、`applySaved()`（`06.Settings/ProbeConfig`）。`load()` はスケッチがピンを空きの状態にする前に呼び、保存が無効に
  した channel を除きます: `config.load(); pins.setDisabled(config.savedDisabled()); platformParkMask(mask & ~pins.disabledMask());`
  （無効にした channel には起動時も触れない）。PinTable は設定の無効（外せる）と `forbid`（firmware のもの、ずっと）を分けて持ち、
  どの項目も forbid したピンを使えるようにはしません。

## 8. push と出来事

流すインターフェースは `subscribe()`、`pull()`、`pending()` を持ちます。endpoint は、購読した host がロックを持つ間、データの
フレームを送ります。`endpoint.event(*this, kind, payload, length)` で出来事を送ります。コンソール、fixture の UART、capture が
これを使っています。

## 9. USB の名乗り

USB の probe は、`OEP` で始まる iProduct（表示の自由な文字列。プロジェクトの VID:PID ができるまでの host の暫定の手がかり、
oep-spec host 開発ガイド §1.7）、個体ごとに違う serial number（unit_id。名指した probe を host はこれで探す）、VID:PID を名乗ります。
VID:PID はボードの既定（ESP32-P4 では `303a:0002`）です。これは仮の USB の ID で、配布には使えません。専用の PID を取得できたら、
それに切り替える予定です（[PID-USE.ja.md](../../PID-USE.ja.md)）。
vendor bulk のインターフェースは bInterfaceSubClass 0x4F / bInterfaceProtocol 0x45、vendor HID は usage page 0xFF4F /
usage 0x45 を持ちます（core §3.3。`Firmware/OepProbe/Esp32P4.h` が EspUsbDevice の記述子をそう直します）。
`endpoint.setDiscoverable(true)`（describe の discoverable、core §7.5）を呼ぶのは、プロジェクトの VID:PID で列挙する probe だけです。
その VID:PID が registry に載るまでは、どの probe も呼びません。

## 10. 試す

- `tests/host/run.sh` は、移植できる部分を PC でビルド（g++）して試します: シリアルの口の読み、endpoint の規則（ロック、plan、
  共用）、bind。
- host の側には、仕様どおりに答える偽の probe があります（`oep-client-python`: `python -m oep_client.fake_serve`）。probe が
  無くても host を試せ、自分の probe の答えと比べられます。
- 実機では、`oep dump --port <port>` が probe の宣言をすべて読みます。その後、Python から動かします。

## 11. はまりどころ

- `Arduino.h` は `word(...)` を macro にしています。`word` という名前の関数や lambda は引数をそのまま返します。
- ESP32-P4 で `RvswdPhy::begin` の後に RVSWD のピンへ `pinMode` / `digitalWrite` を使うと、dedicated GPIO の束から外れて戻り
  ません（chip の reset が要る）。PHY とピンの表はこれを避けています（`releaseQuiet`）。自分のコードでも避けてください。
- `build_opt.h` のあるスケッチ（P4 の HS vendor の direct build）は、それを変えたら `--clean` でビルドします。
- `handle()` や `loop()` で止まらないこと。poll しなくなった probe は答えなくなり、host は時間切れになります。
