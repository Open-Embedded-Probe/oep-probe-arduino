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
                                             |     --> oep.probe.plan、oep.probe.restart（endpoint 自身のもの、最後）
                                     fn 0（本体、名前なし）: confirm、list、describe、clock、ロック
```

- **Endpoint** がプロトコルを持ちます。各経路からフレームを読み、セッションとロックを確かめ、fn でインターフェースを探し、
  要求が来た経路に結果を返します。fn 0（本体。名前を持たず list に載りません）は組み込みです。`oep.probe.plan`（plan の役を
  持つインターフェースがあれば出す）と `oep.probe.restart`（`setRestart` で出す）も組み込みで、endpoint は最初の `poll()` で、
  足したすべてのインターフェースの後にこの順で番号を振ります。同じ firmware ならどの起動でも同じ fn です。
- **インターフェース**は、名前と操作を持つクラスです。endpoint は `add()` した順に番号を振ります（fn 1、2、...）。host は番号
  ではなく名前で探します。インターフェースは少なくとも 1 つの op を持ちます。
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
- **TCP**: 待ち受けの socket が経路の entry 1 つ（kind 6）で、受けた接続はそれぞれ別の経路です。ほかの経路をすべて足した後に
  `endpoint.addTcpListener(tcp.slots(), rx, sizeof rx[0], n)`、`oep::TcpListener<n>`（`OepTcp.h`、lwIP の socket）は `loop()` で endpoint より先に
  poll します。接続の応答と通知はその接続に返り、max_frame を超える長さは接続を閉じ、閉じた接続は何も終わらせません（transports §1〜§3）。
  `OepWifi.h` は設定の wifi の項目のネットワークに入り、mDNS で `_oep._tcp` を名乗ります（`03.Transports/WifiTcp`）。
- **上限** `{max_frame, window, max_inflight}` は、confirm で host に約束する値です。シリアルの口の rx は、符号化したフレームを
  入れるので max_frame より少し大きくします（`cobsFrameMax`）。
- fn 0 の **describe** は自分で書きます。`describeCore(w, model, unit_id, ...)` が、firmware の版、model、どの経路でも同じ
  unit id（`platformUnitId`）、channel の数を書きます（チップはその後に `describeChip`）。`setProbeDescription` で渡します。
  経路の一覧の transport と `max_op_ms` は endpoint が足します。describe は宣言だけです（core §7.3）:
  動いている間に変わるものは入れません。
  unit_id は必須です（core §7.5）。ライブラリが固有の番号を知らないチップでは、`-DOEP_UNIT_ID='"..."'`
  （`a-z 0-9 -` で 1〜16 文字、個体ごとに違う値）を与えるまでビルドが止まります。
- boot_id（core §6.5）は、最初のメッセージが届いたときに endpoint が自分で選びます。プラットフォームにハードウェアの乱数源があれば
  それ、無ければそのときのタイマーの値です。もっと良い源（不揮発の記憶に保つ起動回数など）を持つスケッチは、`setup()` で
  `setBootId` で渡します。
- 起動時、最初の `poll()` より前に、予約しない channel をすべて空きの状態にします。Hi-Z、pull 無し（`platformParkMask`。
  core §8）、または probe が保存した設定を持つならその idle です。
- `loop()` から `poll()`。長く止まりません。インターフェースも止まってはいけません。

## 3. インターフェース

```cpp
class Blink final : public oep::Interface {
 public:
  const char *name() const override { return "io.github.you.blink"; }   // 自分の逆 DNS の名前。oep.* は使わない
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }                      // 固定部分の形
  bool lockFree(uint8_t op) const override { return op == 0x03; }      // 何も変えない読み出し
  bool offers(uint8_t op) const override { return op >= 0x01 && op <= 0x03; }   // 持つ op（ops の tag）
  size_t describe(uint8_t *out, size_t capacity) override;             // 使う前に host が読むもの
  oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
};
```

- **名前**で host が探します。標準のインターフェースは `oep.*`（oep-spec が定める）で、自分のものは自分が持っている名前の
  逆 DNS（`io.github.<you>.<name>`）にします。誰の承認も要らず、知らない host は使わないだけです。これが OEP の拡張の仕方です。
- **revision** は、すべての固定の形を決めます（core §2.3、§2.7）: payload の固定部分、TLV の値、並びの要素。固定の形は
  後ろに伸ばしません。足すものは新しい TLV、任意の op、新しい値にし、固定の形を変えるときだけ revision を上げます。
- **offers** は持つ op を言います（core §1.2）: 表の必須の op すべてと、この probe が持つ任意の op。endpoint はそれから
  describe の `ops` の tag（0x09、base + bitmap）をすべての fn の describe の最初に書き、ほかの op にはセッションを見る前に
  `unknown_operation` で答えます。既定は何も持たないので、上書きしないインターフェースは何にも答えません。任意の op は
  ここで宣言し、`features` では宣言しません。
- **handle** は要求を 1 つ受け、結果の payload を `out` に書きます。
  - `completed(n)`: 実行して成功した。payload は n バイト。
  - `failed(n)` / `partial(n)`: 実行したが、うまくいかなかった（全部 / 一部）。payload は成功のときの形。
  - `rejected(reason)`: 実行していない。`kRejectMalformed`、`kRejectUnavailable`、`kRejectUnsupported`、`kRejectUnknownOperation`。
- **TLV** は `tag(u8) len(u16) value` で、長さによらず形は一つです（core §2.2。`TlvWriter`、`tlvAt`、`kTlvHeader`）。
  並びは `count、count × 要素` で、要素の長さは置きません。
- **TLV の後ろの部分**: どの要求も最後に TLV を付けられます。`plainTail(tail, payload, length, fixed, out, capacity)` が固定部分の
  後ろを読みます。知らない critical の TLV は要求を断り（`refused()` で分かる）、それ以外の知らない TLV は無視し、応答には何も
  付けません（core §2.3）。実装する tag があるときは `tail.parse(...)`、`tail.find(tag, len, &critical)`（繰り返されたら最初のもの）、
  決まった長さの TLV には `tail.fixed(tag, size, value, ...)`（ほかの長さは critical によらず malformed）、扱わない値には
  `Tail::refuse(tag, critical, ...)`（critical によらず、受け取ったままの tag で unsupported。実装する TLV は無視しません）。
- **ロック**は endpoint が見ます。どの要求も見出しに session_id を持ちます（0 = なし）。`lockFree` でない op は、ロックを持つ
  セッションにだけ実行されます（session_id 0 は `session_required`）。そのセッションのロックが終わるたびに（end、リースの
  期限切れ、ほかの host の force、どれも同じ。core §6.4、§9）`sessionOver()` が呼ばれるので、作ったものと共有の分（線の接続の
  分、コンソールのストリームの分）をすべて外します。次のセッションには何も渡しません。設定が持つもの（スロットの接続）は
  残ります。資源がほかのインターフェースの資源の上にあるもの（線の接続の上のコンソールのストリーム）は `sessionOverFirst()` で
  そう言い、先に外されます。
- **describe** は `TlvWriter` で書きます。共通の tag（`roleChannels`、`u32(kTagMaxClockHz, ...)`、op でない任意の機能には
  `u32(kTagFeatures, ...)`）と、自分の tag（0x40 から）。`ops` の tag は書きません（endpoint が書きます）。
- **oep.probe.link**（線の試験と、UART bridge の port_speed）は任意のインターフェースです: `oep::Link link(endpoint);
  endpoint.add(link);`。前の fn の番号が変わらないよう最後に足します。`endpoint.setPortSpeed(...)` で port_speed が ops に入ります。
- **oep.probe.restart**（任意、oep-if-restart）: `setup()` で最初の `poll()` の前に `endpoint.setRestart(oep::platformRestart, max_ms)`
  を呼ぶと出ます（restart 0x01、describe に `restart_max_ms`）。`max_ms` は、応答から同じ経路で confirm にまた答えるまでの最長の時間で、起動と USB の列挙し直しを
  含みます。ボードごとに見積もり、余裕を持たせます。endpoint は先に答えて flush し、セッションを終え、どのインターフェースにも
  `probeRestart()` を呼び（設定が持つもの - スロットの接続 - も、target には触れずに放す）、すべての plan を解いてから handler を
  呼びます。handler は戻りません（`esp_restart`、`rp2040.reboot()`）。自分の USB device を先に外すなら包んで渡します。

## 4. ピン: 表と plan

- `oep::PinTable pins(mask)` は、スケッチがインターフェースに渡す channel（GPIO の番号）の集まりです。インターフェースは使う
  ものを自分の持ち主の番号で `claim()` し、`release()` で返します。持たれている channel はほかには断られるので、2 つの
  インターフェースが同じピンを駆動することはありません（core §8.1）。返した channel は空きのときの状態（Hi-Z、または設定の
  とおり）になります。
- **plan** は、host が実行中にピンを割り当てる方法です（`oep.probe.plan`、oep-if-plan）。plan の役を持たない（`planRoles()` が
  false の）インターフェースの fn は unsupported で断ります。`plan_apply` が (fn, role, channel) を挙げると、endpoint は
  各インターフェースに `planCheck()`（副作用なし: 0 か断る理由）、次に `planApply()` を聞き、`planRelease()` で返させます。
  役とその候補のピンは describe で宣言します（`roleChannels`）。宣言していない役や channel は planCheck が
  `kRejectUnsupported` で断り（endpoint が tag 0x90 を付けます）、宣言した channel を他が持っているときは
  `kRejectUnavailable` で断ります。planApply はピンを何も変えません（出力の idle は駆動を続けます）。変えるのは
  そのインターフェースの最初の操作です。debug wire のピンは、接続が持っていない間は空きの状態に戻ります
  （`DmiPhy::free`）。
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
- **debug の線は、線のタイミングが許すいちばん弱い出力の強さで駆動します。** 線の鋭いエッジは、同じ治具の隣の fixture の線に
  乗ります。classic ESP32 の既定の 20 mA では、コンソールを読んでいる間、SWIO の線が 1 MHz の SPI target の bit を落としたり
  ずらしたりしました（36 frame 中 23 だけ正しい。最弱では 72 中 72 で、線の速さは変わらない。2026-10-02）。このライブラリの
  PHY はどれもそうしています（RVSWD: ESP32-P4 は `GPIO_DRIVE_CAP_0`、RP2 は 2 mA。SWIO: classic と P4 は `GPIO_DRIVE_CAP_0`、
  `OEP_SWIO_DRIVE_CAP`）。新しい PHY や別のチップへの移植でも必ず設定します（P4 の SWIO の移植は、最初は既定のままでした）。
  線がピンを取るたびに設定し直します（RVSWD の PHY は attach のたびにそうします）。host が選ぶ組は接続と接続の間は空いていて、
  fixture gpio の出力や出力の idle が別の強さを残すことがあるためです。
- **fixture gpio の出力の強さ**（oep-if-fixture §1.1）は `OepPlatform.h` の `platformDriveLevels()` から来ます。段階と、
  そのおよその mA と、既定（パッドのリセット時の強さ）です。classic ESP32 / ESP32-P4 は `GPIO_DRIVE_CAP_0..3` で約
  5 / 10 / 20 / 40 mA、既定は 2。RP2040 / RP2350 は 2 / 4 / 8 / 12 mA で、既定は 4 mA。ほかのチップは持ちません（drive_levels
  を出さず、set の drive TLV は知らない tag です）。別のチップへの移植では、そこに段階を足すか、持たないままにします。
  強さが効くのは mode 3 / 4（set と出力の idle）だけです。`PinTable::setPad` は、パッドがそれ以外の mode に移るとき既定の強さに
  戻します。ピンを自分の周辺回路に使う fixture（UART、I2C / SPI target）は claim のあとで `PinTable::ownStrength` を呼ぶので、
  出力の idle から直接取ってもパッド本来の強さから始まります。線は自分の強さ（最弱）を自分で設定します。

## 7. シリアルの口、bind、設定

- シリアルの口は OEP のフレームと、その間の生のバイトを運びます。生のバイトは **bind** のとおり（スロットのコンソール、fixture の
  UART。口ごとに 1 本）です。`endpoint.setRawPorts(&binds)` で有効になります。セッションがロックを持つ間、それが使う口の
  生の流れは止まり、終わった後に止めた位置から続きます（その間にあふれていれば残っている一番古いバイトから。transports §4、
  probe.config §1.2）。
- `ProbeConfig` はスロット、bind、plan、label、空きのときの状態、fixture UART の設定（uart の項目）、無効にした channel
  （disable の項目）を持ち、保存し（ESP32 は NVS、RP2 は flash の最後の領域）、起動時に行います。`state`（op 0x06）がスロットと
  bind の状態を、`unset`（0x05）がキーでの削除です。最後に `add(config)` し、`addPlace(wire, console)`、`addUart(uart)`、
  `setPins(&pins)`、`applySaved()`（`06.Settings/ProbeConfig`）。`load()` はスケッチがピンを空きの状態にする前に呼び、保存が無効に
  した channel を除きます: `config.load(); pins.setDisabled(config.savedDisabled()); platformParkMask(mask & ~pins.disabledMask());`
  （無効にした channel には起動時も触れない）。PinTable は設定の無効（外せる）と `forbid`（firmware のもの、ずっと）を分けて持ち、
  どの項目も forbid したピンを使えるようにはしません。
- 自分の USB デバイスを transport にする probe は、host がデバイスを構成し終えるまで at boot のスロットの attach を待たせます:
  `applySaved()` の前に `config.setAttachGate([] { return BootGuard::attachReady(tud_mounted()); })`（P4 は `usbDevice.ready()`）。
  起動時から target を読むコンソールはフレームごとに割り込みを止め、host がデバイスを列挙している間に遅れた USB の割り込みが
  USB スタックを落とすことがあります（RP2: TinyUSB 0.18 の "Can't continue xfer on inactive ep" の panic）。`BootGuard`
  （`OepBootGuard.h`）: `setup()` の最初に `begin()`、`loop()` で `poll()`。watchdog を動かし、すぐに落ちた起動を数え、
  `kSafeAfter` 回続いたら `safe()` が真になるので、その起動では `config.skipBootAttach()` を呼びます。RP2 では SDK の `_exit` と
  `isr_hardfault` を `BootGuard::crashed()` に向けます（`Firmware/OepProbe/Rp2.h`）。panic で止まらず、チップを reset します。
  firmware 更新（bootloader の rollback がある ESP32）は動くものとして扱い、起動したときに確かなものにします: `verifyRollbackLater`
  を定義しなければ ESP32 の core が `setup()` の前に image を確定するので、その後のどのリセットでも前には戻りません
  （`Firmware/OepProbe/Esp32P4.h`）。保存した設定が起こす crash の繰り返しは safe boot が受け持ちます。前の起動が何で終わったか
  （firmware が意図しなかったリセット: 落ちた・止まったときの種類と上がっていた秒数、brownout、bootloader が更新を起動しなかったか。外からのリセットと自分でした再起動は何も言いません。RP2 は `crashed()` が付ける印で自分の crash を外からの watchdog の再起動と分けます）は `BootGuard::lastBoot()` が文字列で言います。
  `Firmware/OepProbe` はそれを fn 0 の describe の firmware の文字列で版の後ろに付けます（`describeCore` の最後の引数。
  例 `0.0.29 (panic at 12 s)`）。
- fixture の UART は UART の割り込みで受けます。ビットを自分で刻む線のフレームは、その core の割り込みを止めます（フレームごと:
  SWIO は `SwioPhy::kIrqOffMaxUs` まで、RVSWD は最も遅い max_speed で約 1.1 ms まで）。ESP32 では割り込みが 128 byte の RX FIFO の
  `kUartRxFifoFull`（32）byte で起き、2000000 で 480 us の余りがあります。`loop()` がそうした線を動かすデュアルコアの ESP32 は、
  `applySaved()` の前に `uart.setInterruptCore(0)` で割り込みをもう一方の core に置きます（`Firmware/OepProbe/Esp32P4.h`）。
  その core が自分で割り込みを止めるなら置きません（classic の sampler: `Esp32.h` は `loop()` の core に置いたまま、FIFO を SWIO と
  比べて確かめます）。それでも UART が落としたバイトは lost（detail 1 あふれ）のマークになり、黙って抜けることはありません。マークは
  抜けた所の直後のバイトの位置か、それより前に付き、後ろには付きません: `FixtureUart` は driver が教える所に置きます。ESP32 は
  ESP-IDF の driver の出来事を、割り込みの core のタスクで順に自分で取り、出来事が数えていないバイトは読みません（UART に自分の
  `onReceiveError` / `onReceive` を付けないでください。arduino-esp32 の出来事のタスクが取ってしまいます）。FIFO のあふれは抜けた所
  ちょうどに付きます。RP2 は arduino-pico の受信の列のあふれが抜けた所ちょうど、PL011 の overrun と break（`poll()` で印として
  読みます）は一つ前に見たときに数えたバイトの所です。
- classic ESP32 の sampler と SWIO の線は GPIO のレジスタのバスを分け合います: sampler が続けて読む GPIO.in は、もう一方の
  core の SWIO のパルスをフレームが乱れるほど動かし、SWIO にはパリティがありません。`OepWireGate.h` が両者を分けます: フレームは
  どれも sampler が読むのを止めるのを待ち（1 サンプルまで）、sampler はフレームの後で読み直します。線はフレームで取り、`loop()` が
  一回りするまで持ちます。窓が排他の間（即時の窓、トリガの後の区画の残り、番の外のトリガの探索）は取れません: 要求は待ち、探索は
  区切りの中ですぐ番を譲ります（コンソールには区切りごとに 1 回。送るものを渡し終えたら `backgroundSent()` で早く終える）。即時の窓は
  持ち主を待ちます。`SamplerCapture::poll()` が線を手放すので、両方を持つスケッチは毎回の `loop()` でそれを呼びます（呼ばないと即時の
  窓が開きません）。sampler のループは `OepSamplerRun.h` で、host の試験も同じコードを動かします。コンソールの poll は、線を
  もらえないときは読みません（次の poll で読む）。oep-if-console §3 は読む間隔を決めず、
  その connection の要求の後は DATA0 の前に DMSTATUS を読むことと、hart が止まっている間 DATA0 / DATA1 に触れないことだけを
  求めます（docs/implementation-limits.ja.md §4.1）。

## 8. push と出来事

通知を送り出すインターフェースは `notifies()` で true を返します。endpoint はその ops に subscribe と unsubscribe（0x30 / 0x32、
core §11.3）を立て、自分で答えて `subscribe(true / false)` を呼びます。流すものは `pull()` と `pending()` も持ちます。endpoint は、
購読した host がロックを持つ間、購読の min_bytes / max_delay_ms でまとめてデータのフレームを送ります。
`endpoint.event(*this, kind, payload, length)` で出来事を送ります。出来事はまとめず、先に送る応答を送り終えたらすぐ送ります。
logic と analog の capture、capture-group がこれを使っています。何も送らないインターフェースはどちらの op も持ちません（fn 0 は
何も送りません。heartbeat は無く、host は probe の時刻を fn 0 の clock で読みます）。

## 9. USB の名乗り

USB の probe は、プロジェクトの VID:PID `1209:4F45`（`oep::reg::kUsbProjectVid` / `kUsbProjectPid`。これで firmware を
出してよい条件は [PID-USE.ja.md](../../PID-USE.ja.md)）と、個体ごとに違う serial number（unit_id。host はこれで個体を区別し、
名指した probe を探す）を名乗ります。iProduct は人が読むための自由な文字列で（firmware では `OEP probe (ESP32-P4)`）、host は
これで probe を見分けません。同じ device の OEP の外のインターフェース（ESP32-P4 のアプリの中の DFU など）も、その device の一部です。
vendor bulk のインターフェースは bInterfaceSubClass 0x4F / bInterfaceProtocol 0x45、vendor HID は usage page 0xFF4F /
usage 0x45 を持ちます（transports §3。`Firmware/OepProbe/Esp32P4.h` が EspUsbDevice の記述子をそう直します）。
describe は VID:PID について何も宣言しません（前の discoverable は無くなりました、core §7.5）。host は、プロジェクトの VID:PID、
名指された unit id、利用者が選んだ口で probe を見つけます（transports §3）。

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
- debug の線が忙しいとき（コンソールの読み取り、flash の書き込み）だけ壊れる fixture は、CPU より線のエッジを先に疑います。
  線が休んでいるとき（コンソールを外す）と忙しいときで試し、まず出力の強さを見ます。直ったかは前後を同じ手順で測り、arm した
  frame の直後に読みます（DUT の周期より長く待つと、arm していない次の frame が入って失敗に見えます）。
- 問題を直したら、同じ仕組みの箇所（ほかの PHY、ほかの SoC、線を駆動する fixture）を探して一つずつ確かめ、どれが大丈夫でどれが
  未確認かを書き残します。
