# 使い始める

[English](getting-started.md)

何も入っていないボードから、PC が動かす probe になるまでの 4 段階: firmware を焼く、host のライブラリを入れる、probe を
見つける、使う。Arduino IDE は要りません。

## 1. firmware を焼く

チップに合う firmware を [Releases](https://github.com/Open-Embedded-Probe/oep-probe-arduino/releases) から取ります
（`firmware-<version>.json` にファイルと sha256 の一覧があります）。

| チップ | ファイル | 書き方 |
|---|---|---|
| RP2040（Pico、RP2040-Zero など） | `OepProbe-rp2040-<version>.uf2` | BOOTSEL を押しながら挿し、出てきたドライブにコピー |
| RP2350（Pico 2 など） | `OepProbe-rp2350-<version>.uf2` | 同じ |
| SparkFun Pro Micro RP2350 | `OepProbe-promicrorp2350-<version>.uf2` | 同じ（GP19（PSRAM の選択）以外のすべての GPIO） |
| ESP32-P4 | `OepProbe-esp32p4-<version>.merged.bin` | `esptool.py --chip esp32p4 write_flash 0x0 <file>` |
| classic ESP32（DevKitC など） | `OepProbe-esp32-<version>.merged.bin` | `esptool.py --chip esp32 write_flash 0x0 <file>` |

firmware には配線を焼き込んでいません。どのピンを使うかは、host が probe を使うときに決めます。同じファイルで、どの治具にも
使えます。ほかのチップ（ESP32-S3、C3、C6 など）のリリースされた firmware はまだありません。自分でビルドします（[ボード](boards.ja.md)）。

使う前に焼いてください。ボードに何が入っているかは分かりません。

OepProbe が動いている ESP32-P4 は、HS の口だけで更新できます: `dfu-util -D OepProbe-esp32p4-<version>.bin`（merged ではなく
app の image）が、もう一方の app の領域に書き、確かめてから再起動します。設定は残ります（まっさらにしたいときは
`oep config erase <port>`）。新しい firmware が USB の列挙まで進まなければ、次のリセットで前の firmware に戻ります。一度も焼いていない
チップや、起動しないチップは、上のとおり USB-Serial/JTAG で esptool を使って焼きます。

## 2. host のライブラリを入れる

```sh
pip install oep-client-python
```

`oep` の命令と Python の `oep_client` が入ります。ほかの host（ch32rv、ArduinoCore-CH32RV の書き込み）も同じプロトコルを
話します。この手引きでは Python のものを使います。

## 3. probe を見つけて、何を持つかを見る

```sh
oep dump --port /dev/ttyACM0        # probe のシリアルの口（Windows なら COM5）
oep dump --port usb                 # または USB の最初の OEP の probe（vendor bulk / HID のインターフェース）
```

`dump` は、probe が持つインターフェースを名前、ピン、上限、機能つきで並べます。ロックが要らないので、ほかのプログラムが
probe を使っていても邪魔しません。ESP32-P4 は 4 つの経路（HS vendor bulk、USB-Serial/JTAG、HID、CDC の口）を、ほかは 1 つを
出します。

USB の probe は iProduct `OEP probe (...)` を名乗ります。host はこれで、すべての口を開かずに probe を見つけます。

## 4. 使う

host はセッションのロックを取り、要るピンを plan で決め、インターフェースを使い、セッションを終えます。ロックは 2 つの
プログラムが同時に probe を動かすのを防ぎます。リース（下では 5 秒）が切れれば、プログラムが落ちてもロックは外れます。

### GPIO と UART（試験の治具）

```python
from oep_client import core, link
from oep_client.fixture import FixtureUartIO, Gpio

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 5000, owner="my-test")               # ロック。落ちたプログラムが残したロックなら奪う

gpio = Gpio(hst)
uart = FixtureUartIO(hst)
core.plan_apply(hst, [(gpio.fn, 1, 15),             # (fn, role, channel): gpio の線をピン 15 に
                      (uart.uart.fn, 1, 1), (uart.uart.fn, 2, 0)])   # uart の RX をピン 1、TX をピン 0 に
gpio.set([(15, Gpio.OUTPUT_HIGH)])
print(gpio.read([15]))
uart.configure(115200)
uart.write(b"hello\r\n")
print(uart.read(64))
hst.end()
```

あるインターフェースが持っているピンは、ほかのインターフェースには断られます（`rejected: unavailable`）。試験が 2 つの
出力をぶつけることはありません。

### RVSWD で CH32 をデバッグする

```python
from oep_client import core, link, target

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 5000, owner="my-debugger")
wire = target.Wire(hst)                              # oep.wire.rvswd
found = wire.scan()                                  # 空いているピンの組をすべて試す: どこで debug module が答えるか
conn, dmstatus = wire.attach(halt=True, pins=found[0].pins)
dm = target.RiscvDm(hst, conn)
print(dm.read_block(0x08000000, 4).hex())            # flash の最初の数語
dm.resume()
wire.detach(conn)
hst.end()
```

target が線に求めるもの（休ませ方、速さ）は host が言います。CH32L103 には attach で `idle_clock="low"` と
`max_speed=1_000_000` を渡します。NRST のピンで target を reset するときは、その channel を名指します
（`wire.attach_under_reset(channel)`）。既定の reset 線はありません。

書き込み、コンソールなどは host の道具の仕事です。ch32rv と ArduinoCore-CH32RV の書き込みが OEP で行います。

### 信号をキャプチャする（ESP32-P4）

```python
from oep_client import capture, core, link

hst = link.open_host("usb")
core.take(hst, 5000, owner="my-capture")
lc = capture.LogicCapture(hst)
core.plan_apply(hst, [(lc.fn, 0, 20), (lc.fn, 1, 21)])   # チャネル 0 を GPIO20、チャネル 1 を GPIO21 に
cfg = lc.configure(rate=20_000_000, samples=200_000)
lc.start()
(segment,) = lc.wait()
data = lc.read_segment(segment)
lc.to_sr("capture.sr", data, segment.samples)          # PulseView で開ける
hst.end()
```

16 チャネルまで、どのピンでも取れます。ほかのインターフェースが持っているピンでも取れるので（capture は聞くだけ）、試験中の
SPI デバイスを見ながら取れます。2 チャネルで 160 Msps、8 で 40 Msps、16 で 20 Msps です。

出来事を捕まえるには、ワンショットにトリガ（1 つのチャネルのレベルかエッジ）と、その前にどれだけ残すかを渡します:

```python
lc.configure(rate=20_000_000, samples=200_000,
             trigger=(capture.EDGE, 1, 1),   # チャネル 1 の立ち下がり（value 0 立ち上がり、1 立ち下がり、2 両方）
             pretrigger=1_000)               # その前の 1000 サンプルを残す
lc.start()                                   # トリガを待つ
(segment,) = lc.wait(timeout=30)             # lc.force() でトリガなしに始められる
data = lc.read_segment(segment)              # トリガは segment.trigger_index 番目のサンプル
```

classic ESP32 の sampler もトリガを取れます（8 チャネル、2 MHz まで）。割り込みを止めてサンプルするので、250 ms までの
区切りで探します。区切りの間のすき間（約 1 ms）に来たエッジは見逃します。

ロジックとアナログを一緒に、ロジックのトリガで取るには capture-group を使います。アナログはロジックのトリガに従い、どちらの
区画も同じ瞬間に印を付けます。

```python
an = capture.AnalogCapture(hst)
grp = capture.CaptureGroup(hst)
core.plan_apply(hst, [(lc.fn, 0, 20), (lc.fn, 1, 21), (an.fn, 0, 16)])
lc.configure(rate=20_000_000, samples=200_000, trigger=(capture.EDGE, 1, 1), pretrigger=1_000)
an.configure(rate=40_000, samples=4_000, pretrigger=40)    # 自分のトリガは無し: ロジックのトリガに従う
grp.bind([lc, an], trigger=lc)
grp.start()
st = grp.wait(timeout=30)                                   # st.trigger_ns: トリガが立った時刻
(logic,), (analog,) = lc.segments(), an.segments()          # logic.trigger_index、analog.trigger_index: その瞬間
```

従うトラックは組と一緒に始まり、トリガまで取り続けます。その pretrigger は、そのトラック自身のサンプル数です。classic ESP32 の
sampler はトリガのトラックにはなれますが、ほかのトリガには従えません（区切りの間にすき間があるため）。

## 5. 治具の設定を probe に持たせる

治具 = firmware + その設定です。一度書いて保存すれば、probe は起動のたびにそれを行います。

```sh
oep config slot /dev/ttyACM0 --name dut --wire rvswd --pins 2,3 --attach at-boot --retry 1 --mechanism dmseq
oep config bind /dev/ttyACM0 --port 0 --mode last-reset --stream slot:dut      # target のコンソールをこの口に
oep config idle /dev/ttyACM0 5 pull-up --save
oep config show /dev/ttyACM0
```

これで、probe のシリアルの口のターミナルに、起動時から target のコンソールが出ます。書き込みの道具は、同じ口で OEP を
話し続けます。各項目の説明は `06.Settings/ProbeConfig` にあります。

## 次に

- [probe を書く](writing-a-probe.ja.md): ライブラリの中身と、自分のインターフェース。
- [ボード](boards.ja.md): チップごとにできること、リリースされた firmware の無いボード向けのビルド。
- プロトコル: [oep-spec](https://github.com/Open-Embedded-Probe/oep-spec)。
