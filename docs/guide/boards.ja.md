# ボード

[English](boards.md)

このライブラリで各チップが何をできるか、リリースされた firmware がどのピンを出すか、リリースされた firmware の無いボード向けの
ビルドの仕方。

## チップごとにできること

| | RP2040 / RP2350 | ESP32-P4 | classic ESP32 | ほかの ESP32（S3、C3、C6 など） |
|---|---|---|---|---|
| リリースされた firmware | `OepProbe-rp2040` / `-rp2350` | `OepProbe-esp32p4` | `OepProbe-esp32` | 無し: 自分でビルド（下） |
| ベンチでの確認 | まだ | 済み（CH32X035 の治具） | 済み（CH32V003 の治具） | 無し |
| 経路 | USB CDC | HS vendor bulk、HID、USB CDC、USB-Serial/JTAG | UART bridge（115200） | USB-Serial/JTAG か USB CDC |
| RVSWD（CH32 の 2 線） | あり（SIO） | あり（dedicated GPIO） | 無し | あり（dedicated GPIO。ビルドのみ、未確認） |
| SWIO（CH32V00x の 1 線） | 無し | 無し | あり | 無し |
| SWD（ARM） | あり | 無し | 無し | 無し |
| target のコンソール（debug module 経由） | あり | あり | あり | あり |
| GPIO、UART の fixture | あり | あり（UART 2 つ） | あり | あり |
| ロジックのキャプチャ | 無し | PARLIO: 16 ch まで、2 ch 160 Msps / 8 ch 40 Msps / 16 ch 20 Msps。ワンショットのレベル / エッジのトリガ、プリトリガは 32 Ki サンプルまで（16 ch） | GPIO の sampler: 8 ch、0.4〜2 MHz、ワンショット。レベル / エッジのトリガを区切りごとに探す | 無し |
| アナログのキャプチャ（ワンショット、生の値、しきい値のトリガとプリトリガ） | GP26〜28、4 ch、合計 500 kS/s | ADC1 の GPIO16〜23、4 ch、合計 46 kHz、減衰 0〜12 dB | ADC1 の GPIO32〜36 / 39、4 ch、合計 20〜100 kHz | 無し（ビルドしていない） |
| トラックを一緒に始める（capture-group） | 無し | ロジック + アナログ | ロジック（sampler）+ アナログ | 無し |
| SPI / I2C のデバイス（target の相手） | 無し | あり | あり | あり（未確認） |
| 設定の保存 | flash（最後の領域） | NVS | NVS | NVS |

「無し」は、そのチップ向けの実装がライブラリにまだ無いという意味で、チップにできないという意味ではありません。SWIO は classic
ESP32 のサイクル数でビットの時間を数え、SWD の bit-bang は RP2 のもの、キャプチャは P4 の PARLIO と classic ESP32 の sampler です。

アナログのキャプチャは、取っている間その pad をアナログの機能に切り替えるので、デジタルの入力が切れます。同じ pad の
ロジックのキャプチャ（や GPIO の fixture）は、その間 0 を読みます。1 つの信号をロジックとアナログで見るときは、2 つの pad に
つなぎます。

どれも 3.3 V 系です。ライブラリはレベルを変換しません（README の電気的な注意を参照）。

## リリースされた firmware のピン

一覧のピンは、host の plan のとおり何にでもなります（RVSWD / SWD の組、reset の線、GPIO、UART、キャプチャのチャネル）。
あるインターフェースが持っているピンは、ほかには断られます。一覧に無いピンはボードのものです。

| firmware | 出すピン | 触らないピン |
|---|---|---|
| RP2040 / RP2350 | GP0〜GP22、GP26〜GP28 | GP23〜GP25、GP29（Pico の SMPS、VBUS の検出、LED、VSYS）。ほかのボードで、出すピンにつながった部品（LED、PSRAM の chip select）は、host が触らないようにする |
| ESP32-P4 | GPIO0〜GPIO54（24、25 以外） | GPIO24 / 25（USB-Serial/JTAG） |
| classic ESP32 | GPIO4、5、13、14、16〜19、21〜23、25〜27、32、33、34〜36、39（34〜39 は入力だけ）。SWIO は 32 未満の出力のピン | 1、3（UART0: 経路）、6〜11（flash）、0、2、12、15（起動のストラップ） |

RP2 の UART の fixture は UART0 で、RX / TX は GP1/0、GP13/12、GP17/16、GP29/28 のどれかです。ESP32 はどのピンでも使えます
（GPIO の行列）。

## USB の名乗り

| firmware | VID:PID | iProduct | serial number |
|---|---|---|---|
| RP2040 / RP2350 | ボード（arduino-pico）の既定 | `OEP probe (RP2040)` / `(RP2350)` | ボードの既定（flash の unique id） |
| ESP32-P4（HS の口） | `303a:0002` | `OEP probe (ESP32-P4)` | MAC + `-hs` |
| classic ESP32 | bridge のもの | -（UART なので、host が開いて聞く） | bridge のもの |

pid.codes が OEP の PID `1209:4F45` を割り当てたら、firmware はそれに移ります（[PID-USE.ja.md](../../PID-USE.ja.md)）。

## ほかのボード向けのビルド

ベンチの無いボードには、リリースされた firmware を出しません。確かめていない binary は誰の役にも立たないからです。自分で
ビルドするのは数分で済みます。例: ESP32-S3 DevKitC を CH32 のデバッガにする。

1. **やりたいことに近い example から始める。** CH32 のデバッガなら `04.Debug/RvswdDebugProbe`、治具なら `01.Basics/FixtureProbe`、
   全部なら `Firmware/OepProbe`（自分のチップに近いチップの部分を写す。例: `Esp32P4.h`）。
2. その `sketch.yaml` に **profile を足す**: ボードの FQBN、core の版、ライブラリ。

   ```yaml
   profiles:
     esp32s3:
       fqbn: esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc
       platforms:
         - platform: esp32:esp32 (3.3.12)
           platform_index_url: https://espressif.github.io/arduino-esp32/package_esp32_index.json
       libraries:
         - OpenEmbeddedProbe (0.0.9)
   ```

   `RvswdDebugProbe` にはこの profile がもうあり、`arduino-cli compile --profile esp32s3 examples/04.Debug/RvswdDebugProbe` で
   ビルドできます。
3. `Serial` が何かで**経路を選ぶ**。USB-Serial/JTAG（`USBMode=hwcdc`: `kUsbSerialJtag`）、TinyUSB の CDC（`USBMode=default`:
   `kUsbCdc`。host が見つけられるように、製品名を `OEP` で始める: `USB.productName("OEP probe (ESP32-S3)")`）、USB-UART bridge
   （`kUartBridge`。classic ESP32、UART0 の ESP32-C3）。
4. **ピンを選ぶ。** flash / PSRAM のピン、USB のピン、起動のストラップ、ボードが部品につないだピンを外し、`describeCore` で
   予約とし、残りを出す（ピンの表の mask、host が選ぶなら `pin_choice`）。
5. **ビルドして焼く**: `arduino-cli compile --profile esp32s3 <dir>`、`arduino-cli upload -p <port> --profile esp32s3 <dir>`。
6. `oep dump --port <port>` で**確かめ**、Python から target を attach する（[使い始める](getting-started.ja.md)）。

動いたら知らせてください（リポジトリの issue）。ベンチのあるボードには、リリースされた firmware を出せます。

## ライブラリが知らないチップ

Arduino の core の違いは少なく、1 か所（`src/OepPlatform.h`: ピンのモード、unit id、乱数、UART の設定）にまとめてあります。
debug の線には、そのチップ向けの PHY の実装が要ります（`OepRvswdPhy.cpp` に ESP32 の dedicated GPIO と RP2 の SIO があり、
ほかは attach しない空の実装）。endpoint、ピンの表、fixture、設定は移植できます（`tests/host` は PC でビルドします）。
