# LibraryTest: プローブの上でライブラリを試し、その波形とログを紐付ける

[English](README.md)

配布の firmware（`OepFirmware.h`）に足すファイルの例です。試したいライブラリの呼び出しをプローブ自身の上で動かし、
同じプローブのロジックのキャプチャでその線を取り、コードが書いたログの各行を波形のサンプルに紐付けます。

- `LibraryTest.cpp`: `io.github.example.library-test` を足す（`libraryTestRegister()`: `oep::firmware.onSetup` と `add`）。
  [`oep::SimpleInterface`](../../docs/guide/simple-interface.ja.md) で作ってあり、op の名前と欄が describe に出る（`oep call` や
  `oep_client.call` で名前で呼べる）。
  - `run`（0x10、`case: u8`）: ケースをワーカーで起こしてすぐ答えるジョブ。走っている間に呼ぶと busy、無いケースは failed で終わる。
  - `status`（0x20）と `cancel`（0x21）: ワーカーの op（`addWorker`）。状態、失敗の code と message、開始と終了の時刻。
  - `result`（0x22、ロック不要）: 最後のケースの番号と戻り値。
  - `log-read` / `log-marks` / `log-clear`（0x11〜0x13）: ログ（`addLog`）。ケースの中の行は `mark()` / `markf()` で入る。
- `test_library_test.py`: ホストの例（pytest）。SDA / SCL にキャプチャを plan し、START をトリガに start、`call_wait` で `run` を
  終わりまで待ち、区画を読み、ログの各行をサンプルに置いて確かめる。短い書き込みと、表示器を模した長い一連の処理（初期化と描画、約 90 ms）。

## ケースはワーカーで動く

ケースは `oep::Worker` の上で動きます（ESP32 / ESP32-P4 は loop() と別のコアに固定した FreeRTOS のタスクで、ESP32-P4 ではコア 0。
RP2040 / RP2350 は 2 つ目のコアの `loop1` から `worker.poll()`）。表示器の初期化から描画まで、電源 IC の手順、ライブラリの中の `delay`
など、長く掛かっても、その間プローブはキャプチャ、lease、status に答え続けます。ケースのコードはふつうの順番どおりのコードで、
マーカーは `mark()` / `markf()` で書きます（ログは書き込みと読み出しの間だけ短く排他するので、自分でロックを取る必要はない）。
バスもワーカーの上で、最初のケースのときに始めます。割り込みがワーカーのコアに付きます。CH32 にはワーカーのタスクも 2 つ目のコアも
無いので、この例は入れていません。

## ログは送らない

`mark("## step")` は、RAM のリングに行を書き、その先頭にプローブの時計（ns）のマークを付けるだけです（数百 ns）。UART に流す
マーカーと違い、送り終わるまで処理が止まらず、ピンも要りません。

- キャプチャの区画の `start_ns` も同じ時計なので、行のサンプルは `(行の ns − start_ns) × レート` です（`oep_client.probe_log.sample_of`）。
  確かさは区画の `start_uncertainty_ns`（ESP32-P4 で ±5 µs）。
- ホストはログを common §1 の read / marks で読みます（`oep_client.probe_log.ProbeLog`）（読んでも消えない）。リングがあふれれば lost のマーク、マークのリングが
  あふれればマークの通し番号の飛びで、欠けたことが分かります（`probe_log.Log.lost`）。
- `probe_log.rows` は、行を記録器（WireSkein の `run.json`）の行にします: ホストの `time.monotonic()`、`probe_ns`、`boot_id`、
  プローブの名前。`#` で始まる行はマーカー（WireSkein の見出しの方言）、ほかはイベント。

## 使い方

1. 自分のスケッチのフォルダに `LibraryTest.cpp` を置き、`.ino` を次の形にしてビルドする（`sketch.yaml` は
   [firmware の機能](../../docs/guide/firmware-features.ja.md) §5）。

   ```cpp
   #include <OepFirmware.h>
   void libraryTestRegister();

   void setup() {
     libraryTestRegister();
     oep::firmware.begin();
   }

   void loop() { oep::firmware.loop(); }
   ```
2. ケースの関数を自分のライブラリの呼び出しに置き換え、マーカーを `mark()` / `markf()` で書く。
3. ホストで `OEP_LIBTEST_PORT=<port> OEP_LIBTEST_PINS=<sda>,<scl> pytest test_library_test.py`。

ピンは石ごとに選んである（ESP32-P4 32 / 33、classic ESP32 25 / 26、RP2040 / RP2350 26 / 27）。スケッチが `kSketch` で押さえるので
ほかのインターフェースは plan されず、キャプチャだけが聞ける。何もつないでいないバスでも、誰もいないアドレスへの書き込みで
START、アドレス、NACK、STOP が出る（プルアップの無いバスでは ESP32-P4 の `endTransmission` は 4 を返す）。

