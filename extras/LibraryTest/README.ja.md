# LibraryTest: プローブの上でライブラリを試し、その波形とログを紐付ける

[English](README.md)

配布の firmware（`examples/Firmware/OepProbe`）に足すファイルの例です。試したいライブラリの呼び出しをプローブ自身の上で動かし、
同じプローブのロジックのキャプチャでその線を取り、コードが書いたログの各行を波形のサンプルに紐付けます。

- `LibraryTest.cpp`: `io.github.example.library-test` を足す（`oepUserSetup`）。op `run`（0x10）でケースを起こしてすぐ答え、
  `status`（0x20）で走っているか、終わったか、結果を返す。ケースの中の行は `mark()` で `OepProbeLog.h` のログに入る。
- `test_library_test.py`: ホストの例（pytest）。SDA / SCL にキャプチャを plan し、START をトリガに start、`run`、`status` で終わりを
  待ち、区画を読み、ログの各行をサンプルに置いて確かめる。短い書き込みと、表示器を模した長い一連の処理（初期化と描画、約 90 ms）。

## ケースは別のスレッドで動く

ケースは OEP とは別のスレッドで動きます（ESP32 / ESP32-P4 は FreeRTOS のタスク、RP2040 / RP2350 は 2 つ目のコアの `loop1`）。
表示器の初期化から描画まで、電源 IC の手順、ライブラリの中の `delay` など、長く掛かっても、その間プローブはキャプチャ、lease、
status に答え続けます。ケースのコードはふつうの順番どおりのコードで、マーカーは `mark()` / `markf()` で書きます（ログは
エンドポイントのスレッドと共有するので、写す間だけ排他する）。CH32 には FreeRTOS も 2 つ目のコアも無いので、この例は入れていません。

## ログは送らない

`mark("## step")` は、RAM のリングに行を書き、その先頭にプローブの時計（ns）のマークを付けるだけです（数百 ns）。UART に流す
マーカーと違い、送り終わるまで処理が止まらず、ピンも要りません。

- キャプチャの区画の `start_ns` も同じ時計なので、行のサンプルは `(行の ns − start_ns) × レート` です（`oep_client.probe_log.sample_of`）。
  確かさは区画の `start_uncertainty_ns`（ESP32-P4 で ±5 µs）。
- ホストはログを common §1 の read / marks で読みます（読んでも消えない）。リングがあふれれば lost のマーク、マークのリングが
  あふれればマークの通し番号の飛びで、欠けたことが分かります（`probe_log.Log.lost`）。
- `probe_log.rows` は、行を記録器（WireSkein の `run.json`）の行にします: ホストの `time.monotonic()`、`probe_ns`、`boot_id`、
  プローブの名前。`#` で始まる行はマーカー（WireSkein の見出しの方言）、ほかはイベント。

## 使い方

1. `OepProbe` のフォルダを写し、`LibraryTest.cpp` を足し、写した `sketch.yaml` の `libraries: - dir:` をこのライブラリの場所に
   直してビルドする（[firmware の機能](../../docs/guide/firmware-features.ja.md) §4）。
2. ケースの関数を自分のライブラリの呼び出しに置き換え、マーカーを `mark()` / `markf()` で書く。
3. ホストで `OEP_LIBTEST_PORT=<port> OEP_LIBTEST_PINS=<sda>,<scl> pytest test_library_test.py`。

ピンは石ごとに選んである（ESP32-P4 32 / 33、classic ESP32 25 / 26、RP2040 / RP2350 26 / 27）。スケッチが `kSketch` で押さえるので
ほかのインターフェースは plan されず、キャプチャだけが聞ける。何もつないでいないバスでも、誰もいないアドレスへの書き込みで
START、アドレス、NACK、STOP が出る（プルアップの無いバスでは ESP32-P4 の `endTransmission` は 4 を返す）。

