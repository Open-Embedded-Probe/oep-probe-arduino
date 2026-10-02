# OEP の USB の VID:PID を使う

[English](PID-USE.md)

このライブラリの MIT License が及ぶのはソースコードである。下の USB の VID:PID を使ってよいかは、それだけでは決まらない。
この文書が、ここに書く条件のもとで認める。

## ID

OEP の USB の VID:PID（pid.codes の VID）は、**OpenEmbeddedProbe ライブラリから作った firmware を動かしている
USB の device** を指す。ライブラリが対応するどの基板でもよい（ESP32-P4、RP2350、RP2040、後から足すもの）。その device は
Open Embedded Probe のプロトコル（[oep-spec](https://github.com/Open-Embedded-Probe/oep-spec)）を話す。

host はこの ID で、すべてのシリアルの口を開かずに OEP の probe を見つける（discovery）。probe が何を持つかは、ID ではなく、
probe 自身（confirm、list、describe）から読む。そのためには、こうした probe 全部に ID が 1 つあれば足りる。

## 使ってよい場合

次をすべて満たすなら、この VID:PID で firmware を出してよい（自分の基板、自分の治具、example を変えたもの）:

1. **このライブラリから作っている**（fork でもよい）。出すもののソースを、pid.codes が求めるとおり OSS のライセンスで公開している。
2. **仕様どおりに OEP を話す**: confirm、list、describe に oep-spec の core のとおりに答え、`oep.` の名前で出すインターフェースは、
   どれもそのインターフェースの仕様に従う。自分のインターフェースには自分の逆 DNS の名前（`io.github.<you>.<name>`）を付け、
   `oep.` を使わない。
3. **正直に名乗る**: iProduct を `OEP` で始め、USB の serial number を probe の `unit_id` にする（個体ごとに違い、firmware の版で
   変えない。oep-spec core §3.3 / §7.5。参照の firmware はチップの固有の番号から作る）。oep.core の describe に `unit_id`、経路の
   一覧、`discoverable = 1` を出す。
4. **プロトコルを非互換に変えない。** wire の形を変えた firmware、仕様が許さない形で OEP の要求に答える firmware は OEP の probe
   ではなく、自分の VID:PID を使う。

## 使わないでほしい場合

- OEP を話さない firmware（基板が同じなだけのもの）: 自分の ID を使う。
- このライブラリから作っていない、独立した OEP の実装: 自分の VID:PID を取る。host は iProduct と describe で同じように見つける。
  プロトコルはこの ID に依存しない。
- 手元の試しのビルドは、pid.codes の試験用の PID `1209:0001` を使ってよい。出荷はしない。

## 変更

この文書は、このリポジトリへのレビューを経た pull request でだけ変える。ID の登録は pid.codes にあり、
持ち主は `Open-Embedded-Probe`。
