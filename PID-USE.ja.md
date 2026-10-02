# OEP の USB の ID

[English](PID-USE.md)

参照の firmware は、今は **仮の USB の ID** で動いている。仮の ID はボードの既定の VID:PID（ESP32-P4 では arduino-esp32 の
TinyUSB の既定の `303a:0002`）で、iProduct は `OEP` で始まる。この仮の ID は**配布には使えない**。これに頼る製品や
firmware を出さないこと。

専用の PID を取得できたら、参照の firmware はそれに切り替える予定である。それまでは**暫定の決まり**として、host は iProduct が
`OEP` で始まること（ほかの手がかりは oep-spec の host 開発ガイド §1.7）で OEP の probe の候補を探す。手がかりは候補にすぎず、
OEP の probe かどうかは、host が開いて confirm に応答があるまで分からない。そのため、たまたま手がかりに当たった関係の無い USB の
device を開くことがある（host は confirm だけを送り、正しい応答が無ければ閉じる。oep-spec core §3.3）。構成によっては、対象を
明示する必要がある（probe を unit_id で名指す `oep://<unit_id>`、または利用者が口を選ぶ）。何ができるかは probe 自身から読む
（confirm、list、describe）。それまでに配布のために USB の ID が要る firmware は、自分の ID を使うこと。

## 専用の PID を取得した後に使ってよい範囲

このライブラリの MIT License が及ぶのはソースコードである。専用の PID を使ってよいかは、それだけでは決まらない。この文書が、
下の条件のもとで認める。その PID は、**OpenEmbeddedProbe ライブラリから作った firmware を動かしている USB の device**
（ライブラリが対応するどの基板でもよい）で、Open Embedded Probe のプロトコル
（[oep-spec](https://github.com/Open-Embedded-Probe/oep-spec)）を話すものを指す。probe が何をできるかは ID ではなく probe 自身から
読むので、こうした probe 全部に ID が 1 つあれば足りる。

次をすべて満たすなら、その PID で firmware を出してよい（自分の基板、自分の治具、example を変えたもの）:

1. **このライブラリから作っている**（fork でもよい）。出すもののソースを OSS のライセンスで公開している。
2. **仕様どおりに OEP を話す**: confirm、list、describe に oep-spec の core のとおりに答え、`oep.` の名前で出すインターフェースは、
   どれもそのインターフェースの仕様に従う。自分のインターフェースには自分の逆 DNS の名前（`io.github.<you>.<name>`）を付け、
   `oep.` を使わない。
3. **正直に名乗る**: iProduct を `OEP` で始め、USB の serial number を probe の `unit_id` にする（個体ごとに違い、firmware の版で
   変えない。oep-spec core §3.3 / §7.5）。oep.core の describe に `unit_id`、経路の一覧、`discoverable = 1` を出す。
4. **プロトコルを非互換に変えない。** wire の形を変えた firmware、仕様が許さない形で OEP の要求に答える firmware は OEP の probe
   ではなく、自分の USB の ID を使う。

使わないでほしい場合:

- OEP を話さない firmware（基板が同じなだけのもの）: 自分の ID を使う。
- このライブラリから作っていない、独立した OEP の実装: 自分の ID を使う。host は、利用者が名指すか口を選んだとき、またはその ID
  への自前の対応で開く。プロトコルはこの ID に依存しない。

## 変更

この文書は、このリポジトリへのレビューを経た pull request でだけ変える。
