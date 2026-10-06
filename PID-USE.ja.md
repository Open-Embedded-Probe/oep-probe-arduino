# OEP の USB の ID

[English](PID-USE.md)

プロジェクトの USB の VID:PID は **`1209:4F45`**（VID 0x1209、PID 0x4F45）で、[pid.codes](https://pid.codes/1209/4F45/) が割り当てた
（oep-spec の registry の `usb`: `project_vid` / `project_pid`）。host は OEP の probe をこの VID:PID だけで自動で見分ける（oep-spec
core §3.3）。iProduct は人のための名前で、何の見分けにも使わない。この VID:PID を持てない口の probe（USB-UART ブリッジ、ID を
ハードウェアが決めている内蔵の USB シリアル）は、利用者が口を選ぶか、probe を unit_id で名指したとき（`oep://<unit_id>`）に開く。
probe が何をできるかは ID ではなく probe 自身から読む（confirm、list、describe）。

## 使ってよい範囲

このライブラリの MIT License が及ぶのはソースコードである。プロジェクトの VID:PID を使ってよいかは、それだけでは決まらない。
この文書が、下の条件のもとで認める。この VID:PID は、**OpenEmbeddedProbe ライブラリから作った firmware を動かしている USB の
device**（ライブラリが対応するどの基板でもよい）で、Open Embedded Probe のプロトコル
（[oep-spec](https://github.com/Open-Embedded-Probe/oep-spec)）を話すものを指す。probe が何をできるかは ID ではなく probe 自身から
読むので、こうした probe 全部に ID が 1 つあれば足りる。

範囲は USB の device 全体で、同じ device に属する OEP の外のインターフェースも含む（probe 自身の firmware を更新する、
アプリの中の DFU のインターフェースなど）。チップの ROM のブートローダは、チップ自身の ID のままである。

次をすべて満たすなら、その VID:PID で firmware を出してよい（自分の基板、自分の治具、example を変えたもの）:

1. **このライブラリから作っている**（fork でもよい）。出すもののソースを OSS のライセンスで公開している。
2. **仕様どおりに OEP を話す**: confirm、list、describe に oep-spec の core のとおりに答え、`oep.` の名前で出すインターフェースは、
   どれもそのインターフェースの仕様に従う。自分のインターフェースには自分の逆 DNS の名前（`io.github.<you>.<name>`）を付け、
   `oep.` を使わない。
3. **正直に名乗る**: USB の serial number を probe の `unit_id` にする（個体ごとに違い、firmware の版で変えない。oep-spec core
   §3.3 / §7.5）。fn 0 の describe に `unit_id`、経路の一覧、`discoverable = 1` を出す。
4. **プロトコルを非互換に変えない。** wire の形を変えた firmware、仕様が許さない形で OEP の要求に答える firmware は OEP の probe
   ではなく、自分の USB の ID を使う。

使わないでほしい場合:

- OEP を話さない firmware（基板が同じなだけのもの）: 自分の ID を使う。
- このライブラリから作っていない、独立した OEP の実装: 自分の ID を使う。host は、利用者が名指すか口を選んだとき、またはその ID
  への自前の対応で開く。プロトコルはこの ID に依存しない。

## 変更

この文書は、このリポジトリへのレビューを経た pull request でだけ変える。
