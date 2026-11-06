# キャプチャの使い分け

[English](capture.md)

OEP のプローブは、ロジックアナライザとして信号を取り込めます。取り込みは、プローブを**キャプチャだけに使う**場合と、
**デバッグや治具のラベルと組み合わせる**場合とで、用意するもの、設定、手順が違います。この文書はその違いをまとめます。
どのボードで何チャネル、どの速さで取れるかは [ボード](boards.ja.md#チップごとにできること) にあります。

## どれを選ぶか

| 使い方 | 向いている場面 | プローブ本体の設定 | チャネルの指し方 |
|---|---|---|---|
| [A. キャプチャだけ](#a-キャプチャだけで使う) | 手元の回路の信号を見る、プロトコルを解読する | 要らない | プローブのピンの名前（`GPIO47`） |
| [B. ラベル付きの配線で取る](#b-ラベル付きの配線で取る) | デバッグや治具でつないだ線を、そのまま観測する | 配線のラベル（据え置きの設定） | 口の線の名前（`mcu.swdio`、`mcu@pb6`） |
| [C. 操作と一緒に取る](#c-操作と一緒に取る) | 書き込み、リセット、通信などの操作の最中に何が起きたかを記録する | B と同じ | B と同じ |
| [D. テストで照合する](#d-テストで照合する) | 波形が期待どおりかを自動で判定する | B と同じ | B と同じ |

## 共通のこと

- **聞くだけ**: キャプチャはピンを駆動しません。ほかの機能（デバッグの線、UART、SPI など）が使っているピンも、そのまま取れます。
- **アナログは別**: アナログの取り込みは、その間そのピンをアナログの機能に切り替えるので、ほかの機能と共有できません。
  同じ信号をロジックとアナログの両方で見るときは、2 本のピンにつなぎます。
- **取り方の種類**:
  - ワンショット（`--mode oneshot`）: プローブのメモリに決めた数だけ取る。トリガ（エッジかレベル）と、その前に残す数（プリトリガ）を指定できます。
  - 流し続ける（`--mode stream`）: 取りながら PC へ送る。プローブのメモリより長く取れます。速さは経路（USB の速さ）で決まります。
  - 繰り返し（`--mode repeat`）: トリガのたびにワンショットを繰り返し、1 つのファイルにまとめます。
- **経路**: 速い取り込みの読み出しには USB（ESP32-P4 は HS の vendor bulk。`oep://<プローブ>?via=usb`）が向きます。UART でつなぐ
  プローブ（classic ESP32）は、読み出しの間だけリンクの速さを上げます（WireSkein が自動で選びます）。
- **ファイル**: 取り込んだものは WireSkein の形式（`.wireskein`）で残ります。実際に使った速さ、トリガの位置、プローブの時刻、
  チャネルの名前が入ります。`wireskein convert` で PulseView の `.sr` や VCD にできます。

## A. キャプチャだけで使う

プローブをロジックアナライザとして使います。プローブ本体に設定は要りません。

**用意するもの**: キャプチャに対応するプローブ、WireSkein（`pip install "wireskein[oep]"`）。

**配線**: 見たい信号をプローブの空いているピンへ、GND を共通に。ボードの部品につながったピン（LED、ボタン、起動のストラップ）は避けます。

**手順**:

1. ピンの名前を確かめる: `oep dump --port oep://<プローブ>` で、取れるピン（`GPIO47` など）が分かります。
2. 取る:

   ```sh
   wireskein capture --source oep:oep://<プローブ> \
       --channels SDA=GPIO47,SCL=GPIO48 --rate 20M --samples 200k \
       --trigger SCL:fall --pretrigger 1000 -o i2c.wireskein
   ```

   `NAME=ID` の `NAME` は、ファイルの中でのチャネルの名前です。取るたびに自分で付けます。
3. 見る・解読する:

   ```sh
   wireskein gui i2c.wireskein                 # ブラウザで表示
   wireskein analyze i2c.wireskein             # バスを見つけて解読（UART、SPI、I2C、CAN、LIN など）
   wireskein decode i2c.wireskein --spec '{"buses":[{"protocol":"i2c","pins":{"sda":"SDA","scl":"SCL"}}]}'
   ```

**決めること（取るたびに指定）**: チャネル、速さ、サンプル数（流すなら時間）、トリガ、プリトリガ。どれもプローブ本体には残りません。

## B. ラベル付きの配線で取る

デバッグや治具のために配線へ名前（ラベル）を付けたプローブでは、チャネルを線の名前で指せます。配線を変えても、ラベルを
直せば同じコマンドで取れます。

**用意するもの**: A に加えて、配線のラベル。

**設定（一度だけ、プローブに保存）**:

- デバッグの線は `oep pins --wire <rvswd|swio> --name <口> --save` で見つけてラベルにできます。
- ほかの線は `oep settings set <プローブ> core channel.GPIO12.label=mcu.rx@pa9 channel.GPIO6.label=mcu@pb6 --save` のように書きます。
  名前の規則は [設定項目](settings.ja.md#ピンのラベル口と線の名前) にあります。

**手順**:

```sh
wireskein capture --source oep:oep://<プローブ> \
    --channels SWDIO=mcu.swdio,SWCLK=mcu.swclk,RX=mcu.rx --rate 40M --samples 1M -o debug.wireskein
wireskein analyze debug.wireskein           # CH32 のデバッグの線（RVSWD、SWIO）も解読できます
```

**A との違い**:

| | A. キャプチャだけ | B. ラベル付き |
|---|---|---|
| プローブの設定 | 無し | ラベル（保存して据え置き） |
| チャネルの指し方 | `GPIO47` | `mcu.swdio`、`mcu@pb6`（大小文字を区別しない） |
| 配線を変えたとき | コマンドのピンを直す | ラベルを直す。コマンドはそのまま |
| ほかの PC、ほかの道具 | ピンの対応を人が伝える | 同じ名前で同じ線を指せる |

## C. 操作と一緒に取る

書き込み、リセット、デバッグの接続、I2C の転送などを行いながら取ると、その操作で線に何が起きたかを記録できます。
同じプローブの別のピンで操作しながら、その線を取ります（B のラベルがあると、操作とキャプチャで同じ名前を使えます）。

**手順（Python）**: 取り込みの最中に操作を走らせる形で書きます。

```python
from oep_client import acquire, core, link, riscv
from wireskein import capture as wc
from wireskein.sources import oep as wso

hst = link.open_host("oep://<プローブ>?via=usb")
core.take(hst, 30000, owner="capture-during-attach")
try:
    w = riscv.Wire(hst, "oep.wire.rvswd")
    def steps(wait):
        wait(0.01)
        conn, _ = w.attach(halt=True, pins=(18, 17))   # 取っている最中の操作
        w.detach(conn)
        wait(0.01)
    res = acquire.capture(hst, acquire.Request([("dio", "GPIO18"), ("clk", "GPIO17")], 40_000_000, 0,
                                               mode="stream", during=steps))
    wc.write("attach.wireskein", wso.capture_of(res))
finally:
    hst.end()
```

- 操作は、トリガを使って「arm してから操作する」形にすると、操作が起こした出来事でトリガが立ちます。
- classic ESP32 の sampler は、取っている窓の間は SWIO の線を止めます。操作の結果を写すにはトリガを使います
  （[使い始める](getting-started.ja.md#信号をキャプチャするesp32-p4)）。
- ロジックとアナログを同じトリガで切るときは capture-group を使います（`--analog` と `--trigger` を一緒に指定すると、WireSkein が組にします）。

## D. テストで照合する

波形を取るだけでなく、期待どおりかをテストで判定します。WireSkein の記録（runlog）にテストの手順と期待を書き、
`wireskein verify` で照合します。pytest では pytest-embedded-wireskein がテストごとの記録と照合を受け持ちます。

- 期待は `square`（周期と duty）、`uart`、`i2c`、`voltage` などで書きます。結果は測った値つきの合否で、JUnit XML にもなります。
- 線はラベルで指すと、治具の配線が変わってもテストのコードを変えずに済みます。
- 手順は [WireSkein のキャプチャの試験](https://github.com/Open-Embedded-Probe/wireskein/blob/main/docs/capture-test-guide.ja.md) にあります。
