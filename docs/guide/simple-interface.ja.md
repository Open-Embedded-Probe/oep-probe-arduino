# 自分のコマンドを足す（SimpleInterface）

[English](simple-interface.md)

関数を書いて登録するだけで OEP のインターフェースになり、ホストから名前でも番号でも呼べるようにする部品
`oep::SimpleInterface` の使い方です。

使う場面の例:

- **プローブに処理を足す**: ライブラリの動作を試す、センサーを読む、決まった手順で周辺回路を動かす、データを集める。
- **試験の治具**: 相手のピンの変化を待って遅れを測る、決まった幅と数のパルスを出す、周波数を数える、電源を狙った時刻で切る。
- **OEP の core だけを話す自分の機器**: 温度計、電源装置、表示器など、ピンを OEP に任せない製品が、自分のインターフェース
  だけを持つ。

どの場合も、ホストは同じ `oep call` と `oep_client.call` で呼びます。

向かないものもあります。ピンの役を plan で選ばせる機能、ロックの規則が細かい機能、データを流し続ける機能、状態を多く持つ機能
（SWD やロジックのキャプチャのようなもの）です。これらは `oep::Interface` を直に実装します（[はじめてのインターフェース](your-first-interface.ja.md)）。

## 1. いちばん小さい例

```cpp
#include <OepFirmware.h>
#include <OepSimpleInterface.h>

oep::Result add(oep::Args &in, oep::Reply &out) {
  const int32_t a = in.i32();
  const int32_t b = in.i32();
  out.i32(a + b);
  return out.done();
}

oep::SimpleInterface mine("io.github.yourname.mine", 1);   // インターフェースの名前と revision

void setup() {
  mine.op(0x10, "add", {{"a", oep::i32}, {"b", oep::i32}}, {{"sum", oep::i32}}, add);
  oep::firmware.add(mine);   // 配布の firmware の機能はそのまま残る
  oep::firmware.begin();
}

void loop() { oep::firmware.loop(); }
```

配布の firmware に足すときの `sketch.yaml` と、機能の選び方は [firmware の機能](firmware-features.ja.md) にあります。

### core だけの自分の機器にする

[MinimalProbe](../../examples/01.Basics/MinimalProbe/MinimalProbe.ino) と同じく、自分のスケッチで `oep::Endpoint` を作り、
そこに足します。firmware の部品は何も入りません。

```cpp
void setup() {
  Serial.begin(115200);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  mine.op(0x10, "add", {{"a", oep::i32}, {"b", oep::i32}}, {{"sum", oep::i32}}, add);
  endpoint.add(mine);
}

void loop() { endpoint.poll(); }
```

### ホストから呼ぶ

```
$ oep dump --port /dev/ttyACM0
...
io.github.yourname.mine  rev 1
  add(a: i32, b: i32) -> sum: i32

$ oep call /dev/ttyACM0 io.github.yourname.mine add a=2 b=3
completed  rev 1  sum=5
```

## 2. 名前と番号

- **インターフェースの名前**は、自分の逆 DNS の名前にします（`io.github.<アカウント>.<名前>`、48 byte まで）。`oep.` で始まる名前は
  使えません。規則に合わない名前では、`op()` も `oep::firmware.add()` も false を返します。
- **revision** は 1 から始めます。コマンドの引数や答えの形を変えたら上げます。ホストは名前と revision で形を判断します。
- **op の番号**は 0x10〜0xFF から選びます（0x01〜0x0F はすべてのインターフェースに共通の op）。番号は自分で決め、変えません。
- **コマンドの名前**は 1〜24 文字の `a-z 0-9 - _` です。ホストは名前でも番号でも呼べます。
- 登録できないとき（番号や名前が重なる、欄の名前が重なる、型が無い）は、`op()` が false を返します。

## 3. 引数と答え

`op()` に、引数の欄と答えの欄を `{名前, 型}` の並びで渡します。関数の中では、登録した順に取り出し、登録した順に詰めます。
欄の名前と型は describe に出るので、ホストは何も知らなくても呼べます。

| 型 | 中身 | 取り出す / 詰める |
|---|---|---|
| `oep::u8` `u16` `u32` `u64` | 符号なしの整数 | `in.u8()` / `out.u8(v)` など |
| `oep::i8` `i16` `i32` `i64` | 符号付きの整数 | `in.i32()` / `out.i32(v)` など |
| `oep::boolean` | 真か偽 | `in.boolean()` / `out.boolean(v)` |
| `oep::bytes` | バイト列（長さ付き） | `in.bytes()` → `oep::Span` / `out.bytes(p, n)` |
| `oep::text` | 文字列（UTF-8、長さ付き） | `in.text()` → `oep::Span` / `out.text("...")`、`out.printf(...)` |
| `oep::list(oep::u32)` など | 整数か真偽の並び（数付き） | `in.count()` のあと要素を 1 つずつ / `out.list(p, 数, 1 要素のバイト数)` |

- **単位は欄の名前の終わりで示します**: `_ns`、`_us`、`_ms`、`_hz`、`_samples`（例: `width_ns`、`edges_samples`）。
  時刻をプローブの時計（`oep::nowNs()`）で返すと、キャプチャやログと同じ時間軸に並びます。
- 引数の長さは部品が確かめます。足りない要求は、関数を呼ばずに断ります（malformed）。
- 値の範囲は関数で確かめ、扱えない値なら `return out.unsupported();` を返します。今はできないときは `return out.busy();` です。
- 登録したとおりに答えを詰めずに `out.done()` を返すと、ホストには failed が届きます。
- 引数や答えのないコマンドは `{}` を渡します。

### 失敗を返す

受け付けたが、うまくいかなかったときは `out.fail(番号, "短い文")` を返します。番号の意味はそのインターフェースで決めます。

```cpp
oep::Result readSensor(oep::Args &in, oep::Reply &out) {
  if (!sensor.begin()) return out.fail(1, "no answer at 0x44");
  out.i32(sensor.temperature());
  return out.done();
}
```

ホストには、失敗した op の名前、番号、文が届きます（Python では `CommandFailed`）。

### ロック無しで受ける

読むだけで何も変えないコマンドには、印 `oep::kLockFree` を付けます。ホストはセッションのロックを取らずに呼べ、ほかの人が
ロックを持っている間も読めます。

```cpp
mine.op(0x20, "read", {}, {{"temp_mc", oep::i32}}, readTemp, oep::kLockFree);
```

## 4. 時間の掛かる処理（ワーカー）

コマンドの関数は、OEP の要求を処理するスレッドで動きます。すぐ終わる処理はそのまま書けます。
それより長い処理や、周辺回路を始める処理は `oep::Worker` に渡します。

```cpp
oep::Worker worker;

// ジョブ: ワーカーで動き、コマンドはすぐ答える。様子は status で見る
void drawFrame(oep::Args &in, oep::Worker &w) {
  const uint32_t color = in.u32();
  for (int row = 0; row < 64; ++row) {
    if (w.cancelled()) return;               // cancel が来た: status は cancelled になる
    if (!display.drawRow(row, color)) {
      w.fail(2, "no answer");                // status は failed、番号 2 と文になる
      return;
    }
  }
}

// ワーカーの中で動き、終わってから答える（周辺回路の begin / end、設定）
oep::Result beginBus(oep::Args &in, oep::Reply &out) {
  Wire.begin(kSda, kScl);
  Wire.setClock(in.u32());
  return out.done();
}

void setup() {
  mine.addWorker(worker, 0x20);   // status（0x20）と cancel（0x21）を足す。job() より先に
  mine.op(0x10, "begin", {{"clock_hz", oep::u32}}, {}, beginBus, oep::kInWorker);
  mine.job(0x11, "draw", {{"color", oep::u32}}, drawFrame);
  ...
}
```

- `job()` で登録したコマンドは、要求の欄を写してワーカーで動き始め、すぐ答えます。ジョブが走っている間に次のジョブや
  `kInWorker` の op を呼ぶと、断られます（unavailable）。
- `addWorker()` が足す status は、state（0 idle、1 running、2 done、3 failed、4 cancelled）、最後のジョブの op、失敗の番号と文、
  開始と終了の時刻（`start_ns`、`end_ns`、プローブの時計）を返します。ロック無しで呼べます。
- **中止は協調**です。cancel は印を立てるだけで、ジョブが `w.cancelled()` を見て抜けます。見ないまま固まったジョブは止められない
  ので、プローブを再起動します（`oep restart`）。
- `kInWorker` の op は、ワーカーの中で動き終わってから答えます。短い処理にしてください。

ワーカーが動く場所は石ごとに決まっています。

| 石 | ワーカー |
|---|---|
| ESP32 / ESP32-P4 | FreeRTOS のタスク。loop() と別のコア（ESP32-P4 ではコア 0）に固定。最初のジョブのときに作る |
| RP2040 / RP2350 | `worker.poll()` を `loop1()` から呼ぶ（2 つ目のコアで動く） |
| CH32 | 別のスレッドが無いので、`worker.poll()` を `loop()` から呼ぶ。動いている間は答えないので、短い処理だけにする |

ESP32 / ESP32-P4 と RP2 では、ジョブが動いている間も OEP の要求に答え続けます。

```cpp
void loop1() { worker.poll(); }   // RP2040 / RP2350
```

**割り込みを使う周辺回路（Wire、SPI など）は、ワーカーの中で始めてください**（`kInWorker` の op か、ジョブの中）。ESP32 では、
周辺回路の割り込みは `begin()` を呼んだコアに付きます。ESP32-P4 のコア 1 では USB とプローブ自身の処理が動いています。そこで
I2C の割り込みが遅れると、ESP-IDF の I2C のドライバが誤動作する例があります。

## 5. ログを付けてキャプチャと並べる

ログを足すと、コマンドの中で書いた行に、プローブの時計の時刻が付きます。ロジックのキャプチャと同じ時計なので、
行を波形のサンプルに置けます。

```cpp
#include <OepProbeLog.h>

oep::ProbeLog logbook(8192, 256);   // リング 8 KiB（2 の累乗）、印 256 個。heap から取る（logbook.ok()）

void drawFrame(oep::Args &in, oep::Worker &w) {
  logbook.line("## draw");
  ...
  logbook.printf("rows=%d", rows);
  logbook.line("##");
}

void setup() {
  mine.addLog(logbook);   // log-read、log-marks、log-clear を 0x11〜0x13 に足す
  ...
}
```

- `line()` と `printf()` は RAM に書くだけで、送り終わるのを待ちません（数百 ns）。ピンも要りません。どのスレッドから書いてもかまいません。
- `addLog()` は op 0x11〜0x13 を使います。この 3 つは自分のコマンドに使わないでください。
- `#` で始まる行は区切り（WireSkein の見出し）、ほかは出来事として記録されます。
- ホストでは、同じインターフェースの名前で `oep_client.probe_log.ProbeLog` から読みます。リングがあふれて行が欠けると、それも分かります。

## 6. ピン

配布の firmware に足すときは、コマンドが使うピンを起動時に取ります。取ったピンは、ほかのインターフェースが plan できなくなります
（ロジックのキャプチャは聞くだけなので、同じピンを取り込めます）。

```cpp
oep::firmware.onSetup([](oep::Endpoint &, oep::PinTable &pins) {
  pins.claim(kSda, oep::owners::kSketch);
  pins.claim(kScl, oep::owners::kSketch);
});
```

core だけの機器は channel を持たないので、何もしません。

## 7. ホストから呼ぶ

### コマンドライン

```
oep call PROBE INTERFACE OP [ARGS...] [--wait S]
```

- `OP` は名前（`add`）か番号（`0x10`）。
- 引数は `名前=値`。整数は 10 進か `0x`、真偽は `true` / `false`、並びは `1,2,3`、`bytes` は 16 進、`text` はそのまま。
- 欄の形を出していない op（標準のインターフェースの op など）は、16 進のバイト列で渡します（`--text` で文字列）。
- `--wait S` は、ジョブを始めて status で終わりを最大 S 秒待ち、status を出します。
- 答えは、結果（completed / failed / 断った理由）、revision、欄の値です。`--json` もあります。

### Python と pytest

```python
from oep_client import link, core
from oep_client import call as c

hst = link.open_host("/dev/ttyACM0")
core.take(hst, 10000, owner="my-test")

r = c.call(hst, "io.github.yourname.mine", "add", a=2, b=3)
assert r["sum"] == 5 and r.revision == 1

st = c.call_wait(hst, "io.github.yourname.mine", "draw", color=0xFF0000, timeout=5)   # ジョブを始め、status で終わりを待つ
print(st["start_ns"], st["end_ns"])
```

- 結果は欄の名前で引けます（`r["sum"]`、`r.sum`）。`r.revision` も確かめてください。番号と生のバイト列でも呼べます
  （`c.call(hst, name, 0x10, b"...")`、`r.payload`）。
- 誤りは例外です。
  - `CommandRejected`: 断られた（理由付き）。
  - `CommandFailed`: 失敗した（`fail()` の番号と文）。
  - `JobFailed`、`JobCancelled`、`JobTimeout`: ジョブの終わり方。`JobTimeout` は先に cancel を送ってから投げる。
  - `ProbeRestarted`: 待つ間にプローブが再起動した（boot_id が変わった）。
  - `InterfaceMissing`、`RevisionMismatch`、`OpMissing`: どれも `NotOffered` の子。pytest では `pytest.skip` に変えられる。
- ロック無しの op は、セッションの外から呼ばれます（`call()` が describe の印を見る）。`call_wait` の `wait=` には、ロジックの
  キャプチャの `during=steps` が渡す `wait` を渡せます。待つ間もキャプチャが続きます。

## 8. 例

- [extras/LibraryTest](../../extras/LibraryTest/README.ja.md): ライブラリの呼び出しをワーカーで動かし、そのバスをキャプチャし、
  ログの行を波形に置く（ジョブ、status、`result`、ログ）。
