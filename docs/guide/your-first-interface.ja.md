# はじめてのインターフェース

[English](your-first-interface.md)

OEP の probe に、自分の機能（インターフェース）を足す手順です。ピンを使わない小さなものから始め、最後にピンを使うものへ
進みます。`examples/02.Interfaces/HelloInterface` と `CustomInterface` を開きながら読んでください。もっと詳しい規則は
[probe を書く](writing-a-probe.ja.md) にあります。

## 1. インターフェースとは

probe は、host（PC）から届く要求に答える小さな機械です。要求は「どのインターフェースの、どの操作（op）を、どんな中身
（payload）で」の 3 つでできています。

```text
host --要求: fn 1, op 0x10, payload--> Endpoint --> あなたのインターフェース（handle）
host <--結果: completed, payload------ Endpoint <--
```

- **Endpoint** はライブラリが持つ部分です。フレームの読み書き、セッションとロック、どのインターフェースへ渡すかを受け持ちます。
- **インターフェース**はあなたが書くクラスです。名前を持ち、op を受けて結果を返します。
- endpoint は、足した順にインターフェースへ番号（fn 1、2、...）を振ります。host は番号ではなく名前で探すので、番号は
  気にしなくてかまいません。

## 2. 名前を決める

名前は、あなたが持っているものを逆さにした形（逆 DNS）にします。GitHub のアカウントなら `io.github.<you>.<name>` です。
誰の承認も要りません。`oep.*` は仕様が決める標準のインターフェースのためのもので、使いません。

```cpp
const char *name() const override { return "io.github.example.hello"; }
uint16_t instance() const override { return 0; }   // 同じ名前の何個目か。1 つだけなら 0
uint8_t revision() const override { return 1; }     // payload の形の版
```

payload の固定の部分の形を変えたら、revision を上げます。新しい op を足すだけなら上げません。

## 3. op を決める

HelloInterface は合計を数えるだけの例です。

| op | 名前 | 要求の payload | 結果の payload | ロック |
|---|---|---|---|---|
| 0x10 | add | value（u32） | total（u32） | 要る（合計を変える） |
| 0x11 | total | なし | total（u32） | 要らない（何も変えない） |

自分の op は 0x10〜0xFF から選びます。0x01〜0x0F はどのインターフェースにも共通の操作の番号で（0x01 subscribe、0x02 unsubscribe。
core §2.5）、ここに自分の op を置くと endpoint が受け付けません（unknown operation）。

```cpp
bool offers(uint8_t op) const override { return op == kOpAdd || op == kOpTotal; }   // 持っている op
bool lockFree(uint8_t op) const override { return op == kOpTotal; }                 // 何も変えない op
```

- `offers` で持っている op を言います。ほかの op には endpoint が `unknown_operation` で答えるので、自分で断る必要はありません。
- 何かを変える op は、ロックを持つセッションからしか届きません。endpoint が確かめます。何も変えない読み出しは `lockFree` に
  して、ロックなしでも読めるようにします。
- 数値はリトルエンディアンです（`oep::getU32` / `oep::putU32`）。

## 4. 要求に答える

```cpp
oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override {
  oep::Tail tail;
  const oep::Result parsed = oep::plainTail(tail, payload, length, op == kOpAdd ? 4 : 0, out, capacity);
  if (oep::refused(parsed)) return parsed;   // 短すぎる、または無視できない TLV が付いていた
  if (capacity < 4) return oep::failed();
  if (op == kOpAdd) total_ += oep::getU32(payload);
  oep::putU32(out, total_);
  return oep::completed(4);                  // 成功。結果の payload は 4 バイト
}
```

- 結果の種類は次の 3 つです。
  - `completed(n)`：成功。`out` に n バイトを書いた。
  - `failed()`：実行したが、うまくいかなかった。
  - `rejected(理由)`：実行しなかった（`kRejectMalformed` など）。
- どの要求も、固定の部分の後ろに TLV（追加の情報）を付けられます。`plainTail` に固定の部分の長さを渡すと、長さの確かめと TLV の
  扱いをまとめて行います。最初はこの 2 行をそのまま書けば十分です。
- `handle` の中で待たないでください。時間のかかることは `loop()` で少しずつ進めます（§7）。

## 5. 足して、試す

```cpp
static Hello hello;
void setup() {
  // ...endpoint の用意（MinimalProbe と同じ）
  endpoint.add(hello);
}
void loop() { endpoint.poll(); }
```

焼いたら、PC で Python のクライアントから呼びます。

```python
from oep_client import core, link
hst = link.open_host("<port>"); hst.open(3000)          # セッションを開き、ロックを取る
fn = core.find(hst, "io.github.example.hello")          # 名前で探す
hst.call(fn, 0x10, (5).to_bytes(4, "little")).payload    # b'\x05\x00\x00\x00'
hst.call(fn, 0x11, locked=False).payload                 # ロックなしで読む
```

`oep dump --port <port>` を使うと、probe が何を持っているか（あなたのインターフェースと、その ops）が見えます。

## 6. セッションが終わったら片付ける

host がセッションを終えたとき（end、リースの期限切れ、ほかの host による強制、どれも同じ）に `sessionOver()` が呼ばれます。
そのセッションが作ったものを、ここで元に戻します。次のセッションには何も残しません。

```cpp
void sessionOver() override { total_ = 0; }
```

## 7. ピンを使う

ピンを動かすインターフェースは、どのピンを使うかを自分で決めません。host が実行中に **plan** で割り当てます。こうすると、
同じ firmware をどの治具でも使え、2 つのインターフェースが同じピンを駆動することもありません。`CustomInterface` が
その例です（LED を点滅させる `io.github.example.blink`）。

1. **役（role）を決める**：「1 = 出力の線」のように番号を付けます。
2. **describe で候補を宣言する**：`w.roleChannels(roles, n, pins.allowedMask())`。host はこれを読んで割り当てます。
3. **plan の 3 つを書く**：
   - `planRoles()` は true を返します。
   - `planCheck()` は何も変えずに確かめます。宣言していない役やピンなら `kRejectUnsupported`、ほかが使っているピンなら `kRejectUnavailable` を返します。
   - `planApply()` で `pins.claim(channel, 持ち主)` してピンを取り、`planRelease()` で `pins.release(持ち主)` して返します。
4. **持ち主の番号**は `oep::owners::kSketch`（0x80）から上を使います。ライブラリの部品は `oep::owners` の番号を使うので、
   重なりません。
5. ピンは、最初の op が来たときに初めて駆動します。plan を受けただけでは何も変えません。

時間のかかる動き（点滅など）は、`handle` ではすぐに答え、`loop()` から呼ぶ `poll()` で少しずつ進めます。

## 8. 次に

- できあいの firmware（`Firmware/OepProbe`）に、自分のインターフェースを足す：[firmware の機能](firmware-features.ja.md)
- 通知を送る、設定を持つ、debug の線など、詳しい規則：[probe を書く](writing-a-probe.ja.md)
- 仕様そのもの：oep-spec の `docs/oep-core.ja.md`（§2 要求と結果、§6 ロック、§7 describe、§8 plan）
