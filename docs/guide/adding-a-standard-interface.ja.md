# 標準インターフェースを足す（ライブラリに貢献する人へ）

[English](adding-a-standard-interface.md)

oep-spec が定める `oep.*` のインターフェースを、このライブラリと配布する firmware に足す手順です。自分のインターフェース
（`io.github.<you>.*`）を自分の probe に足すだけなら、この手順は要りません。[はじめてのインターフェース](your-first-interface.ja.md)
と [firmware の機能](firmware-features.ja.md) を見てください。

上から順に進めます。最近の例として、`oep.fixture.pwm` / `counter` / `dac`（commit 9317805）がそのまま参考になります。

## 1. 仕様と番号

- [ ] oep-spec に、インターフェースの文書（`interfaces/oep-if-*.ja.md` の節）と、`registry/oep-v1.toml` の登録（名前、op、TLV の tag、
      列挙）があります。**wire 上の数値の定義は registry だけ**です。ライブラリでは数値を書かず、名前で引きます。
- [ ] `tools/sync_registry.sh`（`OEP_SPEC_REF` で commit を選ぶ）で、`src/OepRegistry.h` と `tests/vectors/` を写します。
      `tests/vectors/SPEC_COMMIT` が更新されます。写したものは編集しません。

## 2. クラス

- [ ] `src/Oep<Name>.h` / `.cpp` を作り、`oep::Interface` を継承します。冒頭のコメントには次を書きます。
  - 従う仕様の節
  - op の一覧（`0x10 set(...) -> ...` の形）
  - チップごとの実装の違い
  - 実機で確かめていないなら、そのこと（例 `NOT YET RUN ON HARDWARE`）
- [ ] 名前、revision、op は `reg::<interface>::kName` / `kRevision` / `kOp...` から取ります。describe の tag も registry のものを
      使います。
- [ ] `offers()` には、表の必須の op すべてと、この実装が持つ任意の op を挙げます。`lockFree()` は何も変えない op（status など）だけにします。
- [ ] ピンを使うなら次の 2 つをします。
  - **持ち主の番号**を `src/OepPinTable.h` の `oep::owners` に足し、クラスでは `kOwnerId = owners::k<Name>` とします。
    同じ firmware に同時に入らない実装どうし（チップごとの SPI target など）は、同じ番号を共有してかまいません。
  - plan の 4 つ（`planRoles`、`planCheck`、`planApply`、`planRelease`）を、既存の fixture（`OepPwm.cpp` など）と同じ形で書きます。
    describe の `roleChannels` で宣言した役とピンだけを受けます。
- [ ] `sessionOver()` で、セッションが作ったものをすべて戻します。待つ処理は `handle()` に置かず、`poll()` を `loop()` から呼びます。
- [ ] チップに依存するコードは `#if defined(ARDUINO_ARCH_...)` で分けます。持たないチップでは、インターフェースを出さないか、
      describe で何も宣言しないようにします。host の試験のために、偽物に差し替える `OEP_HOST_FAKE_<NAME>` を用意します。

## 3. 大きさ（CH32X035）

- [ ] X035（62 KiB）には既定で入れません。`.cpp` 全体を `#if !defined(ARDUINO_CH32X035)` で囲みます。
      ライブラリの `.cpp` はリンクの対象になり、そこから参照するだけで core の割り込みハンドラなどが入ってしまうためです。
      `Ch32.h` では `kRoomy` の時だけ足します。
- [ ] X035 のビルドの大きさが変わっていないことを確かめます（`--clean` で。古い include のキャッシュが残ることがあります）。

## 4. host の試験

- [ ] `tests/host/test_<name>.cpp` を書き、`tests/host/run.sh` に足します。偽物（`OEP_HOST_FAKE_<NAME>`）の上で、op、plan、
      断るときの理由、セッションの終わりの片付けを確かめます。
- [ ] `tests/host/run.sh` がすべて通ること。`test_vectors.cpp` は仕様の共通の vector を endpoint に当てます。

## 5. firmware に配線する

チップのヘッダ（`examples/Firmware/OepProbe/Rp2.h`、`Esp32P4.h`、`Esp32.h`、`Ch32.h`）のうち、持つものすべてに次を足します。

- [ ] `#include <Oep<Name>.h>`
- [ ] `void oepAdd<Name>()`：`static oep::Slot<oep::<Class>> x; endpoint.add(x.make(pins));`。静的な RAM が足りないチップでは
      `HeapSlot` を使います（classic ESP32 の DRAM）。`loop()` の仕事は `loopWork.add(...)` で渡します。
- [ ] `__attribute__((weak)) void oepWith<Name>() { oepAdd<Name>(); }`（既定で入れないなら `{}`。CH32 は `if (kRoomy)`）
- [ ] `setup()` の `oepWith...()` の並びの**最後**に `oepWith<Name>();` を足します。途中に入れると、既存の fn の番号と保存した
      設定の読み戻しがずれます。
- [ ] `OepFeatures.h` に `oepWith<Name>()` と `oepAdd<Name>()` を宣言します。

## 6. ビルド

- [ ] 全プロファイルをビルドします：`rp2040`、`rp2350`、`esp32p4`、`esp32p4x`、`esp32`、`ch32x035`、`ch32v307`、`ch32h417`。CI（`.github/workflows/`）
      も examples の `sketch.yaml` をすべてビルドします。

## 7. 文書

- [ ] [probe を書く](writing-a-probe.ja.md) §5 の表（英・日）
- [ ] [ボード](boards.ja.md) の表（英・日）：チップごとの有無と上限、確かめたかどうか
- [ ] `examples/Firmware/OepProbe/OepProbe.ino` の冒頭の一覧、[firmware の機能](firmware-features.ja.md) の X の表
- [ ] README の構成の表（英・日）、`CHANGELOG.md`

## 8. host 側と実機

- [ ] oep-client-python にクラス、単体の試験、hw の試験を足します。hw の試験は、配線が要るなら label で opt-in にし、無ければ
      skip します。配線を固定で仮定しません。oep-client-js にも足します。
- [ ] ベンチで hw の試験を通し、結果（`tests/hw/results`）を残します。確かめられないものは、確かめていないとクラスの冒頭と
      ボードの表に書きます。
