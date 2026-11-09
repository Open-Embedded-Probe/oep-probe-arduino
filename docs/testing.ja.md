# プローブ実装と更新の検証

プローブ firmware の実装品質、image、更新、更新後の復帰と利用経路への影響は、このリポジトリが検証・判定します。テストコードが Python client 側にあっても責任は移りません。

共通の保証対象と判定は [OEP 全体のテスト方針](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/testing-policy.ja.md)、設置済みプローブを使う側の責任は [利用プロジェクト向けガイド](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/testing-consumers.ja.md)を参照します。通信の規範は oep-spec が所有し、設備設定の TOML を wire protocol の一部にしません。

Arduino の実行環境は `tests/` の uv / pytest workspace にまとめます。[実機転送の入口](firmware-transfer.ja.md)と `tests/.env.example` を参照してください。通常の運用は候補を転送してから利用し、Flash 退避や旧 firmware の復元を前提にしません。

## 現在の入口と保証範囲

| 入口 | 保証する範囲 |
|---|---|
| `cd tests && uv run pytest` | 転送判定、既存 host 検査、release profile 検査。実機は操作しない |
| `uv run --env-file .env pytest -c pyproject.toml ../examples/Firmware/OepProbe/test_transfer.py --profile esp32` | 選択した ESP32 / P4 / P4X / Pico の build・個体／image 照合・転送・再列挙・宣言と設定／slot の再確認 |
| `tests/host/run.sh` | host 上の shim と C++ による portable な protocol・状態・資源の検査 |
| `uv run --project tests --env-file tests/.env pytest tests/equipment`（[実機の適合検査](../tests/equipment/README.ja.md)） | 実機の firmware を、選んだ oep-spec の checkout に照らした core と共通の宣言の検査。書き込みと設定の変更はしない |
| `cd tests && uv run pytest oneshot/<名前> --profile <profile> --port <ポート>`（下の「単発の試験」） | 決まった機材と配線でだけ流せる実機の試験。プラグインがビルドし、機器ロックの中でそのポートから書き込んでから確かめる。自動では集めない |
| `python3 tests/compile/test_profiles.py` | profile/model の定義の回帰検査 |
| `.github/workflows/tests.yml` | host 検査と P4/P4X 等の build。実機の電気動作の保証とは別 |
| Python client の `tests/hw` | 独立 host による実機の結合検査。現行設定の制約はその README に記載 |
| `.github/workflows/firmware.yml` | image と manifest/hash の生成。実機での起動・復帰は別途検査 |

新しい設備形式には offline loader/planner、仮想 smoke、共通ロックと結果保存を伴う実機 probe preflight があり、[最小構成の確認](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/hardware-quickstart.ja.md)から試せます。probe preflight の成功で wire/DM・fixture・USB pair の適合を認定しません。

## 単発の試験

決まった機材や配線でだけ流せる実機の試験は、`tests/oneshot/<名前>/` に 1 つずつ置きます。`tests/pyproject.toml` の
testpaths に入れないので、`uv run pytest` では集めません。流すときはフォルダを名指し、機材のポートを `--port` で渡します。
ビルド、書き込み、機器ロックは pytest-embedded-arduino-cli のプラグインの既定のとおりです。

- フォルダはプラグインのスケッチのフォルダです。配布の firmware を試すときは、`.ino` に
  `#include <../examples/Firmware/OepProbe/OepProbe.ino>` だけを書きます（ライブラリの src がインクルードのパスにあり、
  firmware の中のヘッダは元の場所から見つかります）。`sketch.yaml` は必要な profile だけを、配布の firmware と同じに書きます。
- インクルードできない `build_opt.h` は、配布の firmware と同じにします（firmware に無ければ置きません）。`tests/unit/test_oneshot_sketches.py` が
  同じかを確かめます。
- 試験ファイルの先頭に、使う機材、配線、流し方を書きます。

| フォルダ | 確かめること | 機材 |
|---|---|---|
| `p4_uart_bridge` | ESP32-P4 / P4X の UART bridge（GPIO37 / 38）から書き込み、同じ口で OEP が答え、その 2 本を fixture に出さない | GPIO37 / 38 に USB-UART を持つ P4 のボード（ベンチの 2-3: CH340C） |
| `esp32_uart_bridge` | classic ESP32 の UART bridge（UART0）から書き込み、同じ口で OEP が答え、Wi-Fi の TCP の待ち受けが起動時に作られて経路に並ぶ | UART0 に USB-UART を持つ classic ESP32 のボード（ベンチの 1-1） |

## 候補 firmware を検証する順序

1. 変更で影響する契約と platform/profile/transport の差分を選び、必須と対象外を明記します。
2. 単体・生成整合・build を通し、候補 image の source commit/dirty、SPEC 対応版、profile、hash を記録します。
3. 設備設定、個体、給電、配線を照合し、共有ロックを取得します。変更前の firmware と保存設定、更新後の終了状態の方針を記録します。
4. 対象 image と更新経路を明示して更新します。再接続後に identity、申告版、revision、ops/limits、slot/bind と設定を照合します。
5. protocol/session/config、transport、wire/DM、fixture/capture と、変更に関係する実機契約を検査します。宣言漏れを設備不足による skip にしません。
6. 影響する利用プロジェクトの既存テスト入口を使い、flash/debug、upload/monitor、DUT 動作等を回帰確認します。利用側の期待値を設備設定で緩めません。
7. 更新と回帰の結果、未検証の軸、cleanup と残った image/設定を保存し、必須契約が成立した範囲でリリース判断します。

probe の版は提供側で管理し、利用側へ最低 firmware 版の設定や自動更新を一律に要求しません。実装する SPEC と宣言機能、互換性、不具合情報は提供側で説明します。更新後に構成が変わる場合は、必要な配線・能力を再確認して設備の確定記録を更新します。

## 追加する実機契約

共有ベクタと mock の成功に加え、実 transport の境界・切断・再列挙、wire の候補探索と曖昧性、既存接続の維持、独立基準による fixture/capture、失敗後の駆動解放と設定復元を確認します。

複数 target を採用する platform では、connection/console/config の上限を実装して宣言し、同じ probe の A/B の reset・操作・stream が混線しないことを確認します。上限1の platform を、同時接続が必要な契約へ割り当てません。target の USB data と probe 自身の USB transport は別の契約として検査します。

## 実時間に依存する host 検査

`tests/host/test_wire_gate.cpp` は host の 2 つのスレッドで、実時間の短い波形を作って取ります。host の負荷に左右されるので、
`tests/host/run.sh` は 3 回まで試し、3 回とも落ちたときだけ失敗にします。そのときは、負荷の少ないときに単独で流し直します。
ここで通っても、実機の時間の精度を保証したことにはなりません。
