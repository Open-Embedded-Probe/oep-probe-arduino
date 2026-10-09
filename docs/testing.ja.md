# プローブ実装と更新の検証

プローブ firmware の実装品質、image、更新、更新後の復帰と利用経路への影響は、このリポジトリが検証・判定します。テストコードが Python client 側にあっても責任は移りません。

共通の保証対象と判定は [OEP 全体のテスト方針](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/testing-policy.ja.md)、設置済みプローブを使う側の責任は [利用プロジェクト向けガイド](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/testing-consumers.ja.md)を参照します。通信の規範は oep-spec が所有し、設備設定の TOML を wire protocol の一部にしません。

Arduino の実行環境は `tests/` の uv / pytest workspace にまとめます。[実機転送の入口](firmware-transfer.ja.md)と `tests/.env.example` を参照してください。通常の運用は候補を転送してから利用し、Flash 退避や旧 firmware の復元を前提にしません。

## 現在の入口と保証範囲

| 入口 | 保証する範囲 |
|---|---|
| `cd tests && uv run pytest` | 転送判定、既存 host 検査、release profile 検査。実機は操作しない |
| `uv run --env-file .env pytest ../examples/Firmware/OepProbe/test_transfer.py --profile esp32` | classic ESP32 の build・個体照合・転送・宣言と設定／slot の再確認 |
| `tests/host/run.sh` | host 上の shim と C++ による portable な protocol・状態・資源の検査 |
| `python3 tests/compile/test_profiles.py` | profile/model の定義の回帰検査 |
| `.github/workflows/tests.yml` | host 検査と P4/P4X 等の build。実機の電気動作の保証とは別 |
| Python client の `tests/hw` | 独立 host による実機の結合検査。現行設定の制約はその README に記載 |
| `.github/workflows/firmware.yml` | image と manifest/hash の生成。実機での起動・復帰は別途検査 |

新しい設備形式には offline loader/planner と仮想 smoke があり、[最小構成の確認](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/hardware-quickstart.ja.md)から試せます。実機 preflight と複数 target の共有 session adapter は未実装です。この文書を追加したことで CI に実機 gate が導入されたわけではありません。

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
