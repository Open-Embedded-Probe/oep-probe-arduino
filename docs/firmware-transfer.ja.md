# uv / pytest によるプローブ転送

Arduino 側の Python 環境は `tests/` にまとめます。`pytest-embedded-arduino-cli` が pinned profile の build と Arduino CLI upload を担当し、OEP client が転送前の個体照合と転送後の通信を検査します。依存は `tests/pyproject.toml` と `tests/uv.lock` で管理します。入口の設計はプラグインの [初回テストガイド](https://github.com/tanakamasayuki/pytest-embedded-arduino-cli/blob/main/FIRST_TEST.ja.md)に沿います。

## 最小入口

現時点の実機入口は、既に OEP が動作する **classic ESP32** の `examples/Firmware/OepProbe` です。RP2 / P4 の更新経路や、新品・応答しない個体の初回書込みはこの gate の対象外です。接続機器の自動検索や、全機器への転送は行いません。

```sh
cd tests
uv sync --locked
cp .env.example .env
# .env の port、unit ID、候補版、既存の共通ロック、今回の結果保存先を編集
uv run --env-file .env pytest ../examples/Firmware/OepProbe/test_transfer.py \
  --profile esp32 --run-mode=all -vv --junitxml=build/transfer.xml
```

`arduino-cli` と profile が解決できる package index が必要です。profile の ESP32 core は `sketch.yaml` で版を固定しています。Python plugin の `dut` fixture は使用せず、upload 後は OEP client が binary serial port を所有します。テキスト monitor と同じ port を同時に開きません。

`--run-mode=all` は build → 個体照合 → upload → OEP 確認です。`--run-mode=test` も既存 build を **転送してから**確認します。build だけなら設備設定や実機は不要です。

```sh
uv run pytest ../examples/Firmware/OepProbe/test_transfer.py --profile esp32 --run-mode=build -vv
uv run pytest
```

最後のコマンドは実機を操作せず、転送判定の回帰検査、既存 C++ host 検査、release profile 検査を実行します。build モードの実機 test は build 完了後に skip と表示されます。

## 設定と排他

`.env` は Git 管理外で、明示した `uv run --env-file` のみが読み込みます。雛形は `tests/.env.example` です。私的設備リポジトリの配置には依存しません。

| 設定 | 意味 |
|---|---|
| `TEST_SERIAL_PORT_ESP32` | 対象個体の port。serial のない bridge の汎用 by-id は複数接続時に曖昧になるため、個体に対応する経路を渡す |
| `OEP_TRANSFER_UNIT_ID` | 転送前後に照合する OEP unit ID |
| `OEP_TRANSFER_FIRMWARE` | 今回転送する image の申告版。利用側に要求する最低版ではない |
| `OEP_HW_LOCK` | 全設備利用者が共有する、既存のロックファイル |
| `OEP_TRANSFER_RESULTS` | 今回専用の新しい結果ディレクトリ。既存なら拒否する |
| `OEP_TRANSFER_SPEC_COMMIT` | 候補が対応する SPEC commit。未確定なら `unknown` |

build 後、共有ロックを非待機で取得し、個体照合から upload、再接続、確認、結果保存まで保持します。plugin 自身の個体ロックと cleanup も維持します。別プロセスが共通ロックを保持していれば、転送に入る前に失敗します。port を `--port` でも渡す場合は `.env` の指定と同じ実体でなければ拒否します。

設備 TOML による配線探索・複数ターゲットの割当てと、この最小転送入口の接続は今後の実装です。ここでは必要最小限の設定を明示し、未確認の配線を推測して駆動しません。

## 確認範囲と終了状態

通常の Arduino CLI upload を使い、Flash の退避と旧 firmware の復元は行いません。転送した候補を残して利用します。ESP32 の通常 profile は NVS を全消去しませんが、保存設定の維持は転送後の実測で判定します。

`transfer.json` には source commit/dirty、SPEC commit、build image の SHA-256、個体、転送前後の版、宣言、保存設定、slot 状態、合否を記録します。upload 中の失敗は pytest の setup error とログにも残ります。結果には設備情報が入るため、Git 管理外の保存先を使います。

この入口は identity、confirm/list/describe と必須宣言、保存項目の一致、保存設定の適用、既存 slot の再接続を確認します。DUT の flash/debug、fixture/capture の精度、Arduino Core の upload/monitor の全面回帰まで通ったとは扱いません。それらは [プローブ検証方針](testing.ja.md)に沿って別の契約として追加します。
