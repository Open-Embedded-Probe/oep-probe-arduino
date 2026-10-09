# uv / pytest によるプローブ転送

Arduino 側の Python 環境は `tests/` にまとめます。`pytest-embedded-arduino-cli` が pinned profile の build と Arduino CLI upload を担当し、OEP client が転送前の個体照合と転送後の通信を検査します。依存は `tests/pyproject.toml` と `tests/uv.lock` で管理します。入口の設計はプラグインの [初回テストガイド](https://github.com/tanakamasayuki/pytest-embedded-arduino-cli/blob/main/FIRST_TEST.ja.md)に沿います。

## 最小入口

実機入口は、既に OEP が動作する `examples/Firmware/OepProbe` です。profile と対象個体は毎回明示します。新品・応答しない個体の初回書込みは対象外です。全機器への自動転送は行いません。

| profile | 転送経路 | 対象 |
|---|---|---|
| `esp32` | 通常の Arduino CLI upload | classic ESP32 の UART bridge |
| `esp32p4` | Arduino CLI の upload recipe から dfu-util | P4 v0.1–v1.99、HS DFU |
| `esp32p4x` | 同上。P4X の app image | P4 v3.1–v3.99、HS DFU |
| `rp2040` | Arduino CLI recipe から、選択した BOOTSEL の picotool / UF2 書込み | Pico等。物理ボードに合う profile を使う |
| `rp2350` | 同上 | Raspberry Pi Pico 2 |
| `promicrorp2350` | 同上 | SparkFun Pro Micro RP2350。今回の実機確認対象外 |

```sh
cd tests
uv sync --locked
cp .env.example .env
# .env の port、unit ID、候補版、既存の共通ロック、今回の結果保存先を編集
uv run --env-file .env pytest -c pyproject.toml ../examples/Firmware/OepProbe/test_transfer.py \
  --profile esp32 --run-mode=all -vv --junitxml=build/transfer.xml
```

`arduino-cli` と profile が解決できる package index が必要です。profile の ESP32 core は `sketch.yaml` で版を固定しています。Python plugin の `dut` fixture は使用せず、upload 後は OEP client が binary serial port を所有します。テキスト monitor と同じ port を同時に開きません。

`--run-mode=all` は build → 個体・silicon・image 照合 → upload → OEP 確認です。`--run-mode=test` も既存 build を **転送してから**確認します。build だけなら設備設定や実機は不要です。

```sh
uv run pytest -c pyproject.toml ../examples/Firmware/OepProbe/test_transfer.py --profile esp32 --run-mode=build -vv
uv run pytest
```

最後のコマンドは実機を操作せず、転送判定の回帰検査、既存 C++ host 検査、release profile 検査を実行します。build モードの実機 test は build 完了後に skip と表示されます。

## 設定と排他

`.env` は Git 管理外で、明示した `uv run --env-file` のみが読み込みます。雛形は `tests/.env.example` です。私的設備リポジトリの配置には依存しません。

| 設定 | 意味 |
|---|---|
| `TEST_SERIAL_PORT` | 対象個体の port。serial のない bridge の汎用 by-id は複数接続時に曖昧になるため、個体に対応する経路を渡す |
| `OEP_TRANSFER_UNIT_ID` | 転送前後に照合する OEP unit ID |
| `OEP_TRANSFER_FIRMWARE` | 今回転送する image の申告版。利用側に要求する最低版ではない |
| `OEP_HW_LOCK` | 全設備利用者が共有する、既存のロックファイル |
| `OEP_TRANSFER_RESULTS` | 今回専用の新しい結果ディレクトリ。既存なら拒否する |
| `OEP_TRANSFER_SPEC_COMMIT` | 候補が対応する SPEC commit。未確定なら `unknown` |
| `OEP_TRANSFER_ALLOW_UNREADABLE_CONFIG` | `1` の場合だけ、既知の旧 image の更新前の設定・宣言読出し失敗を記録して進む。更新後の基準は緩めない |
| `OEP_HW_UF2_DRIVE` | Pico の BOOTSEL がマウントされる明示パス。元の USB 接続位置と一致しないドライブには書かない |
| `OEP_TRANSFER_RESUME_FROM` | この gate が個体確認後に BOOTSEL で停止したときの前回 `transfer.json`。Pico系のみ |

build 後、共有ロックを非待機で取得し、個体照合から upload、再接続、確認、結果保存まで保持します。この gate の設備排他は共有ロックが担います。plugin の build と公開 flasher fixture を使い、upload fixture で対象を限定した recipe を渡します。別プロセスが共通ロックを保持していれば、転送に入る前に失敗します。port を `--port` でも渡す場合は `.env` の指定と同じ実体でなければ拒否します。

設備 TOML による配線探索・複数ターゲットの割当てと、この最小転送入口の接続は今後の実装です。ここでは必要最小限の設定を明示し、未確認の配線を推測して駆動しません。

## 確認範囲と終了状態

通常の Arduino CLI upload を使い、Flash の退避と旧 firmware の復元は行いません。転送した候補を残して利用します。ESP32 の通常 profile は NVS を全消去しませんが、保存設定の維持は転送後の実測で判定します。

`transfer.json` には source commit/dirty、SPEC commit、build image の SHA-256、個体、転送前後の版、宣言、保存設定、slot 状態、合否を記録します。upload 中の失敗は pytest の setup error とログにも残ります。結果には設備情報が入るため、Git 管理外の保存先を使います。

この入口は identity、confirm/list/describe と必須宣言、boot ID の変化、保存項目の一致、保存設定の適用、既存 slot の再接続を確認します。DUT の flash/debug、fixture/capture の精度、Arduino Core の upload/monitor の全面回帰まで通ったとは扱いません。それらは [プローブ検証方針](testing.ja.md)に沿って別の契約として追加します。

## P4 / P4X と再列挙

古い firmware は v3.x silicon も `esp32p4` / ESP32-P4 と表示します。更新前に `chip` の実 revision を確認し、image のカスタム descriptor・対応 revision 範囲・ELF の model/product を検査します。更新後は `esp32p4x` / ESP32-P4X を必須とします。P4X に P4 用 image を渡して試すことはしません。

DFU は unit ID に一致する USB serial と DFU interface を明示し、app image だけを書きます。再起動後は製品名や tty 番号が変わっても、USB serial から CDC port を一意に探し直します。他個体へ fallback しません。

dfu-util が download 完了・成功状態の manifest を記録した直後に `LIBUSB_ERROR_NO_DEVICE` / exit 74 を返す場合は、再起動による切断として **判定を後段へ保留**します。実際に候補の個体・版・model・boot ID・宣言・設定を確認できて初めて PASS です。転送途中の失敗や一般の終了エラーを成功扱いしません。

upload recipe の上書きには Arduino CLI の [upload-property](https://docs.arduino.cc/arduino-cli/commands-reference/arduino-cli_upload)を使います。tool の選択後に適用される `upload.pattern` を指定します。

## Pico の BOOTSEL と失敗後の再開

更新前に個体 ID と USB bus / 接続ポート列を記録し、確認した CDC port を1200 baudで BOOTSEL へ移行します。同じ USB 接続位置の、期待する RP2040 / RP2350 bootloader だけを採用します。複数の候補や別の接続位置は拒否します。

picotool を使う場合は core に同梱されたものを bus/address 指定で呼びます。一般ユーザーによる USB bootloader へのアクセス権が必要です。明示した `OEP_HW_UF2_DRIVE` を使う場合は、INFO_UF2 と mount の sysfs 接続元を照合して、その一つのドライブへだけ書きます。core の UF2 全ドライブ探索は使いません。自動マウントで user ACL が設定されるまでの短い遅延は待ちます。

BOOTSEL のまま停止した場合、結果に残った `transfer.json` を `OEP_TRANSFER_RESUME_FROM` で明示します。次回の結果は新しいディレクトリへ保存します。前回の個体・profile と現在の bootloader の接続位置を照合し、候補を再転送してから確認します。前回確認した個体を差し替えていないことが前提です。任意の bootloader を「1台だけだから」と選びません。

旧 image の設定が読めないときや、更新後の storage が unreadable のときは保存設定の維持・適用を保証できません。`settings_preservation` を unknown と記録し、更新後の読出しと宣言の検査は実行します。今回の確認では P4 / P4X、RP2040、Pico 2 の実機転送が通りました。RP2040 と Pico 2 は、BOOTSEL の udev 権限追加後に明示 bus/address の picotool 転送でも成功し、Flash verify・再起動・OEP 宣言・保存設定を確認しました。Pro Micro は入口を用意した段階です。

転送コマンドは sketch が `tests/` の外にあるため、`-c pyproject.toml` で test workspace の pytest 設定を明示します。これにより `probe_checks` 等の import と plugin 設定が安定します。picotool 経由では `OEP_HW_UF2_DRIVE` を未指定または空にし、結果保存先は毎回新しいディレクトリを指定します。Arduino CLI の `--port UF2_Board` は upload recipe のplaceholderであり、実際の転送経路は `transfer.json` の `upload_command` と `upload.log` で確認します。
