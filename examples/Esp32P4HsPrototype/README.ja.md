# Esp32P4HsPrototype

oep-spec の `docs/probe-cdc-and-persistence.ja.md`（シリアルの口と永続化） §7 の試作 P1〜P5 を 1 つにした P4 HS の probe です。target は要りません。
仕様を固める前の実験用で、ここでの振る舞いは仕様ではありません。

| 試作 | 中身 | host 側 |
|---|---|---|
| P1 / P2 | vendor bulk、HID、CDC の 3 つの経路で 1 つのセッションとロックを共有する | `host/transports.py <OEP の CDC の tty>` |
| P3 | oep.fixture.uart（TX 20 / RX 21）をデータの CDC の口に流す。line coding で UART を掛け直す | `host/bridge.py <UART bridge の tty>`、`host/bridge_closed.py <tty>` |
| P4 | oep.probe.config（boot_mode、plan、bind を NVS に保存）と起動モード | `host/config.py setup / check <tty> / mode0 / mode1 / look / erase / reboot` |
| P5 | DFU（ダウンロード形）と Mass Storage での probe 自身の更新 | `host/dfu_dl.py <.bin>`、`host/build_time.py` |

- 線はつながなくてよい。P3 は GPIO マトリクスで UART1 の RX を自分の TX のパッドから取って折り返す（`oep.test.bridge` 0x02）。
- direct build なので、`build_opt.h` を変えたら `arduino-cli compile --clean` でビルドする。
- USB の識別子は 303a:4021、serial は MAC + `-hs`。WSL では再起動（reboot、DFU、Mass Storage の更新）のたびに usbipd の attach が要る（`host/reattach.sh <開始時刻> <busid>`）。
- host の Python は oep-client-python を `../../../../oep-client-python/src` から読む（dev_oep の下に並べた checkout）。
- 結果は oep-spec の probe-cdc-and-persistence.ja.md §7.1〜§7.4。
