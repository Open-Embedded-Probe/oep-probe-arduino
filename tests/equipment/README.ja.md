# 実機の適合検査

この入口は、実機の probe で動いている firmware を、選んだ oep-spec の checkout に照らして検査します。検査器は Python client の `oep_client.conformance` です。firmware の書き込みや設定の変更はしません。

## 流し方

[tests/.env.example](../.env.example) を `tests/.env` に写し、`OEP_CONFORMANCE_*` と、共有の `OEP_HW_LOCK` を設定します。

| 変数 | 中身 |
|---|---|
| `OEP_CONFORMANCE_ADDRESS` | probe の口（`/dev/ttyACM0`、`tcp://HOST:PORT` など） |
| `OEP_CONFORMANCE_UNIT_ID` | 検査する個体の unit ID。ほかの個体なら止まる |
| `OEP_CONFORMANCE_SPEC` | 照らす oep-spec の checkout |
| `OEP_CONFORMANCE_OUT` | 結果の JSON を書く、まだ無いファイルの path |
| `OEP_HW_LOCK` | 設備を使う全員が共有する、既にあるロックのファイル |
| `OEP_CONFORMANCE_FRAMING` | 任意。`serial` は COBS / CRC、線の誤り、同じ口の開き直しでのセッションの保持を、client から独立に確かめる。`tcp` は長さと誤りを確かめる |
| `OEP_CONFORMANCE_TCP_PEER` | 任意。`tcp` で、同じ probe に 2 本目の接続を張れるときの口。2 接続の項目を足す |

```sh
uv run --project tests --env-file tests/.env pytest tests/equipment --junitxml=/absolute/path/to/new-run.xml
```

検査を足している途中の client を使うときは、`--with-editable /absolute/path/to/oep-client-python` を付けます。

## 結果

- core の項目は、項目ごとに pytest / JUnit の結果になります。インターフェースは、全 fn の共通の宣言を 1 つの結果で確かめます。インターフェースごとの動作は、この入口では確かめません。
- JSON には、送受信した byte、SPEC の commit と dirty とハッシュ、検査器のハッシュ、firmware の版、確かめなかった範囲が残ります。
- 何も設定しないときだけ skip になります。一部だけの設定、接続や個体の誤り、仕様との食い違いは FAIL です。transport か後片付けが失敗した後の項目は BLOCKED で、全体は FAIL になります。
- 再起動や個体の変化を見つけたら、そこで止めます。pin、target、設定には触りません。force は、検査器が自分で開いたセッションの間でだけ試します。

検査器が確かめる範囲は、client の [適合検査](https://github.com/Open-Embedded-Probe/oep-client-python/blob/main/docs/conformance-checks.ja.md) にあります。client が自分の模型で確かめる項目（仮想の probe、論理の fixture）は、client の試験に含まれ、この入口では流しません。
