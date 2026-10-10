# 独立した適合検査

仕様 → 検査 → コアの確定 → インターフェース共通契約 → OEPインターフェース固有契約 → 実装追従の順に進める。この入口は現行 firmware を指定した SPEC に当てる検査で、転送も仕様追従も行わない。

検査器は公開 Python client の `oep_client.conformance`。検査を追加中の checkout を使う場合は、その path を明示する。通常の tests プロジェクトの固定依存や隣接 checkout を自動で切り替えない。開発中は editable を使い、過去に作った wheel cache を最新の検査器と取り違えない。公開配布に検査器が含まれた後は `--with-editable` を省略できる。

```sh
cp tests/.env.example tests/.env
# .env の OEP_CONFORMANCE_* と共有 OEP_HW_LOCK を設定する
uv run --project tests --with-editable /absolute/path/to/oep-client-python \
  --env-file tests/.env pytest tests/equipment --junitxml=/absolute/path/to/new-core.xml
```

CLI plugin を含む既存 uv/pytest 環境内で実行する。SPEC checkout、unit ID、port/USB/TCP、既存共有 lock、新規 JSON path を指定する。私的ベンチリポジトリを参照しない。`OEP_CONFORMANCE_FRAMING=serial` を指定するとclientから独立したCOBS/CRCとwire faultと同一portのclose/openによるsession保持の検査も行う。再起動や個体変化を検出したら後続を止め、保持できたとは扱わない。pin・target操作・設定変更を行わず、forceは検査器自身が取得したsession間だけで試す。

`OEP_CONFORMANCE_FRAMING=tcp` と明示した `tcp://HOST:PORT` では独立したlength/fault検査を実行する。公開バーチャルベンチの `--profile core-v1` も同じ入口から検査できる（unit ID `virtual-core-1`、fn 0のみ、target/interfaceなし）。

同時に2接続できるTCP設備では `OEP_CONFORMANCE_TCP_PEER=tcp://HOST:PORT` を明示して追加6項目を実行する（54＋6＝60 core）。同じlistenerでよい。同一個体・boot・宣言を照合してから、共通lock/owner、応答と途中frameの接続分離、primaryのclose後のlock保持、peerだけの過大length切断を確認する。設定がない場合は追加項目を実行しない。設定した設備へ接続できない場合はFAILとし、fallbackしない。元のsessionを別のTCP接続へ移す検査は行わず、再接続後の後始末も再照合して自分のsessionだけを扱う。

core は case ごとの pytest/JUnit 結果、interface は全 fn の共通宣言検査、OEPインターフェース固有の動作は未実行。JSON には送受信 bytes、SPEC commit/dirty/hash、検査器 hash、実装版、未検査範囲が残る。全体適合を名乗らない。

全く設定していない任意入口のみ SKIP。部分設定、接続・権限・個体の誤り、旧仕様と新仕様の不一致は FAIL。transport/cleanup失敗の後の項目は BLOCKED となり、実行全体は FAIL。

既存 portable/C++ の期待値は `tests/vectors/SPEC_COMMIT` に対応する。成功しても最新 SPEC や実機の適合ではない。検査器の範囲と残件は Python client の `docs/conformance-checks.ja.md` を参照する。


raw USB検査器には、公開Pythonの `core-v1` ソフトウェアUSBモデルを使う入口もある。unitは `virtual-core-1`、SPEC/新規JSON/既存lockを明示する。ADDRESS/FRAMING/TCP_PEERを使わず、USB device/gadgetやOSのUSB stackを検査したとは扱わない。

```sh
uv run --project tests --with-editable /absolute/path/to/oep-client-python \
  --env-file tests/.env python -m oep_client.conformance_usb_model --kind bulk
# HIDは別の新規出力先を使う
uv run --project tests --with-editable /absolute/path/to/oep-client-python \
  --env-file tests/.env python -m oep_client.conformance_usb_model --kind hid \
  --out /absolute/path/to/new-hid.json
# 検査器・モデルの必須回帰（実機を使わない）
uv run --project tests --with-editable /absolute/path/to/oep-client-python \
  pytest -c tests/pyproject.toml /absolute/path/to/oep-client-python/tests/test_conformance_usb.py
```

bulk 55項目、ID付きHID 60項目。実機raw adapter・descriptor・packet終端・物理USBの検査は残件。既存HID runtimeの旧仕様回帰はそのまま残し、新しい検査器に合わせて期待値を書き換えない。詳しいadapter契約と検査範囲は公開Pythonの `docs/usb-conformance.ja.md` を参照。

資源寿命の入口は、公開Pythonの `docs/resource-conformance.ja.md` にある明示adapter APIを使う。作成/解放/状態観測を提供するinterfaceについて14資源契約を確認する。最小in-processサンプルは2 fn / 2種類の資源を持ち、宣言検査を含め16項目。通常のcore入口へ自動追加せず、OEP標準interfaceやArduino firmwareへの適合証拠にも置き換えない。

```sh
uv run --project tests --with-editable /absolute/path/to/oep-client-python \
  pytest -c tests/pyproject.toml /absolute/path/to/oep-client-python/tests/test_conformance_resources.py
```

これは実機を使わない回帰。実時計の最小実行例は公開ガイドに従い、既存 `.env` のSPEC/共有lock pathを明示して使う。新しい設備項目や暗黙のベンチ依存は追加しない。購読・通知・経路切断・解放順・電気的idleは続く検査範囲として残す。

購読・出来事の寿命も公開Pythonの明示adapter APIを使う（`docs/subscription-conformance.ja.md`）。最小モデル `virtual-notify-1` では18項目。外部刺激とraw通知を記録し、購読の置換/解除/再送、session終了時の停止を確認する。data、応答優先、route、queue上限、実機の非同期処理はこの成功に含めない。

```sh
uv run --project tests --with-editable /absolute/path/to/oep-client-python \
  pytest -c tests/pyproject.toml /absolute/path/to/oep-client-python/tests/test_conformance_subscriptions.py
```

実時計APIは既存`.env`のSPEC/共有lock pathを明示して使い、新規JSONへ保存する。このモデルは実機やADDRESS/FRAMING/TCP_PEERを使わず、私的ベンチ依存を加えない。

データ通知には公開Pythonの `docs/data-conformance.ja.md` の明示adapterを使う。サンプル `virtual-stream-1` は購読を含め33項目。外部のbyte位置、raw通知、min_bytes/max_delay、data/event共通seqを検査する。`loss_free=True`は刺激量に対する設備条件で、SPECが認めるdropを禁止する規範変更ではない。seq一周の単体65,538通知は仮想時計・in-processの証拠として扱う。

```sh
uv run --project tests --with-editable /absolute/path/to/oep-client-python \
  pytest -c tests/pyproject.toml /absolute/path/to/oep-client-python/tests/test_conformance_data.py
```

実時計APIは既存`.env`のSPEC/共有lockを明示して使い、別の新規JSONへ保存する。データモデルはADDRESS/FRAMING/TCP_PEERを使わない。実機の応答順序・通知経路・queue上限・drop・物理非同期処理は残件。

通知経路・writerは公開Pythonの `docs/route-conformance.ja.md` のinstrumented adapterを使う。論理モデル `virtual-routes-1` で9項目。部分送信の残りbytes、応答優先、dropのseq、経路切断後の保持を確認する。実機のframing/OS buffer、経路別window/max_inflight、並行処理はこの成功に含めない。

```sh
uv run --project tests --with-editable /absolute/path/to/oep-client-python \
  pytest -c tests/pyproject.toml /absolute/path/to/oep-client-python/tests/test_conformance_routes.py
```

実時計APIは既存`.env`のSPEC/共有lockを使い、新規JSONへ保存する。私的ベンチ依存や設備探索を加えない。

経路別のwindow/max_inflight受付・回復は[公開pipelineガイド](../../../oep-client-python/docs/pipeline-conformance.ja.md)に分ける。明示したreject-overflow論理モデルの補助検査で、実機の超過拒否を必須にしない。Arduinoの同じuv/pytest入口で実行する。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_pipeline.py
```

SPEC/共有lockは既存`.env.example`を使い、追加の設備設定や私的ベンチ依存を加えない。

混雑時のsession再送・拒否cacheは[公開pressureガイド](../../../oep-client-python/docs/replay-pressure-conformance.ja.md)を使う。同じ明示pipelineモデルに対する検査で、Sを別接続へ移さない。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_replay_pressure.py /path/to/oep-client-python/tests/test_conformance_pipeline.py
```

応答送信とleaseは[公開leaseガイド](../../../oep-client-python/docs/lease-conformance.ja.md)を参照する。実時計の論理writer補助検査と、長いop・切断・同session openの仮想時計単体を分ける。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_lease.py /path/to/oep-client-python/tests/test_conformance_replay_pressure.py /path/to/oep-client-python/tests/test_conformance_pipeline.py
```

再送cacheの保持上限は[公開retentionガイド](../../../oep-client-python/docs/retention-conformance.ja.md)の明示sampleで検査する。16 byte / 8件はfixture条件で、実機の最小保持サイズを定めない。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_retention.py /path/to/oep-client-python/tests/test_conformance_lease.py /path/to/oep-client-python/tests/test_conformance_replay_pressure.py /path/to/oep-client-python/tests/test_conformance_pipeline.py
```

保持欠落とsession/boot寿命は[公開lifecycleガイド](../../../oep-client-python/docs/retention-lifecycle-conformance.ja.md)を使う。論理modelのresetを明示し、実機の再起動/転送へ接続しない。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_retention_lifecycle.py /path/to/oep-client-python/tests/test_conformance_retention.py /path/to/oep-client-python/tests/test_conformance_lease.py /path/to/oep-client-python/tests/test_conformance_replay_pressure.py /path/to/oep-client-python/tests/test_conformance_pipeline.py
```

corr u16の半周・上限とend用番号の確保は[公開corrガイド](../../../oep-client-python/docs/corr-conformance.ja.md)の明示sampleで検査する。Arduino firmwareの適合結果には含めない。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_corr.py
```
