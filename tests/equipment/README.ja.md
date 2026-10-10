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

core は case ごとの pytest/JUnit 結果、interface は全 fn の共通宣言検査、OEPインターフェース固有の動作は未実行。JSON には送受信 bytes、SPEC commit/dirty/hash、検査器 hash、実装版、未検査範囲が残る。全体適合を名乗らない。

全く設定していない任意入口のみ SKIP。部分設定、接続・権限・個体の誤り、旧仕様と新仕様の不一致は FAIL。transport/cleanup失敗の後の項目は BLOCKED となり、実行全体は FAIL。

既存 portable/C++ の期待値は `tests/vectors/SPEC_COMMIT` に対応する。成功しても最新 SPEC や実機の適合ではない。検査器の範囲と残件は Python client の `docs/conformance-checks.ja.md` を参照する。
