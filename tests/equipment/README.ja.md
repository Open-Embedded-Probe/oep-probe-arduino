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

result_lost後の回復は[公開recoveryガイド](../../../oep-client-python/docs/recovery-conformance.ja.md)の明示sampleを使う。応答を論理的に失わせ、wireの読み直しで判断する。実機timeoutやproduction Host統合の適合結果には含めない。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_recovery.py
```

回復中のforceと複数未解決要求は[公開recovery-racesガイド](../../../oep-client-python/docs/recovery-races-conformance.ja.md)の明示sampleで検査する。経路closeとreader世代は論理fixtureの動作で、物理USB/TCP回復の検査とは別に報告する。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_recovery_races.py
```

corr枯渇前の切替は[公開rolloverガイド](../../../oep-client-python/docs/rollover-conformance.ja.md)の明示sampleで検査する。production Hostや物理readerの切替を検査した結果には含めない。

```bash
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_rollover.py
```

end/openの応答喪失後の回復は[公開session-lossガイド](../../../oep-client-python/docs/session-loss-conformance.ja.md)の明示sampleで検査する。保存されたopenの成功と現在のsessionの有効性を分け、result_lostからoutcomeやlease値を推測しない。物理再接続やproduction Hostへの統合は未検査。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_session_loss.py
```

番号枯渇時の複数pending放棄は[公開exhaustedガイド](../../../oep-client-python/docs/exhausted-conformance.ja.md)の3条件で検査する。旧Sを新経路へ移さず、解放を確認して別IDでopenし、旧操作のunknownを保持する。物理再接続やproduction Host統合は未検査。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_exhausted.py
```

全fnの共通宣言は[公開interfaceガイド](../../../oep-client-python/docs/interface-conformance.ja.md)で検査する。独自拡張にも同じ8項目を適用し、OEP固有opやcommon部品の動作と分けて報告する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_interfaces.py
```

common §1を採用するstreamのread/marksは[公開streamガイド](../../../oep-client-python/docs/stream-conformance.ja.md)の明示logical fixtureで検査する。採用・addressing・op対応を指定し、captureや資源型streamの動作まで通ったとは扱わない。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_stream.py
```

common writeのacceptedとdebug status/doneは[公開outcomeガイド](../../../oep-client-python/docs/outcome-conformance.ja.md)の明示logical fixtureで検査する。partial/failedとcache再送を確認し、物理配送・target進捗・標準debug opの適合とは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_outcomes.py
```

資源型streamとdebug connectionの寿命は[公開dependencyガイド](../../../oep-client-python/docs/dependency-conformance.ja.md)の明示logical fixtureで検査する。依存順の解放・個別資源指定・historical replayを確認し、標準mechanism制約と物理pin idleの適合は別に報告する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_dependencies.py
```

標準wire／console固有の宣言は[公開OEP宣言ガイド](../../../oep-client-python/docs/oep-declaration-conformance.ja.md)で検査する。第三段階`oep-interface`として報告し、共通宣言・共通部品・op動作・実機適合と分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_oep_declarations.py
```

標準attachのcritical TLVとconsole openの制約は[公開attachガイド](../../../oep-client-python/docs/attach-conformance.ja.md)の明示logical fixtureで検査する。副作用前の拒否と再送を確認し、電気的bringup/配送の実機適合とは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_attach.py
```

標準wireの任意scanは[公開scanガイド](../../../oep-client-python/docs/scan-conformance.ja.md)の
明示logical fixtureで検査する。全候補の事前検証、有限prefix、pin解放、再送を確認する。
電気的探索や配線確定の実機適合は別に報告する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_scan.py
```

標準consoleのSDI/DMDATAは[公開mailboxガイド](../../../oep-client-python/docs/mailbox-conformance.ja.md)の
明示logical fixtureで検査する。DMIアクセス順とqueueを確認し、実target配送とは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_mailbox.py
```

probe側dmseq framingは[公開dmseqガイド](../../../oep-client-python/docs/dmseq-conformance.ja.md)の
明示logical fixtureで検査する。CRC・通番・pendingとreset時の状態を確認し、target側timeoutや実配送とは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_dmseq.py
```

target側dmseqは[公開targetガイド](../../../oep-client-python/docs/dmseq-target-conformance.ja.md)の
論理時計で検査する。20/1000 ms境界は論理モデルの結果で、実時間適合とは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_dmseq_target.py
```

必須riscv-dmとconsole調停は[公開debugガイド](../../../oep-client-python/docs/debug-conformance.ja.md)の
明示logical fixtureで検査する。optional操作の状態復元・物理DMI・応答writerは別に確認する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_debug.py
```

optional blockと状態復元は[公開blockガイド](../../../oep-client-python/docs/block-conformance.ja.md)の
明示logical fixtureで検査する。busのprefixとscratch/mailbox保持を確認し、物理抽象コマンドとは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_blocks.py
```

optional stepと再停止は[公開stepガイド](../../../oep-client-python/docs/step-conformance.ja.md)の
明示logical fixtureで検査する。PC/状態復元とstep_leftを確認し、物理命令・CSR・待ち時間は別に検査する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_step.py
```

optional resetとDM復帰は[公開resetガイド](../../../oep-client-python/docs/reset-conformance.ja.md)の
明示logical fixtureで検査する。mode/flags・DM待ち・mark/再同期とconnection保持を確認し、物理リセットとは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_reset.py
```

optional runは[公開runガイド](../../../oep-client-python/docs/run-conformance.ja.md)の
明示logical fixtureで検査する。停止結果・経過時間・host変更とprogram効果の保持を確認し、CPU実行や実clockとは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_run.py
```

wireのattach reset/idle_clock/targetselは[公開wireガイド](../../../oep-client-python/docs/wire-options-conformance.ja.md)の
明示logical fixtureで検査する。ピン競合と解放・接続同一性を確認し、電気的open-drain/時間/波形は別に検査する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_wire_options.py
```

scanのidle_clock/targetselは[公開scanガイド](../../../oep-client-python/docs/scan-options-conformance.ja.md)の
明示logical fixtureで検査する。指定selectorと有限候補、探索中のlevelsと解放順を確認し、実配線探索とは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_scan_options.py
```

線の再試行は[公開retryガイド](../../../oep-client-python/docs/retry-conformance.ja.md)の
明示logical fixtureで検査する。host DMIの曖昧配送は再送せず、busyとprobe自身の読みに分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_retry.py
```

線切れ時の順序は[公開cutガイド](../../../oep-client-python/docs/cut-conformance.ja.md)の
logical writer gateで検査する。応答の後に依存資源を解放し、consoleにはlink-lostを記録する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_cut.py
```

mandatory debug検査にはDMI応答長の境界も含む。64 byte経路で値配列が入らない要求を
副作用前に拒否し、入る要求では補助TLVを省略して値をすべて返す。入口は既存の
`test_conformance_debug.py`で、追加設備設定は不要。

mandatory debugのhartsel検査は、halt/resumeの成功/失敗/既停止とraw DMIを区別する。
高水準opはhart 0へ戻し、resume/failed haltではhaltreqを下ろす。既存のuv入口から実行できる。

writer待ち中のleaseは[公開ガイド](../../../oep-client-python/docs/writer-lease-conformance.ja.md)の
明示logical clockで検査する。実行中停止とwriter待ち、期限切れ後の古い応答を分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_writer_lease.py
```

登録済み標準名の必須opと任意opの組は[公開ガイド](../../../oep-client-python/docs/registered-ops-conformance.ja.md)で
宣言だけを検査する。各opの実動作や固有describe tagの適合とは分ける。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_registered_ops.py
```

## 標準describe metadataの論理検査

設備なしでrestart/config/RISC-V/ARM-ADIの宣言条件を検査する。
対象の公開clientを明示し、私的ベンチへの依存や設備の暗黙検索を追加しない。
広告opの実動作や実機適合を証明する検査ではない。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_selected_metadata.py
```

## fixture系metadataの論理検査

GPIO/UART/I2C/SPIの選択した宣言条件は設備なしで検査できる。
広告opの動作、配線と電気的適合の証拠にはしない。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_fixture_metadata.py
```

## capture/config metadataの論理検査

選択したcapture/configの宣言は設備なしで検査できる。広告opの動作と物理適合は別に検証する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_capture_metadata.py
```

## GPIO/planの論理動作検査

明示した仮想fixtureでset/read、planの原子置換、駆動順序、再送と解放を検査する。
設備なしで動作し、実機のlevel/電流/波形は別に検証する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_gpio.py
```

## GPIO planでのcore寿命検査

明示logical clockで期限切れ/force/再送履歴とpin/planの解放を検査する。
物理pinの時間とwriter並行性は別に検証する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_gpio_lifecycle.py
```

## UARTの論理検査

configure/stream/queue/plan寿命は第三段階、採用した共通streamの境界は第二段階で確認する。
設備なしで明示logical fixtureを使用し、物理baud/波形/配送は別に検証する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_uart.py /path/to/oep-client-python/tests/test_conformance_uart_stream.py
```

## dmseqのmark検査

console §2のresync/TO/CRC markを論理fixtureで確認する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_dmseq.py
```

## I2C targetの論理検査

公開checkerを明示して呼び出し、stretch/内部pullupの有無とFIFO/plan寿命を確認する。
設備なしで使用できるlogical fixture。物理バスの波形/実時間は別に検証する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_i2c.py
```

## SPI targetの論理検査

公開checkerへlogical CS/SCK edgeとarm readinessを明示し、LSB宣言の有無を確認する。
物理波形と負荷下のCS setup実時間は別に検証する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_spi.py
```

## logic captureの論理検査

公開checkerへsample/時計/errorを明示し、one-shot/repeatの世代と区画寿命を確認する。
物理sample clockや不確かさ保証、analog/group/streamingは別に検証する。

```sh
uv run --project tests --with-editable /path/to/oep-client-python pytest -c tests/pyproject.toml /path/to/oep-client-python/tests/test_conformance_logic.py
```
