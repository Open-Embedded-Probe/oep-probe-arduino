# 開発用プローブ / X035 検証: 現在地と次の作業（2026-09-22）

この文書は、実験の成功、OEP firmwareで公開済みの機能、今後のrelease gateを分けて
記録する棚卸しである。一つのstandalone sketchの成功を、汎用probeの公開能力やX035 coreの
release合格として扱わない。詳細なX035 release gateは
[x035-release-worklist.ja.md](x035-release-worklist.ja.md)、個々の実験の一次記録は
別repositoryで管理する`wch-protocols/experiments/LEDGER.ja.md`を正とする。

## 結論

- **P4 I2C slave v1の実用上の境界は把握できた。** P4二台をGPIO32=SDA/GPIO33=SCLで直結した
  peer試験では、固定長write、二transactionのlength-framed write、preload済みreadを全byte照合で
  確認した。1 MHzで128 byte×100回のpreload readは10回連続で成功し、平均199,828 us
  （payload約64 kB/s）だった。
- **ただし、現行OEP imageはhardware I2C peerを公開していない。** X035 prototypeが選択するのは
  write-only・10 kHz以下のsoftware targetである。ESP-IDF direct backendは4 byte固定・一回受信の
  診断実装であり、可変長・動的read・連続readをOEP capabilityとして主張しない。
- **X035のI2C NACKは未解決である。** X035 route 2からP4へ出した4 byte writeはP4側callbackまで
  到達したが、X035 masterはaddress NACKを報告した。P4-P4成功はdriverの基準にはなるが、この
  電気的/時刻的なX035問題の解決を意味しない。
- **従ってrelease段階ではない。** 書込み/RVSWDとUARTの基礎は進んだが、常駐probeのI2C契約、
  observer、HIL runner、X035の各peripheral証跡が残っている。

## 実証済みの事実

| 範囲 | 結果 | 根拠・制約 |
| --- | --- | --- |
| P4 I2C v1 fixed write | 1〜400 kHzの4 byte、100 kHzの1〜128 byte、4 byte×100を成功 | receive jobの長さは一つのwrite transactionと一致が必要。任意長bufferではない（E147） |
| P4 I2C v1 framed write | length headerとpayloadを別transactionにし、1 MHz・128 byte×1000を成功 | callback内で再armしない。host/probe側taskが次のjobをarmする（E148） |
| P4 I2C v1 preloaded read | 1 kHz〜1 MHz、4/16/128 byteを全byte一致 | v1に`on_request`はない。read前にTX ringをpreloadする（E149） |
| P4 I2C v1 burst read | 128 byte×100、1 MHz、10/10回全byte一致 | read slotごとにpayload後のfiller 1 byteが必要（E150） |
| OEP P4 build | Arduino-ESP32 3.3.12、`sketch.yaml` profile、`--clean`で成功 | `platform_index_url`をprofileへ明記。P4専用direct driverは非P4 buildから隔離済み |
| OEP non-P4 build | 無印ESP32 V003 prototypeも同じ3.3.12 profileで成功 | P4固有IDF型をライブラリ全体へ漏らさない |
| X035書込み/RVSWD | identity preflight、program、full verify、abort後のnormalizationを確認 | 詳細な反復条件・未完了gateはrelease worklistのP0を参照 |

## 公開能力の現状

| 能力 | 実装状態 | 公開上の扱い |
| --- | --- | --- |
| GPIO / UART / probe discovery / lease | 一部実装・実機確認済み | revision 2 capability/leaseの既存範囲で使用可能。ただし各HILケースは未完了 |
| I2C software target | X035 prototypeに実装済み | write-only、10 kHz以下の障害切り分け用。速度・read・clock stretch互換を主張しない |
| I2C direct hardware target | driver基礎とP4-P4実証済み | **未公開**。現OEP protocolにはframe長、framing方式、TX slotの指定がない |
| I2C RMT observer | lifecycle/raw captureの基礎あり | **未公開**。X035 transactionのtrace、ACK decode、overflow/cleanupをHILで固める必要がある |
| SPI peer / observer、PWM observer、ADC source | 設計・部分的な基礎試験 | **未公開**。peripheralごとの明示capabilityと測定契約が必要 |

`ProbeCapabilities`はprobe MCUの候補pinとresource groupを返すだけで、接続先DUTのpin mapを
返さない。hostは接続後にmanifestを作り、roleへprobe channelを割り当て、leaseを取得する。
これはP4、ESP32、RP2040などでprobe firmwareを固定し、対象boardだけを差し替えるための前提である。

## I2C hardware targetを公開するための契約

revision 2のcompact capabilityにはI2Cの転送方式・上限を表す欄がない。このまま
`ProbeGroupI2cTarget`だけを返すと、hostが可変長や動的応答を期待してしまう。次のrevisionで
modeを別能力として宣言してから実装する。

| mode | hostが明示する設定 | probeが保証する範囲 | HIL合格条件 |
| --- | --- | --- | --- |
| `fixed-rx` | address、SDA/SCL role、正確なframe bytes、最大clock | 一つの固定長writeを受ける | 1/10/100/400 kHz、各frame長、NACK/timeout/release |
| `framed-rx` | header形式、最大payload、header/payload timeout | header transaction後に同長payloadを受ける | 0 gapを含む短期反復、誤length/中断/再arm安全性 |
| `preloaded-tx` | slot payload長、slot数、filler規則、最大clock | hostが投入済みの固定responseを返す | full pattern、各read境界、master NACK、ring不足 |
| `software-target` | address、最大10 kHz、write-only script | open-drain ACK診断 | worst-case ISR jitter、長時間、release後High、NACK trace |
| `observer-rmt` | SCL/SDA role、resolution、buffer | raw symbolとoverflow/partial情報 | ACK decode、START/STOP、同時peer利用、cleanup |

この宣言はMCU固有のI2C番号やGPIO番号を含めない。P4/ESP32/RP2040は、同じmodeへ自分の
候補pin、上限frame/slot、最大速度、排他resourceを報告する。

## 次の作業（依存順）

### A. capability contractと回帰基盤

- [ ] revision 3のI2C mode capabilityとconfigure payloadを仕様化する。未知mode、上限超過、
  role不足、競合leaseは副作用なしでrejectする。
- [ ] `fixed-rx`、`framed-rx`、`preloaded-tx`をdirect driverのtask-backed backendへ実装する。
  ISR callbackは通知だけに限定し、receive再arm/TX投入/USB送信をISRで行わない。
- [ ] P4-P4のE147〜E150をOEP frame経由のHIL smoke testへ移植する。結果にはmode、clock、
  frame/slot数、全byte照合、timeout、driver error、probe revisionを残す。
- [ ] configure/enable/status/disable、host disconnect、watchdog、失敗途中の全経路でpinが
  input/releaseへ戻ることを自動試験する。

### B. X035 I2C NACKの原因特定

- [ ] RMT observerをSCL/SDA二本で同時armし、X035 route 2のaddress/ACK/STOPをraw traceと
  decode結果で保存する。callback回数をACKの根拠にしない。
- [ ] external pull-up、P4 internal pull、clock 1/10/100 kHz、software/direct peerを一条件ずつ
  比較する。電圧、配線、rise time、9 bit目を同一artifactに残す。
- [ ] hardware peerがACKしていることをtraceで示した後に、X035のwrite/read、repeated START、
  clock stretch、stuck-bus recoveryへ進む。

### C. 常駐probeとしての完成度

- [ ] P0書込み速度の内訳telemetry、physical page commitの変更page反復/failure injection、
  probe電源断を含むrecoveryを完了する。
- [ ] HIL runnerを作り、target image hash、probe firmware revision、manifest、電圧、周波数、
  trace、cleanup結果を一つのartifactとして残す。
- [ ] GPIO、UART、ADC、PWM/timer、SPI、reset/bootを、必要なpeer/observer capabilityの順に
  実装・検証する。ADC中点はP4内部pullではなく校正済み外部sourceを使う。
- [ ] 利用者ガイドに「通常は`loop()`/`millis()`で自前の時間管理を推奨し、hardware timerは
  必要な場合だけ」と記載する。EEPROMと`HardwareTimer`移植は現release gateに含めない。

## 判定基準

次のいずれも満たすまで、P4/X035 probeをrelease-readyと呼ばない。

1. 書込み、verify、abort/recoveryのreliability gateが完了している。
2. capability宣言と実装・HIL結果が一致し、未実証modeを返さない。
3. X035 I2C NACKをtrace付きで解決、または電気/board制約として再現条件と回避策を確定している。
4. P3のGPIO/UART/ADC/PWM/timer/I2C/SPI/resetを、常駐probe firmwareだけで再現可能にしている。
5. 利用者向けガイドに速度、電圧、精度、resource競合、既知の制限を反映している。
