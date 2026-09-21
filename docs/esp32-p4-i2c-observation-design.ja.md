# ESP32-P4 開発用プローブ: I2C peer と波形観測の設計

この文書は、ESP32-P4 を使う開発用プローブで DUT の I2C を検証する際の、
peer（相手機器）と受動観測を分けた設計である。対象は CH32X035 を最初の
実証対象とするが、MCU 固有の API には依存しない。

## 現状と目的

X035 の `Wire` は route 2 で `PC16=SCL`、`PC17=SDA` を選べる。治具では
それぞれ P4 の GPIO52、GPIO50 に接続されている。GPIO52 を Low にした時に
DUT の `PC16` 読み値も Low になり、配線、ピンルート、およびバス・ビジー時の
DUT エラー経路までは確認済みである。

一方、P4 のアドレス `0x42` に対する DUT の送信は現在 NACK となっている。
ソフトウェアのログだけでは「P4 が slave を開始できていない」「START は届くが
アドレスを認識していない」「電気的な立上りが遅い」を区別できない。次段階では、
バスに影響を与えず波形と ACK を採取できる必要がある。

2026-09-21 時点では P4 の `Wire.begin(0x42, 50, 52, 100000)` は成功し、OEP の
`FixtureI2c.getStatus()` でも peer started と両線 High を返す。しかし X035 診断
イメージの address write 後も P4 の receive/request callback 回数は 0 のままであり、
DUT は `endTransmission()` で status `2`（address NACK）を返した。

2026-09-21にgeneric configuration leaseからESP-IDF の新しい`i2c_new_slave_device()` driverを
I2C1/GPIO50/52へ開始し、F8U6 X035のroute 2（PC16/PC17）から4-byte writeを実施した。10 kHzと
1 kHzの両方でP4 receive callbackは増えたが、X035の`endTransmission()`はstatus 2（address NACK）を
返した。clock stretchの有無も結果を変えなかった。従って「callback発生=ACK成功」ではない。配線と
master出力が到達していることまでは示すが、P4が9 bit目をLowへ駆動していない原因は未確定である。
Arduino-ESP32 3.3.11 の`Wire` slaveはI2C0/I2C1ともcallback 0回で、こちらも採用しない。

最初の IDF 試験では callback 内で `i2c_slave_receive()` を再 arm して P4 の interrupt
watchdog を発生させた。receive job の再 arm は ISR で行わず task 側で行う必要がある。
現在の direct-driver target は一回だけ受信する安全な診断実装であり、継続運用にはまだ
しない。RMT trace はHAL回避後も9 bit目の実際のlevel・ACK・timingを記録する最優先実装である。

## 3 つの役割を混ぜない

| 役割 | 実装 | 主用途 | 速度・保証 |
| --- | --- | --- | --- |
| hardware peer | P4 の hardware I2C target | 正常系の read/write、repeated START、clock stretching | 標準/fast mode を実装と電気仕様の範囲で試験する本命 |
| passive observer | SCL/SDA 各 1 本の RMT RX | START/STOP、各 bit、ACK/NACK、周期、glitch、stuck bus の記録 | バスを駆動しない。判定の根拠を残せる |
| software peer | GPIO ISR + open-drain | hardware target の代替、意図的な ACK/NACK/異常応答 | 初期は 10 kHz、計測後に 25 kHz まで。100 kHz を保証しない |

同時に有効にする場合も、**駆動する peer は一つだけ**とする。observer は常時併用
できる。P4 には RMT RX 候補が 4 本あり、SCL と SDA を別チャネルで同時受信できる。

P4 では Arduino `Wire` slave を hardware peer として採用しない。正式実装は ESP-IDF
I2C slave driver を直接使い、ISR は event 通知だけ、receive queue の回収・次 job の arm・
TX response の投入は high-priority task が行う構成とする。

## RMT observer

### 記録方法

- SCL と SDA をそれぞれ 10 MHz（100 ns/tick）で RMT RX に接続する。
- 両 RX を arm してから、同一のプローブ時刻と設定を trace header に記録する。
- 各 RX の level/duration 列を生データとして保持し、後段で SCL の立上り時点の
  SDA を復元する。
- 復元した event 列は `START`、`STOP`、`byte(0..255)`、`ACK/NACK`、
  `clock-stretch`、`glitch`、`overflow` を含む。

RMT はチャネルごとのエッジ列を取得する装置であり、二本の入力を一個の原子的な
サンプルとして読むものではない。このため trace には各チャネルの開始時刻、tick、
overflow、最初の level を必ず添える。100 kHz I2C の bit 幅は約 10 us なので、
100 ns の分解能はデバッグには十分であるが、絶対的な setup/hold 認証には外部
ロジックアナライザを用いる。

Caps/Configure revision 2は実装済みで、group roleをstable wire IDで返す。`capture` groupは
`clock=1`と`data=2`を使用し、host manifestが選んだ二つのprobe channelを受動RMTへ割り当てる。
このobserverをP4 firmwareの固定GPIO50/52機能として追加してはならない。

### lease と trace の確定 contract

RMT observerは単独の`capture` peripheral groupとしてleaseする。`clock`/`data`はともに
`capture` functionであり、どちらもinput-onlyとして構成する。次の不変条件を実装・試験する。

1. reservation済みpin、同一pinの二重指定、同一lease内のUART/I2C roleと重なるpinはrejectする。
2. `apply`成功時だけRMT RX channelを二本作成・enable・armする。`release`、watchdog、失敗途中では
   channelをdisable/deleteし、GPIOをfloating inputへ戻す。
3. `FixtureCapture`はlease中だけ操作できる。既存`FixtureGpio.configure`がcapture pinを変更することも
   rejectする。これは観測そのものを変化させないための必須排他である。
4. 一回のcaptureは同一host monotonic timestamp、resolution、両入力の開始levelをheaderに入れる。
   RMT callbackは固定長ringへcopyするだけにし、decode・再arm・USB送信はtask/`loop()`側で行う。
5. `readTrace(offset,length)`はraw RMT symbol（level/duration）とoverflow/partialを返す。I2C eventへの
   decodeはhostでまず実装し、probe内decoderはraw traceとの一致試験を通してから追加する。

ESP-IDF 5系のP4 driverでは`rmt_new_rx_channel()`、`rmt_enable()`、非同期`rmt_receive()`、
`rmt_rx_register_event_callbacks()`を用いる。`signal_range_max_ns`をcapture終端として使うが、SCL/SDAの
callback完了時刻は一致しない。このため「同時受信」は同一arm設定による二本の独立traceであり、完全な
同期サンプルを主張しない。

2026-09-21にP4（`30:ed:a0:e3:11:08`）でこの最小実装を実機確認した。GPIO52を`clock`、
GPIO50を`data`としてleaseし、10 MHz RMT RXを二本armした後に、両入力idle High、raw symbol read、
releaseを確認した。P4 RMT hardware blockは最小64 symbolsであり、host公開用の最初の16 recordsとは
別に64をdriverへ指定する必要があった。I2C transactionを発生させるHIL trace/ACK decodeは次のgateであり、
この最初の結果はidle状態とlifecycleだけを示す。

同日にX035F8U6 peerを software reset してUART `READY`を確認後、`startCapture → RUN`の順で
測定した。SCLは10 records、SDAは4 recordsを返し、UARTのDUT判定は従来どおり`peer_address_ack FAIL 2`
だった。capture leaseとUART leaseを同時に取得しても、RMT側は受動のままI2C transactionを記録できた。
raw recordsのI2C decodeと9 bit目の確定は未実装であり、この時点でACK levelを断定してはならない。

その後、host側のMCU非依存decoderで同じraw recordsを展開した。SCL high中央の9 sampleから
`address=0x42`、write、`ack=false`を復元し、DUTの`endTransmission()` status 2 と一致した。
したがって、現行P4 I2C targetのcallback回数とは独立に、9 bit目で実際にACKが成立していないことが
確認できた。trace開始は二本のRMT channel間で完全同期ではないため、これは1 kHz fixtureの判定であり、
100 kHz以上のtiming認証には使わない。

### まず確認できること

`0x42` への一回の書込みで、以下を一つの取得結果として返す。

1. idle 時に SCL/SDA がともに High か。
2. DUT が START を発生したか。
3. 送信された 7-bit address と R/W bit は何か。
4. 9 bit 目に SDA が Low（ACK）か High（NACK）か。
5. SCL high/low、SDA setup、立上り時間の概算、および途中でバスが止まったか。

これにより現行の NACK は、peer 初期化、アドレス不一致、配線/プルアップ、DUT
側の送信順序のいずれかに絞り込める。

### OEP 公開 API 案

`FixtureI2c` は GPIO 操作 API と別の capability とし、少なくとも以下を持つ。

| 操作 | 内容 |
| --- | --- |
| `configureObserver(scl, sda, resolution_hz, buffer)` | 受動 RMT 観測を設定する |
| `startCapture()` / `stopCapture()` | 一回の試験境界を明示する |
| `getStatus()` | idle level、overflow、capture state、最後のデコード結果を返す |
| `readTrace(offset, length)` | 生の duration 列または正規化 event 列を分割取得する |
| `configureTarget(...)` | hardware / software peer を選び、アドレス、周波数、応答 script を設定する |

probe は capability 表で `i2c-observer-rmt`、`i2c-target-hardware`、
`i2c-target-software` と各々を宣言する。利用側は「I2C がある」だけで性能を仮定
してはならず、要求した mode と最大検証周波数を照合する。

## software I2C target の扱い

software target は GPIO を **open-drain のみ**で使う。Low は能動駆動、High は
入力へ戻して外部プルアップに任せる。push-pull High は I2C では禁止する。

- START/STOP は `SDA` の変化と `SCL=High` で検出する。
- bit は SCL rising ISR で読む。ACK は 8 bit 目の後の SCL Low 中に SDA を Low にする。
- 実際に deadline を守れるかは ISR の最大遅延を trace と cycle counter で測る。
- 応答処理を OEP/USB のコールバックや動的メモリ確保に置かない。あらかじめ固定長の
  response script を用意する。
- 必要なら slave 側が SCL を Low に保持して clock stretch する。ただし DUT master が
  stretch を許容することも別に試験する。

これは仕様外動作の再現や hardware target の障害切り分けに有用である。しかし
FreeRTOS/USB 割込みの遅延を完全には排除できないため、初期の公称上限を 10 kHz と
する。10 kHz で長時間・worst-case 負荷を通してから 25 kHz を capability として
追加する。100/400 kHz は hardware I2C target と受動 trace で試験する領域であり、
software target の互換性を主張しない。

## 電気条件

P4 の内部 pull-up/down は接続確認には使えるが、I2C の 1.65 V 印加や rise-time
評価の基準にはならない。治具/専用プローブは次を宣言可能にする。

- SDA/SCL ごとの外部 pull-up の値と電圧（3.3 V を基本、切断可）。
- pull-down、open-drain drive、完全 release、レベル読取り。
- バス接続/切断、過電圧防止、必要ならレベル変換。
- 観測専用入力。これを peer 駆動系と電気的に切り離せること。

これにより、idle Low、NACK、clock stretching、プルアップ不良を「ソフトの成否」
から分離して記録できる。

## SPI/PWM への横展開

RMT RX は SPI でも `CS/SCK/MOSI` の受動記録に使え、command/address と周期・duty
を確認できる。ただし複数線の完全同期と高速 MISO 応答は RMT のみでは保証しない。
SPI slave は hardware peripheral を優先し、observer は CS を試験境界にする。

PWM は 1 本の RMT RX で high/low duration、周波数、duty、jitter、停止状態を数値化
できる。ADC/アナログ波形、sub-100 ns の timing 認証、多線高速 SPI は外部 ADC/比較器/
ロジックアナライザを capability として追加するまで対象外である。

## 実装順序と合格条件

1. `capture` groupのlease、RMT二入力、raw trace取得、GPIO排他を実装し、現行 `0x42` NACK の trace を採取する。
2. P4 hardware I2C target の開始結果、アドレス一致、RX/TX callback 回数を
   `getStatus()` で公開する。
3. 10 kHz と 100 kHz で write/read、repeated START、NACK、stuck bus recovery を
   trace 付きで HIL 試験する。
4. その後に software target を 10 kHz 専用 capability として導入し、ACK deadline と
   長時間安定性を測定してから上限を決める。

リリース試験の合否は通信 API の戻り値だけでなく、正常/異常ケースそれぞれの trace
と capability 宣言で再現可能にする。
