# CH32X035 release 検証へ戻るための作業リスト

対象は ESP32-P4 / CH32X035C8T6 fixture と OEP prototype である。ここでいう完了は
「特定の診断 sketch が一度動く」ではなく、同じ常駐 probe firmware と公開された OEP
capability だけを使い、再現可能な HIL 試験を実行できる状態を指す。

2026-09-22時点の実測済み範囲、公開可否、依存順の要約は
[progress-and-next-work-2026-09-22.ja.md](progress-and-next-work-2026-09-22.ja.md)を参照する。
この文書はrelease gateごとの詳細証跡と未完了条件を維持する。

進行順と優先度は[rebuild-plan-2026-09-22.ja.md](rebuild-plan-2026-09-22.ja.md)を正とする。

## 原則

1. **書込み・退避・verify が全試験の前提**である。周辺機能試験を先行しない。
2. probe firmware は試験ごとに peer を焼き替えない。一度起動した firmware が、board
   profile と capability を宣言し、host が明示的に構成・開始・停止・結果取得する。
3. capability は「使えるか」だけでなく、利用可能な P4 pin 組合せ、電気条件、最大速度、
   同時利用不可な資源、測定精度を返す。
4. target 書込み経路（RVSWD）と fixture peer/observer 経路は別の resource lease として
   扱う。SWD GPIO2/54、P4 USB GPIO24--27、input-only GPIO46 は一般 GPIO/PWM/I2C の
   構成候補から除外する。
5. 全ての状態変更には `disable` / `release` と status/trace があり、失敗後も安全な
   input/release 状態へ戻せることを確認する。

## P0: 書込み経路の安定化と高速化（最優先）

### P0.1 単一接続と状態回復

- [x] client が書込み、read、verify、target reset の全期間で board-identify 固定名
  `/run/board-identify/by-id/esp32-series-30eda0e31108` を排他的に保持する。並行 open を
  明示的に拒否する。
- [x] transaction を `attach → halt → 操作 → verify → reset/release` として記録し、
  timeout・プロトコル破損・P4 reset の各ケースで target を通常実行へ戻す。
- [ ] 書込み開始前に target identity、flash base/size、protection 状態を読み、X035 以外や
  容量不一致を拒否する。手元の original image は保全用であり、試験ごとに自動 restore
  しない。
- [ ] page 失敗時の再送、recovery cache、再接続後の再開可能範囲を仕様化する。

**合格条件:** 同じ image の full read/verify を連続 20 回、差分 program + full verify を
連続 20 回、途中で host/client を中断した場合を各 5 回実行して、誤書込み・target 未復帰・
未報告の失敗がないこと。

2026-09-21にPWM probe image（full-image SHA-256
`b6f5b99dab244417aee37c7cdc4459f3a7158ce55af63ba22bea9cb7bf1f93c4`）のfull verifyを
20回連続実行した。失敗0、verifyは平均5.166509 s・中央値5.164288 s・p95 5.226030 s・
最大5.249059 s、resetは平均3.376 ms・p95 3.868 ms・最大4.298 msだった。差分program 20回と
host/client中断各5回は未実施のため、このP0.1 gate全体は未完了である。

同日、PWM probe imageとI2C probe imageを交互にして、27 physical page差分のprogram +
full verifyを20回連続実行した。全回`pages=27, attempts=27`、retry 0、hash一致、両reset成功。
programは平均11.888387 s・中央値11.886405 s・p95 11.925864 s・最大11.949021 s、続くverifyは
平均5.161236 s・中央値5.158991 s・p95 5.243148 s・最大5.250146 sだった。残るP0.1の必須項目は
host/client中断試験各5回である。

同日、差分programの7秒後にhost processを強制終了する中断を実施した。旧firmwareではtargetが
halt状態で残ることを確認したため、endpointが最後の有効requestから1.5秒無通信で、かつRVSWD
sessionがactiveなら`normalizeUser()`を発行するwatchdogを追加した。watchdog版P4 firmwareで同じ
中断後に2秒待機し、別clientから残り20 physical pageを更新して全域hash
`b6f5b99dab244417aee37c7cdc4459f3a7158ce55af63ba22bea9cb7bf1f93c4`まで一致させた。これは
中断後のtarget reset/re-attach/recoveryが可能なことを示す初回のHIL evidenceである。続けてPWMと
I2C imageを交互にした同じ試験を計5回実行し、全回timeout exit=124、2秒待機後の別clientのrecovery
exit=0、`pages=20, attempts=20`で完走した。最後にPWM imageをverify-onlyで読み直し、同じfull-image
hashを確認した。従ってhost processを強制終了する中断5回の基準は満たした。一方、probe自身のreset/
電源断でrecovery cacheが失われる場合と、execution中applicationをPWM波形で独立確認する場合は別の
未完了試験である。watchdogの発火はOEP endpointのsession状態で判定しており、PWM波形を同時capture
するobserverはまだないため、実行中applicationの波形による独立証明はP1で追加する。

同一aliasに対するPOSIX advisory leaseも実機で確認した。全域verifyを実行中の一つ目のclientはexit=0で
完走し、その間に起動した二つ目のclientはOEP frameを送る前にexit=2および
`OEP transport is already in use: /run/board-identify/by-id/esp32-series-30eda0e31108`で拒否された。
leaseはaliasの解決先をhashしたlock fileとpyserialのexclusive openを併用している。

identity gateの候補をread-onlyで調べた。初回に参照した`0x1ffff7c4`はV003向けの番地であり、
X035向けではないため`0xffffffff`だった。X03xの正しいESIG chip-id番地`0x1ffff704`は
`01 06 5e 03`（little-endian `0x035e0601`）を返し、これはP4接続先のCH32X035F8U6と一致する。
F8U6は63,488-byte flash/20 KiB RAMである。FLASH `OBR=0x4002201c`は`0x03fffffc`
（read-protection bit1=0）、`WPR=0x40022020`は`0xffffffff`だった。hostの`--program-image`は
このF8U6 ID、base `0x08000000`、size 63,488、read-protection off、WPR全bit解除をread-onlyで
確認してからだけ書込みを開始する。不一致・保護状態・読出し失敗はfail-closedで拒否する。
同一PWM imageへの実機更新でpreflight 6.039 ms、差分0 page、全域verify 5.109 s、hash一致を確認した。
この調査とpreflightはflash/option byteを書き換えていない。別packageや別familyを扱う将来のprofileは、
ESIG ID/geometry/protection ruleを個別に宣言してから追加する。

P4 resetによるrecovery cache喪失も実機で境界を確認した。故障注入版firmwareで先頭256-byte
physical pageをerase直後に失敗させ、P4を通常firmwareへ再書込みしてprobe RAMを失わせた。その後の
target先頭88 byteは全FFで、旧pageをcacheだけから復元できないことを確認した。この状態で部分fragmentを
継続して書くことはせず、既知のPWM完全imageを`--program-image --destructive`で再送した。preflight後
`pages=1, attempts=1`で復旧し、full-image SHA-256
`b6f5b99dab244417aee37c7cdc4459f3a7158ce55af63ba22bea9cb7bf1f93c4`が一致した。従ってP4 reset/
電源断後の正しい復旧単位は「retained recovery cache」ではなく、hostが保有する完全imageである。

F8U6向けCore自己試験は、`CH32_SERIAL_DEFAULT=4`付きのC8T6 binaryをF8U6へ書込んだ初回観測を
合格扱いにしなかった。このbinaryではREADY/RUNを確認できなかったため、F8U6 HIL evidenceには使わない。
続いてArduino CLIのuser/data/download directoryを完全隔離し、`sketch.yaml`の旧package profileを
ビルド入力から外して、working treeの`CH32X035:pnum=CH32X035F8U6`をxPack toolchainでbuildした。
compile commandに`ARDUINO_CH32X035F8U6`と`CH32_SERIAL_DEFAULT=4`があることを確認したF8U6固有の
9,780-byte `core_api`をOEPでprogram（39 physical page、14.808338 s）し、63,488-byte full verify
（5.108294 s）まで成功した。P4 FixtureUart（GPIO12/6、115200）でREADY後に`RUN`を送ると、19項目全てが
PASSし`core_api done failures=0`を受信した。UART route/bridge/F8U6 build profileの基礎確認はこれで
解消した。targetはこのF8U6 `core_api` imageを保持しており、image復帰は行わない。

プロトコル破損経路も実機確認した。validな4-byte code-flash readでRVSWD sessionをactiveにした直後、
CRCに届かない不正COBS frameを送った。endpointはframeを破棄し、最後のvalid requestから2秒待機して
watchdogがnormalizationする。その後の別clientによるPWM image全域verifyはhash一致で完走した。これで
host kill/timeout（5回）、protocol corruption（1回）、P4 firmware再書込み/reset（cache喪失+完全image
復旧）の各経路を、targetをhaltで放置しないことまで確認した。電源を物理遮断する試験はP4再書込みresetと
同じRAM cache消失を持つが、電源電圧/USB再列挙の別条件はP0 hardware-soakで独立に残す。

preflightを含む通常書込みのresult JSONもI2C↔PWMの1往復で確認した。両方向ともF8U6 ID、OBR、WPRを
確認後、63,488 byte比較・27 physical page・27 attempts・63,488 byte全域verifyで成功した。I2C方向は
preflight 9.683 ms、program 11.519804 s、verify 4.913288 s、PWM復帰方向はpreflight 6.152 ms、
program 11.548080 s、verify 4.957561 sだった。最終targetはPWM image hash
`b6f5b99dab244417aee37c7cdc4459f3a7158ce55af63ba22bea9cb7bf1f93c4`である。

現行PWM imageの非破壊backupも実行した。63,488 byteを4.970510 sで読み、reset 3.876 ms後、backupの
SHA-256がPWM imageと同じ`b6f5b99dab244417aee37c7cdc4459f3a7158ce55af63ba22bea9cb7bf1f93c4`であることを
確認した。result JSONは`backup.bytes_read=63488`、`backup_read`、`backup_reset`を含む。

2026-09-22、v0 stack（`examples/Esp32P4X035Probe` + oep-client-python `oep_client.v0`、dedicated GPIO PHY、autoexec
reader/writer、1 KiB frame × 4 KiB window pipelining）で同じ gate を `tests/hil/probe/test_reliability.py` として再取得した。
target は F8U6、対象は flash 末尾 8 KiB を 2 種の random image で交互に書換える。full verify（probe 内 CRC32、host 側 CRC と一致）
20 回: 平均 0.281 s・中央値 0.312 s・p95 0.414 s・最大 0.415 s。32 physical page の差分 program 20 回: 平均 0.289 s・中央値 0.267 s・
p95 0.354 s・最大 0.453 s、失敗 0、続く full verify は平均 0.296 s。host 無応答（6 page 送信後に 2.5 s 沈黙）5 回: 全回 endpoint の
1.5 s watchdog で lease 解放と target reset（DMSTATUS allhalted=0 を確認）、再接続後の `program_image` が残り 26 page を書いて
全域 CRC 一致。旧 prototype（verify 5.17 s、27 page program 11.9 s）比でそれぞれ約 18 倍・約 40 倍。verify 時間の振れ（0.24〜0.63 s）は
attach ごとの half period margin check の選択差で、正否には影響しない。

同日、v0 stack の runner（ArduinoCore-CH32 `tests/manual/oep_smoke/oep_smoke.py`）で `tests/sketches/basic` の 14 sketch を
F8U6 へ順に program（各 0.6〜0.8 s、CRC verify 込み）し、fixture.uart（USART4 PB0/PB1 ↔ P4 GPIO12/6）で READY / PING /
expectations を再生して **14/14 PASS**（core_api 19、pd_selftest 23、wire_selftest 15 項目など、failures=0）。LinkE も probe-rs も
使っていない。これは P3 表の前提となる「常駐 probe firmware だけで Core 自己試験を回せる」ことの初回証拠であり、P3 の各行
（波形・peer・電気条件）の判定はまだ含まない。

### P0.2 速度の計測と改善

- [ ] `attach`、read、erase/program、verify、reset の wall time、転送 byte 数、retry 数を
  machine-readable な result に含める。速さだけでなく再送を含む実効速度を基準にする。
  現時点でclientはoperation全体のwall timeだけを返す。X035のrequest間session reuseと
  post-frame guard 0 µsはfull verify 3/3で確認済みだが、内訳telemetryは未実装である。
- [ ] 現行の 64-byte logical page と target flash の物理 page 境界を再確認し、OEP payload
  上限内の複数 page/burst、連続 read、不要な attach/halt の削減を設計する。
  TargetFlash revision 2 は `stage-page64` を4回受けた後の`commit-page256`で、X035の
  256-byte physical erase pageを一度だけerase/programする。4 fragment未満のcommitは拒否し、
  stageだけではtarget flashを書換えない。現imageと同一の先頭256 byteをstage/commitして
  全域hash一致まで実機確認済み。変更pageを含む反復・failure injection・中断後のstage破棄は未完了。
- [ ] program 前の差分比較、program 後の該当範囲 verify、最後の full image verify を分離し、
  利用者が安全性と時間のトレードオフを明示選択できるようにする。release HIL は full verify
  を必須とする。
- [ ] P4/host の baudrate、flow control、frame size を一項目ずつ変え、データ破損・timeout・
  P4 reboot がない最大値だけを capability として宣言する。

**合格条件:** 現在の正しい full verify を基準値として保存し、20 回試験の p95 で速度改善を
示す。速度改善が reliability gate を一つでも下回る場合は採用しない。

## P1: 常駐 OEP capability firmware

### P1.1 profile と discovery

- [x] `ProbeInfo` は generic profile `P4DV`、firmware revision、予約pin mask、fixture pin maskを返す。
  `ProbeCapabilities` revision 2 はP4自身のchannel、電圧domain、groupをpage取得で返し、group roleも
  ordinal queryで安定wire IDとして返す。revision 1のfunction-only group roleではcapture二本を区別できない
  ため、新規capabilityはrevision 2を必須とする。transport、最大速度、較正値は個別capability追加時に返す。
- [x] capabilityはP4のchannel候補、方向、input-only、open-drain可否、予約理由、相互排他resourceだけを返す。
  target pin/board名/既知配線表を返す`PinMatrix`は廃止する。接続先はhostのConnectionManifestで扱う。
- [x] capability ごとに候補 pin set と相互排他 resource を返す。host は任意 GPIO 番号を
  仮定せず discovery 結果だけで構成する。

### P1.2 共通の lifecycle

- [ ] 全 capability を `configure(config) → enable → getStatus/readResult → disable` に統一する。
- [x] `ProbeConfiguration` revision 2 の`apply`/`release`はUART groupのpin/resource leaseを原子的に
  取得・解放し、すでにlease中または未知のrole/channelは変更なしで拒否する。I2C targetも同一planへ
  含められ、start失敗時はpinをinputへ戻す。I2C再構成の実機確認、GPIO/captureの競合は継続する。
  2026-09-21にP4（MAC `30:ed:a0:e3:11:08`）で、未予約UART/I2Cの拒否、UART RX12/TX6と
  I2C SDA50/SCL52の4-role単一planのlease取得、UART 115200設定、I2C target 10 kHz started、
  release後の両backend再拒否まで確認した。これはlifecycle確認であり、DUT側I2C transaction/traceの
  成否を示すものではない。
  同日にX035F8U6 route 2からP4 address `0x42`への4-byte writeを10 kHz/1 kHzで試したが、
  P4 receive callbackは各回増える一方、X035はaddress NACK（status 2）だった。I2C peerの
  release gateは未達であり、callback countだけをACK成功の根拠にしない。
  **2026-09-22 追記（v0 stack、`fixture.capture` で線上確認）**: X035 の address byte は正しく 0x84 で、P4 slave が 9 clock 目に SDA を引かず NACK
  になっている。IDF master（peer P4、GPIO32/33）には同じ slave 設定で ACK が出る。役割交換（route 4）でも同じ、P4 pin の駆動は X035 側 INDR で確認済み、
  slave 生成順序・SDA filter・P4 pull-up 追加では変わらない。X035 側の差は SDA hold 0.2〜0.4 µs と立上り 4 µs（外部 pull-up 無し）。
  **解決（同日）**: 原因は X035 側。PC16/PC17 は USB PHY pad で、`AFIO_CTLR.USB_PHY_V33`（reset 値で 1）が立っている間は open-drain の release が
  外部から引けない（X035 halt 中に P4 が GPIO50 を low 駆動しても X035 INDR は 1。bit を落とすと 0）。core `ch32_gpio_set_config()` が PC16/PC17 を
  出力系に設定する時にこの bit を落とす修正で route 2 は `S 84A 10A 11A 12A 13A P` / rc=0（100 kHz、10 kHz）。errata `x035-usb-pads-open-drain`。
  v1 slave の callback は NACK で終わった transaction でも発火し長さを持たないので、callback count も frame も ACK の根拠にならない（既知として固定）。
- [ ] `disable`、host disconnect、watchdog timeout で pin を input/release、peripheral を停止、
  trace を凍結する。
- [ ] `getStatus` は設定値、実効設定、開始結果、overflow、error、最後の timestamp を返す。
  host が serial log を解析して成否判定する設計にしない。

### P1.3 P4 固有 backend を profile の内側へ閉じる

- [ ] I2C hardware peer は Arduino `Wire` slave を使わず ESP-IDF direct slave driver を採用する。
  P4-P4 peerではArduino-ESP32 3.3.12により、fixed write、header+payloadのframed write、
  preload readを実証済みである。ただしv1 receive jobはtransactionと同じ正確な長さを要求し、
  callback内の再armはwatchdogを起こす。OEPにはまだframe/slotを指定するprotocolがないため、
  現在のdirect backendは4 byte固定・一回受信の診断に限定する。ISRはevent通知だけ、受信queue
  回収・次receive job・TX responseはtaskで行う公開backendを次に実装する。
- [ ] GPIO software I2C target は `i2c-target-software` capability として 10 kHz から公開し、
  deadline/jitter の実測値を返す。hardware target の代替として速度を主張しない。
- [x] （2026-09-22、v0 stack）observer は RMT ではなく PARLIO の sampled capture（`fixture.capture`）にした。RMT は線ごとに時間軸の原点が
  ずれて 2 本を揃えられない。read-only observer として同じ plan で I2C target と channel を共有でき、decode は host 側。二台 HIL で NACK / ACK trace 取得。
- [ ] 同様に UART、GPIO drive/sample、PWM observer、SPI peer/observer を backend interface の
  実装として追加し、example sketch 固有の固定 `setup()` から除去する。

**合格条件:** P4 firmware を再書込みせず、host が discovery 後に GPIO、UART、I2C observer/
peer を構成・解放・再構成でき、競合と予約 pin が正しく拒否されること。

## P2: capability ごとの HIL 基盤

- [ ] fixture test runner を用意し、image build、program、capability configure、target reset、
  assertion、trace/result 回収、cleanup を一つのケースとして実行する。
- [ ] 各ケースは target image hash、probe firmware revision、board profile、pin mapping、電圧、
  周波数、trace を artifact に保存する。
- [ ] 失敗時に target を halt したまま残さず、必要な trace を回収して reset/release する。
- [ ] P4 firmware 自身の OEP protocol unit test、client protocol test、実機 smoke test を分離する。

## P3: X035 全 peripheral 検証

P0--P2 が揃ってから、下表の順に戻る。各行は標準 Arduino API の正常系だけでなく、pin route、
速度、異常系、resource 解放後の GPIO 状態までを確認する。

| 優先 | 対象 | fixture capability / 測定 | 最低限の判定 |
| --- | --- | --- | --- |
| 1 | GPIO / `INPUT_PULLUP` / EXTI | `fixture.gpio`（両側駆動・観測） | 全接続 pin の 0/1、pull、edge、割込み回数、予約 pin 非干渉。**2026-09-22 実測**（ArduinoCore-CH32 `tests/manual/oep_gpio_matrix/`、13 pin）: core の 2 不具合を発見・修正 — (a) AFIO_EXTICR を F1 流 4 bit で書いていて port B/C の EXTI が来ない（X035 は 2 bit/line）、(b) X0 の GPIO に汎用 open-drain が無く `OUTPUT_OPENDRAIN` が high を駆動（errata `x035-no-gpio-open-drain`、core でエミュレート）。修正後 PA0〜7 / PB3 / PB11 / PB12 は全項目 OK。PC14/PC15（PD CC）は pull-up idle が 0 のまま（≈5 kΩ 級 pull-down、源は未決） |
| 2 | UART（USART4、次に USART2） | UART peer | 9600/115200、binary payload、長連続 TX/RX、overflow/error、reset 後再開 |
| 3 | ADC | 校正済み 0/Vref/2/Vref source | PA5 を含む接続 ADC pin の許容範囲、settling、repeatability。P4 内部 pull は基準電圧にしない |
| 4 | PWM / `analogWrite` / `tone` | `fixture.capture`（PARLIO） | 周波数、duty 0/50/100%、jitter、timer 資源競合、停止時 level。**2026-09-22 実測**: 1003.5 Hz、duty ±0.3 %、255/0 で edge 無し、tone 誤差 < 0.1 %（ArduinoCore-CH32 `tests/manual/oep_periph_trace/`） |
| 5 | 時刻 / delay / timer API | `fixture.capture` | `millis()`、`micros()`、delay、SysTick、公開 timer API の callback/解除と PWM 非干渉。**2026-09-22 実測**: millis 周期 20.005 ms / 20、`delayMicroseconds` は +3〜4 µs、`digitalWrite` ≈ 2 µs（todo） |
| 6 | I2C master / `Wire` route 2 | IDF hardware peer + 2ch RMT observer | 10/100 kHz write/read、repeated START、NACK、clock stretch、stuck-bus recovery。400 kHz は trace 合格後 |
| 7 | SPI master | `fixture.capture`（peer slave は未） | CPOL/CPHA、CS、MOSI/MISO、clock、連続 transfer、route、bus release。**2026-09-22 実測**: mode 0〜3 とも CPOL と MOSI 遷移エッジが仕様どおり、data 一致、prescaler 64/16/256。MISO 応答（peer slave）は未 |
| 8 | GPIO alternate/remap | PinMatrix + peer | 各 API の route 指定が予約/他 peripheral を壊さず、失敗を明示すること |
| 9 | reset/startup/boot interaction | target transport + GPIO observation | normal reset、software reset、boot entry の範囲、probe disconnect 時の安全状態 |

未対応と明示済みの EEPROM および `HardwareTimer` 移植は、この表の release gate に含めない。
timer は SysTick/loop を基本とする利用者ガイドと、必要時だけ使う公開 API の仕様が先である。

## リリース判定

- [ ] P0 reliability/throughput の基準値と合格ログがある。
- [ ] 常駐 firmware + discovery + resource lease で P3 の全ケースを実行した。
- [ ] capability ごとの速度、精度、電圧、既知の制限を利用者向けガイドへ反映した。
- [ ] probe/target 両方の failure injection で安全な cleanup を確認した。
- [ ] X035 の結果を横展開可能な board profile / capability contract として保存し、別 MCU に固有 pin
  番号や ESP32 固有 API が漏れていないことを review した。
