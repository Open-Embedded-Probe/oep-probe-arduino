# CH32X035 release 検証へ戻るための作業リスト

対象は ESP32-P4 / CH32X035C8T6 fixture と OEP prototype である。ここでいう完了は
「特定の診断 sketch が一度動く」ではなく、同じ常駐 probe firmware と公開された OEP
capability だけを使い、再現可能な HIL 試験を実行できる状態を指す。

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

プロトコル破損経路も実機確認した。validな4-byte code-flash readでRVSWD sessionをactiveにした直後、
CRCに届かない不正COBS frameを送った。endpointはframeを破棄し、最後のvalid requestから2秒待機して
watchdogがnormalizationする。その後の別clientによるPWM image全域verifyはhash一致で完走した。これで
host kill/timeout（5回）、protocol corruption（1回）、P4 firmware再書込み/reset（cache喪失+完全image
復旧）の各経路を、targetをhaltで放置しないことまで確認した。電源を物理遮断する試験はP4再書込みresetと
同じRAM cache消失を持つが、電源電圧/USB再列挙の別条件はP0 hardware-soakで独立に残す。

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

- [ ] `ProbeInfo` に probe firmware revision、board profile id、target transport、予約 pin、
  電圧範囲、利用可能な capability revision を追加する。
- [ ] `PinMatrix` に各 fixture pin の target pin、方向、input-only、open-drain 可否、
  ADC source/measurement 可否、予約理由を返す。現行 X035/P4 対応表を profile の初期値にする。
- [ ] capability ごとに候補 pin set と相互排他 resource を返す。host は任意 GPIO 番号を
  仮定せず discovery 結果だけで構成する。

### P1.2 共通の lifecycle

- [ ] 全 capability を `configure(config) → enable → getStatus/readResult → disable` に統一する。
- [ ] `configure` は pin/resource lease を原子的に取得し、競合（例: GPIO50/52 を I2C と
  GPIO drive で同時使用）を structured error として返す。
- [ ] `disable`、host disconnect、watchdog timeout で pin を input/release、peripheral を停止、
  trace を凍結する。
- [ ] `getStatus` は設定値、実効設定、開始結果、overflow、error、最後の timestamp を返す。
  host が serial log を解析して成否判定する設計にしない。

### P1.3 P4 固有 backend を profile の内側へ閉じる

- [ ] I2C hardware peer は Arduino `Wire` slave を使わず ESP-IDF direct slave driver を採用する。
  ISR は event 通知だけにして、受信 queue 回収・次 receive job・TX response は task で行う。
- [ ] GPIO software I2C target は `i2c-target-software` capability として 10 kHz から公開し、
  deadline/jitter の実測値を返す。hardware target の代替として速度を主張しない。
- [ ] RMT observer は SCL/SDA の 2 RX を同時 arm し、raw duration と decoded event を取得する。
  これは peer と独立に有効化できる read-only capability とする。
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
| 1 | GPIO / `INPUT_PULLUP` / EXTI | GPIO drive + sample | 全接続 pin の 0/1、pull、edge、割込み回数、予約 pin 非干渉 |
| 2 | UART（USART4、次に USART2） | UART peer | 9600/115200、binary payload、長連続 TX/RX、overflow/error、reset 後再開 |
| 3 | ADC | 校正済み 0/Vref/2/Vref source | PA5 を含む接続 ADC pin の許容範囲、settling、repeatability。P4 内部 pull は基準電圧にしない |
| 4 | PWM / `analogWrite` / `tone` | RMT PWM observer | 周波数、duty 0/50/100%、jitter、timer 資源競合、停止時 level |
| 5 | 時刻 / delay / timer API | GPIO/RMT marker | `millis()`、`micros()`、delay、SysTick、公開 timer API の callback/解除と PWM 非干渉 |
| 6 | I2C master / `Wire` route 2 | IDF hardware peer + 2ch RMT observer | 10/100 kHz write/read、repeated START、NACK、clock stretch、stuck-bus recovery。400 kHz は trace 合格後 |
| 7 | SPI master | hardware SPI peer + RMT observer | CPOL/CPHA、CS、MOSI/MISO、clock、連続 transfer、route、bus release |
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
