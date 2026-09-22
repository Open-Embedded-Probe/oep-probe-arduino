# 開発用プローブ再構築の全体計画（2026-09-22）

状態: **作業計画**。この文書が進行順の正であり、
[x035-release-worklist.ja.md](x035-release-worklist.ja.md)（release gate の証跡）と
[progress-and-next-work-2026-09-22.ja.md](progress-and-next-work-2026-09-22.ja.md)（棚卸し）は参照先として残す。
個々の実測は wch-protocols `experiments/LEDGER.ja.md` が正。

## 方針

1. **既存実装の温存は前提にしない。** 現 prototype（96-byte COBS frame、stop-and-wait、function ごとの手書き if-chain、
   `digitalWrite` bit-bang）はゼロベースで、単純で拡張性の高い仕様へ破壊的に作り替える。
2. **優先度は仕様の拡張性が最上位。** 速度だけの改善は「後回し task」へ載せる。ただし**実験の所要時間を圧迫する機能**
   （例: full image program 12 s、verify 5 s）は、予備実験で先に潰してよい。
3. **firmware は使う前に必ず転送する。** pytest（pytest-embedded-arduino-cli）で build → upload → 試験を 1 セットにする。
   `sketch.yaml` で platform/library を pin し、port・pin は `.env`。過去 flash の保全は不要。
4. **実験は wch-protocols の規則**（README 計画先行、同名 `.py`、銘板、`_runs/` 退避、採番）に従う。
   E142〜E150 は `.py` 無し・手動操作で逸脱していた。以後は逸脱しない。
5. 機材はこの session が占有する。別 repo への書込みと commit も本 session では許可されている。

## 現在地（土台の実測、2026-09-22）

| 実験 | 結果 |
|---|---|
| E151 GPIO edge cost | P4 の GPIO register は 1 access 300 ns。`digitalWrite` 570 ns/edge、`gpio_ll` 300、dedicated GPIO **52.8**（read 25.0）。RVSWD 1 bit: 1,770 / 800 / **94.5 ns** |
| E152 `gpio_ll` RVSWD | half 0 ns でも 1 DMI read 53 µs（pp）。P4 コスト律速で target 上限は見えない。od は half 0 で崩れる |
| E153 dedicated GPIO RVSWD | **pp + half 0 ns で X035F8U6 が全数一致、1 DMI read 10.1 µs（現行の約 1/12）**。od は half 300 ns 以下で崩れる |
| E154 8 本リンク | GPIO 33,32,26〜31 同番号が両方向 1 対 1。`peers` fixture で二台同時 pytest が通る |
| E155 USB-Serial/JTAG | 往復 min 0.36 / median 1.3 / p95 11.7 ms（usbipd/WSL）。512 B × in-flight ≥4 で ≈320〜345 kB/s 飽和。outstanding > ring 8 KiB で HWCDC がデータを落とす。port open/close で P4 が reset |
| E156 flash 読出し方式 | dedicated GPIO PHY で 62 KiB を **0.146 s（435 kB/s、現行の 34 倍）**。autoexec + poll なし + 末尾 cmderr 確認。halt 直後の不安定期間は未決（`x035-halt-settle`） |
| E157 page program 方式 | erase 3.3 + program 2.9（autoexec writer、word DMI の 6 倍）+ verify 0.7 ms ≈ 7 ms/page、62 KiB ≈ 1.7 s（現行の約 60 倍）。half 0 ns の間欠 parity 不一致は run 単位で再発 → PHY は session 開始時に margin check、正否は CRC で判定 |
| S2 registry/codegen | `oep-spec/registry/oep-v0.yaml` → C library / Python module / vectors。host core で C と Python が全 vector 一致、`--check` で同期確認。Uno build は test sketch の vector 表が RAM 超過（codec 本体は未確認、task） |
| S3 第 1〜2 段 | v0 endpoint + core + probe.identity + RVSWD PHY + Ch32Dm + target.control/memory/flash。HIL（build→upload→test）で **62 KiB read 0.154 s、page program 7.3 ms/page、host CRC = probe CRC**。旧 prototype src は削除 |
| S4 第 1 段 | oep-client-python `oep_client.v0`（frame transport、byte window pipelining、core、target service wrapper、fake endpoint の unit test） |
| S3 第 3 段 / S4 第 2 段 | lease（plan_apply / release、watchdog 解放）、fixture.gpio / fixture.uart、peer P4 との二台 HIL（GPIO mirror、UART echo 512 B）。host `program_image`（ESIG preflight、page 差分、CRC verify、result JSON）、CLI。**P0 reliability gate を新 stack で再取得**（verify ×20 0.28 s、32 page program ×20 0.29 s、中断 ×5 復旧） |
| X035 reset 挙動 | ndmreset 後に hart が走らない回（約半分、走行中 hart への reset で顕著）。DM の状態読出しは当てにならず、線解放→再 attach で必ず走る。`Ch32Dm::reset()` を状態機械化して 43/44。旧 prototype は program → verify → reset の二重 reset で隠れていた可能性。候補 `x035-ndmreset-hart-not-running` |
| S4 runner | ArduinoCore-CH32 `tests/manual/oep_smoke/oep_smoke.py`（compile → OEP program_image → fixture.uart lease → READY/PING/expectations → 判定）。**basic 14 sketch が F8U6 で 14/14 PASS**（LinkE / probe-rs なし、1 sketch ≈ 30 s、書込み 0.7 s） |
| vendor tool 初回 | registry に owner `0x0100`（oep-probe-arduino）と `p4_i2c_target`（fixed-rx / framed-rx / preloaded-tx）。firmware は IDF slave v1 を task 側で再 arm、peer P4 controller との二台 HIL で **4 B/32 B write、16 B@100 kHz/128 B@1 MHz framed、2 slot preload が全一致**（E147〜E150 の OEP 移植完了）。arm 長の変更は device 再作成（v1 に cancel が無い）。**2026-09-22 追記**: 一時 400 kHz に下げていた宣言を 1 MHz に戻した。「1 MHz で 128 B の末尾が欠ける」は peer の HWCDC RX ring（既定 256 B）が 269 文字の FRAME 行を切っていた test 側の artifact で、slave は header どおり 121 B を受けていた。peer に `setRxBufferSize(4096)`、framed 128 B@1 MHz 20/20 |
| E158 reset 証拠 | debug reset 後に hart が reset vector に駐留する回が約 3〜5 %（DMSTATUS は running）。haltreq→resumereq で 15/15 解放。reset は PC sample（dpc≠0）を完了条件にして 200/200、描述 TLV `max_clock_hz` = 6.3〜6.4 MHz |
| fixture.capture | PARLIO RX 有限長（soft delimiter ≤ 65535 B、内部 DMA RAM 64 KiB）、1 byte/sample、observer lease（channel を claim しない）。1 MHz × 20,480 sample の回収 26〜40 ms。I2C decode は host（`oep_client.v0.decode`）。I2C slave と同じ GPIO32/33 を同じ plan で共有して動作 |
| worklist B trace | X035 route 2 → P4 slave 0x42: 線上 `S 84N P`（address 正、slave 無 ACK）。peer IDF master → 同 slave は `S 84A …`。X035 側 SDA hold 0.2〜0.4 µs、立上り 4 µs（GPIO50/52 に外部 pull-up 無し）。route 3（SWD 線）は probe と衝突。`p4.i2c-target` v1 の fixed-rx は NACK transaction でも stale frame を返す（長さ情報無し） |
| chip-id | device-data `evidence/device_ids.csv`: F8U6 `0x035E0601`、C8T6 `0x03510601`。fixture は **F8U6**（E144/E145 の C8T6 表記は誤り） |

現 prototype の速度問題は (a) PHY の GPIO コスト、(b) word ごとの ABSTRACTCS poll、(c) 96-byte frame と stop-and-wait、
(d) Python の 1 byte read、に分解できた。(a) は E153 で解決策が確定した。

## 機材フェーズ

- **Phase A: P4 二台 8 本 GPIO 直結**（現状）。fixture P4 `30eda0e31108`（X035F8U6 を RVSWD 接続）と peer P4 `30eda0e34a0e`。
  HS USB は cable が埋まっており使えない。**peer を使う実験・8 本リンクで済む実験をここで全部消化する。**
  結線は **GPIO 33, 32, 26, 27, 28, 29, 30, 31 の同番号同士**（E154 で両方向 1 対 1 を確認、2026-09-22）。
  fixture P4 の他 pin は X035 に繋がっているので peer 実験ではこの 8 本以外を駆動しない。
- **Phase B: HS USB**（cable を差し替え。8 本リンクと排他）。vendor bulk / HS CDC の transport 実験と、
  streaming 系（capture download）はここへ集める。差し替えは Phase A の候補が尽きてから一度だけ行う。

## 作業項目（優先順）

### P0 仕様の拡張性（最優先）

| ID | 項目 | 成果物 | 依存 |
|---|---|---|---|
| S1 | **core wire model v0 draft**: frame（length16 + seq + type + payload + CRC）、pipelining window、request / response / event、chunked transfer（transfer id）、service id + revision + private namespace、TLV capability、reject / failure model、session / lease | oep-spec `docs/v0-*.ja.md` | E155（USB-Serial/JTAG 往復）の数値で window / frame を決める |
| S2 | **registry と codegen**: message 定義を 1 つの YAML に置き、C++ pack/unpack と Python codec と test vector を生成 | oep-spec `registry/`、生成物は各実装 repo | S1 |
| S3 | **firmware 骨格**（oep-probe-arduino を作り直し）: transport 抽象（stream / bulk）、dispatcher、service registry、RVSWD PHY（dedicated GPIO、pp、明示 turnaround、session 開始時の half period margin check、parity 失敗の bounded retry）、Target service（memory / flash を physical page 単位の transaction に）、Probe service（info / caps / lease）、Fixture service（GPIO、UART、I2C target `fixed-rx` / `framed-rx` / `preloaded-tx`、capture） | oep-probe-arduino `src/` | S2、E153、E156、E157 |
| S4 | **client と runner**: registry から生成した codec、CLI、pytest HIL fixture、ArduinoCore-CH32 sketch runner（compile → program → UART → assert） | oep-client-python、ArduinoCore-CH32 `tests/` | S2、S3 |

拡張性の判定基準: 未知 service / operation / TLV を副作用なく reject または無視できる、service revision を独立に上げられる、
probe MCU 固有の pin 番号や API が wire に漏れない、同じ registry から 2 実装が生成される。

### P1 P0 に数値を与える実験（Phase A で実施）

| ID | 問い | 用途 |
|---|---|---|
| E154 | （完了）8 本リンクの pin 対応 | GPIO 33,32,26〜31 同番号 |
| E155 | （完了）USB-Serial/JTAG の往復・帯域・window | S1: frame 512 B〜1 KiB、window は byte 数で 4 KiB |
| E156 | （完了）読出し方式。poll なし autoexec で 0.146 s | target.memory は poll なし + 上位 CRC で検証 |
| E157 | （完了）page program 方式。autoexec writer 2.9 ms/page | target.flash は physical page 単位の transaction（erase → autoexec program → read-back CRC を probe 内で 1 request） |
| E157〜 | peer P4 を相手にした fixture 能力の HIL: UART peer、I2C target 3 mode（E147〜E150 の OEP 経由移植）、GPIO drive / sample、RMT capture、SPI peer | S3 の各 service の受入試験 |

### S3 / S4 の残り（次に着手）

1. （完了）fixture.gpio / fixture.uart / lease、二台 HIL。
2. （完了）`program_image` と CLI、ArduinoCore-CH32 sketch runner（basic 14/14 PASS on F8U6）。runner は ArduinoCore-CH32 側で未 commit。
3. （完了）P0 reliability gate を新 stack で再取得。
4. （I2C 完了、capture 完了）P4 独自 tool `p4_i2c_target` を二台 HIL で実証。共通 tool `fixture.capture`（sampled logic capture、observer 型: 他 function が使う channel を同じ plan で観測できる）を
   PARLIO 有限長 RX で実装し、二台 HIL で 1 MHz × 20 ms を 26〜40 ms で回収、host 側 I2C decoder で `S 84N P`（無応答）と `S 84A 10A 11A 12A 13A P`（target 同席）を得た。
   SCL 周期 10 sample = 100 kHz で sample rate の実測が取れる。RMT（duration 型）は 2 本の時間軸が揃わないので採らず、PARLIO の同時 sample を共通契約にした。
   capture の宣言上限は 20 MHz（HIL で 100 kHz SCL の周期 200 sample を確認、26 万 sample の回収 0.34 s）。
   **X035 I2C NACK の trace（worklist B）は完了**: ArduinoCore-CH32 `tests/manual/oep_i2c_trace/`。X035 の address byte は正しく（0x84）、P4 slave は
   address を受けているのに ACK を出さない。IDF master には ACK する。除外: pin（役割交換でも同じ、INDR で駆動確認）、生成順序、SDA filter、P4 pull-up。
   残る容疑は X035 の SDA hold 0.2〜0.4 µs か立上り 4 µs（外部 pull-up 無し）への ESP32-P4 slave の感度。hold/立上りを制御できる master が要る（台帳候補 `x035-p4-slave-no-ack`）。
   **残り: 共通 `fixture.i2c-target`（丸めた契約）の要否判断**。
5. （完了、E158）reset 契約の確定: DMSTATUS だけの reset は 96/100・98/100 で、欠落は全て hart が reset vector に駐留したもの（周辺 register 全て reset 値）。
   target.control reset は `confirm=1` で解放後に halt → dpc → resume を行い、dpc=0 は「駐留を解放した」として再 sample、dpc≠0 で完了（`flags` bit1、`pc`）。200/200。
   E159（7 列 × 550 cycle）: DMCONTROL の順序で駐留率は 1〜8 % の間で変わるがどの列でも 0 にならない。haltreq を reset 越しに保持する列は駐留は減るが、
   probe に入れて E158 を再測すると banner 150/300（走っているのに SysTick が止まる状態が 124）で不採用。`resetOnce` は baseline のまま、契約は PC sample + resume。
   残る問いは「駐留・DMI 乱れ・割込み停止の共通原因」（台帳候補 `x035-ndmreset-hart-not-running`、clock 状態の相関）。
6. （完了）capability の実測値公開: attach の margin check で 1000 read の wall time を測り、target.control の describe TLV `max_clock_hz` に SWCLK 実測（half 0 ns で 6.3〜6.4 MHz）を返す。

### P2 Phase B（HS USB）

vendor bulk transport（WinUSB flat layout、E081）、HS CDC 往復、in-flight 深さ、capture streaming。
S1 の transport 抽象がここで 2 つ目の実装を得る。

### P3 X035 core 検証

worklist の P3 表を、新 stack（S3 + S4）だけで上から実施する。X035 I2C NACK の trace（RMT 2ch）は E157 系の capture が
使えるようになった時点で行う。

## 後回し task（速度のみ、または現時点で不要）

- P0.2 の内訳 telemetry（attach / read / erase / program / verify の wall time、retry）。
- 生成 codec の低スペック確認: test sketch の vector 表を PROGMEM に置くか、codec 単体の AVR / CH32V003 build で footprint を測る（Uno は現状 RAM 超過）。
- registry から生成した C library を oep-probe-arduino へ取り込む経路（submodule / copy / `libraries: dir:`）を決める。
- 256-byte commit の変更 page 反復と failure injection の自動化。
- Python client の 1 byte read ループ。
- LinkE 比の速度目標。**release 判定は速度ではなく reliability gate で行う。**
- EEPROM、`HardwareTimer` 移植（worklist どおり gate 外）。

## 完了の定義

1. S1〜S4 が揃い、E157 系の peer HIL と worklist P0 の reliability gate（verify 20 回、差分 program 20 回、中断 5 回）が新 stack で通る。
2. capability 宣言と実装と HIL 結果が一致し、未実証 mode を返さない。
3. X035 の P3 表を常駐 firmware だけで再現できる。
