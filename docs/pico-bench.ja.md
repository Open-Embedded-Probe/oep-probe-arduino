# Pico bench（RP2040-Zero + Pro Micro RP2350）2026-09-23

## 機材と配線（暫定）

| 役割 | board | 識別 | 配線 |
|---|---|---|---|
| ordinary ARM SWD の対抗 | Waveshare RP2040-Zero（"PICO Zero"） | BOOTSEL serial `E0C9125B0D9B`、`2e8a:0003` | GP0 / GP1 → Pro Micro RP2350 の **SWD ヘッダ**（どちらが SWCLK / SWDIO かは未記録） |
| OEP probe（CH32L103 向け） | SparkFun Pro Micro RP2350（"PICO2"） | flash uid `9489dd2ae0953650`、BOOTSEL `2e8a:000f` | header の**右半分ぐらい**が CH32L103 へ。RVSWD の pair は未特定 |

利用者の方針（2026-09-23）: **配線は暫定で入れ替える可能性がある**。Pico は PIO のために pin の並びが効くが、
**当面 PIO は使わず低速の bit-bang**。まずは **ordinary SWD と単純な GPIO** を今の配線でできる範囲だけ確認し、
**フルテストは CH32V003 を決めた pinout で PICO Zero か PICO2 に繋いでから**行う。

## firmware

| sketch | profile（`sketch.yaml` で版を固定） | 内容 |
|---|---|---|
| `examples/Rp2350L103Probe` | `rp2040:rp2040 (6.1.1)` / `sparkfun_promicrorp2350:usbstack=picosdk` | OEP v0 endpoint を USB CDC に出す。probe.identity / target.control・memory・flash（RVSWD、既定 GP2=SWDIO, GP3=SWCLK、`-DOEP_RVSWD_SWDIO=` で変更）/ fixture.gpio / fixture.uart |
| `examples/Rp2040SwdSurvey` | `rp2040:rp2040 (6.1.1)` / `waveshare_rp2040_zero:usbstack=picosdk` | ordinary ARM SWD の bring-up。GP0/GP1 の両向き × half period × multidrop TARGETSEL 候補を掃いて DPIDR が返る組を報告する（read only） |

core は **global に入れない**。`arduino-cli compile --profile <name>` が `~/.arduino15/internal/` へ pin 止めで展開する。

## 実装の構成

- `src/OepRp2BitBang.h` — RP2040/RP2350 の SIO bit-bang pin 操作。RVSWD と ARM SWD が共有する。PIO は使わない（配線が暫定のため）。
- `src/OepRvswdFrame.h` — CH32 RVSWD の frame（1 実装。ESP32 dedicated GPIO backend と RP2 SIO backend が同じ frame を使う）。
- `src/OepSwdFrame.h` — ordinary ARM SWD（ADIv5）の frame。request / turnaround / ACK / parity / line reset / multidrop TARGETSEL。
- `src/OepPlatform.h` — core 間の差（pin mode、UART の pin 指定とバッファ）。

## 書き込み

BOOTSEL の RP2 は libusb 経由なので、WSL では udev rule が要る（1 回だけ）。

```sh
sudo tee /etc/udev/rules.d/99-rp2-picotool.rules >/dev/null <<'RULE'
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0666", TAG+="uaccess"
RULE
sudo udevadm control --reload-rules && sudo udevadm trigger
```

その後は WSL から `arduino-cli upload --profile <name> --input-dir <dir> <sketch>`。
**VID/PID が変わる遷移（MicroPython → BOOTSEL → 自作 firmware）では usbipd の bind をやり直す**必要がある（管理者権限）。

## 2026-09-23 の実測

### ordinary ARM SWD は通った

RP2040-Zero → Pro Micro RP2350 の SWD ヘッダ。**SWCLK = GP0、SWDIO = GP1**（利用者の記憶どおり「0 と 1」、向きはこの通り）。

| 項目 | 値 |
|---|---|
| ACK | OK（`0b001`） |
| DPIDR | **`0x4c013477`** = Arm designer、DP **version 3**（ADIv6）、RP2350 の SW-DP |
| 起動系列 | `jtag->swd` / `dormant->swd` / line reset のいずれでも応答 |
| **write 経路** | DP CTRL/STAT へ `0x50000000`（CDBGPWRUPREQ + CSYSPWRUPREQ）を書くと ack OK、読み戻し **`0xf0000000`** = **debug domain が power up**。read だけでなく write・ACK・turnaround・parity が揃って通る |
| multidrop TARGETSEL | **不要**。line reset 直後の DPIDR read に答える（RP2040 と違う） |
| half period | 500 ns（bit-bang、SIO 直叩き） |

同じ線で CH32 RVSWD を試すと無応答（正しい陰性対照）。

### pin survey: 何が繋がっているか

released input を読むだけでは RP2350 の**エラッタ E9（浮いた入力が high に張り付く）**で全 pin が「pull-up」に見える。
pad 側の pull を当てて読むと切り分けられる。

| board | 外部 pull-up | 外部 pull-down | それ以外 |
|---|---|---|---|
| RP2040-Zero | GP0, GP1（SWD の 2 本、≤10 kΩ 級） | なし | 18 本すべて free |
| Pro Micro RP2350 | **GP24** | **GP23**, GP29 | 他は free |

GP24 / GP23 は CH32 の SWDIO(PA13, pull-up) / SWCLK(PA14, pull-down) の signature と一致するので、L103 の RVSWD pair の
**候補**として example の既定にした（未確認）。

### L103 は応答しない = 無給電

Pro Micro から ARM SWD 380 組・CH32 RVSWD 420 組を総当たりして、どれも無応答。**CH32L103 基板に電源が来ていない**
（利用者確認済み、2026-09-23。WCH-LinkE `0E028F0692F1` も挿さっていない）。したがってこの結果は配線の否定にはならない。

給電後の再測は 1 コマンドで済む:

```sh
cd <scratchpad>
./picoflash.sh 9-1 /dev/serial/by-id/usb-SparkFun_ProMicro_RP2350_9489DD2AE0953650-if00 \
    $PWD/surveyp/PicoDebugPortSurvey.ino.uf2 2e8a:000f
# console を読むと pull signature と RVSWD pair sweep が出る。確定したら
# examples/Rp2350L103Probe の既定 pin を直して probe firmware に戻す。
```

### 両機の GPIO は互いに繋がっていない

両方を OEP probe にして片側を駆動・他側で bank read（E143 と同じ方法）→ 1 本も動かない。
Zero の GP0/GP1 は Pro Micro の **GPIO ではなく SWD ヘッダ**に行っている、という配線の裏付け。

## L103 に給電したあと（同日、利用者が電源を接続）

### RVSWD の pair が確定

Pro Micro の **SWDIO = GP0、SWCLK = GP1**。全 420 組の総当たりでこの 1 組だけが応答し、DMSTATUS = `0x00000c82`
（version 2、authenticated、allrunning）。GP0/GP1 は Serial1 の既定 pin でもあるので、`fixture.uart` には別の組を割り当てる。

### 半周期の実測（この配線・flying wire）

| half | 冷えた bus の初回応答 | 連続 1000 read |
|---:|---|---|
| 0 / 25 ns | 応答なし | — |
| 50 ns | 応答するが内容が壊れる | identical 0 / differed 793 / parity fail 207 |
| **100 ns** | 安定 | **1000 / 1000** |
| 200 / 500 ns | 安定 | 1000 / 1000 |

`attach()` は 100 ns を選ぶ。1 DMI read は約 26 µs。

### `attach()` の修正 2 点（どちらも実測が根拠）

1. **半周期ごとに bus を張り直す**。追従できない半周期を試すと DM が歩調を崩し、そのままだと後続の遅い候補まで巻き添えになる。
2. **冷えた DM は最初の wake に答えない**。500 ns で「5 回目の試行」で初めて応答し、以後は 95/100。よって候補ごとに
   wake を最大 8 回試してから判定する。これを入れる前は、温まっていない L103 が「ターゲット無し」に見えていた。

修正後、OEP の `target.control.read_dmi` が probe 経由で DMSTATUS を返す。

### 未解決: halt が効かない（原因は 2 候補まで絞り込み、特定は未）

| 試したこと | 結果 |
|---|---|
| `DMCONTROL` へ 0x80000001 を書いて**読み戻す** | `0x80000001`（haltreq=1）。**書き込みは届いている** |
| その直後の DMSTATUS | `0x00000c82` のまま = allrunning。halt しない |
| haltreq のまま 10〜80 ms 追跡 | `0xfffffdbe` / `0xfffffffe` など**ほぼ全 1 の壊れた語**に劣化する |
| 半周期ごとの bus 再初期化（`wakeBus()`）を挟んで再読 | 同じ。allrunning か壊れた語 |
| haltreq + ndmreset → ndmreset だけ解除（reset 経由の halt） | 返る語がやはり全 1 系。`allhalted=1` に見える回も内容が壊れており**信用できない** |

つまり **halt を要求した後だけリンクが壊れる**。hart が走り続けているのか、halt はしているが読めていないのかを
区別できていない。候補は 2 つ:

1. **DMCONTROL への書き込みが時々化ける**。RVSWD の write は ACK も検証も無いので、bit 0（dmactive）が落ちれば
   DM 自体が寝てバスは全 1 になる。読みが 1000/1000 clean でも write が同じ品質とは限らない。
2. **L103 側が低消費電力モード**で core clock が止まっており halt を受け付けない。

### 決着（2026-09-23）: 原因は 2 つ、どちらも予想外だった

**(a) プローブの未使用パッドが RP2 既定の pull-down のままだった。** RP2 の pad は reset 直後 pull-down 有効。
この治具は半分しか結線していないので、そのうちの 1 本が L103 のリセット系に乗っていたと見られる。
**空きピンを `pinMode(INPUT)` で Hi-Z にしただけで halt が 5/5 → 8/8 通るようになった。**
切り分けは「halt が通る bench sketch」と「通らない probe firmware」の差分から出た。前者は毎周 `pinMode(INPUT)` で
全空きピンの pull を外していた。→ `platformParkPins()` を追加し、probe sketch は service 起動前に必ず呼ぶ。

**(b) 100 ns（5 MHz）ではリンクが限界で、halt の *書き込み* だけが化けていた。** DMSTATUS は 1000/1000 一致、
`DMCONTROL` を読み戻すと `0x80000001` が入っているのに hart は走り続け、abstract command は cmderr=4。
読みが clean でも書きが同じ品質とは限らない、が実測で確定した。

判定は DMSTATUS ではなく **abstract command（halt 必須の register read）が cmderr=0 で通るか**で行うこと。
`allhalted=1` に見える値の大半は化けた word だった。

### attach() の変更（2026-09-23）

- **write の読み戻し検査**: 候補の半周期ごとに DATA0 へ 8 パターン × 32 周を書いて読み戻す。読み検査（1000 回）と
  同じ重さにしないと 200 ns を通してしまい、その先のメモリ読みが崩れた。
- **DATA0 を使う前に ABSTRACTAUTO と cmderr を落とす**。中断した flash 手順が autoexec を残していると、
  DATA0 に触るたびにコマンドが走って読み戻しが永久に合わない（実際に踏んだ）。`configureBus()` 側で毎回落とす。
- **候補リストを 2 巡**。冷えた DM は 1 候補あたり 8 回の wake では足りないことがあり、速い候補の失敗が
  結果的に warm-up になっていた。floor を入れて候補が 1 つになった途端に初回失敗が出た。
- **`setMinHalfNs()`**: 暫定配線の治具は下限を明示する。この L103 治具は **500 ns**（100/200 ns は検査を通るのに
  長いシーケンスが崩れる）。X035 の PCB 治具は floor 無しのままで、`basic` 14/14 PASS・書込み 0.79 s と変化なし。

### `Ch32Dm::halt()` の変更

minichlink と同じく **halt request を繰り返す**（1 発では入らない）。各ラウンドの頭で `phy_.reinit()` し、
成立後も 1 回 reinit する（hart の状態が変わると DMI が落ち、直後の 1 トランザクションが失われるため）。

### バスを待機させるレベルが決定的だった（2026-09-23）

CH32 の 2 線デバッグは **両線 High で放置されるとリンクをリセットする**。フレームの stop 条件が
`bothHigh()` のままだったので、トランザクション間はいつもその状態だった。実測：

| アイドル | 結果 |
|---|---|
| 500 µs | 健全 |
| 1 ms | 以後の読みが全部 `0xffffffff` |
| 5 ms 以上 | 1 回の再初期化では戻らない（復帰に 12 回）。**DM がリセットされ hart が走り出す** |

待機を **SWCLK Low** にすると 3 秒放置しても `DMSTATUS 0x00000382`（halt 維持）のまま、メモリも読める。
→ `stopFrame()` は stop 条件を出した後にクロックを下げて待機する。X035 も 14/14 PASS で影響なし。

これが「OEP 経由だけ壊れる」の正体だった。リクエスト間は必ず数十 ms 空くので毎回リンクが死に、
`readWords` は **cmderr を見ずに成功を返していた**ため、DATA0 に残っていた値（`loadRegisters` が書いた
a1 = `0xe0000384`）をデータとして返していた。ベンチ sketch は連続実行なので気付けなかった。

関連して直したもの：

- `readWords()` は cmderr 0 と 3（末尾での先読み例外）以外を **失敗として返す**。黙って嘘をつかない。
- `reviveIfIdle()`: 300 µs 以上空いたら、まず生存確認、駄目なら wake 無しの再同期、それでも駄目なら wake 付きを 12 回。
- **wake バースト（100 クロック）と再同期を分離**した。`configureBus(bool with_wake)`。再同期のたびに
  wake を撃つのは、ターゲットを起こし直すのと同じで乱暴。
- `attach()` は既に dmactive が立っていれば **DMCONTROL を書かない**。あの書き込みは haltreq を落とすので、
  他人が halt したターゲットに attach しただけで走り出してしまう。安定性検査もその遷移で崩れていた。
- `detach()` は `release()`（Hi-Z）ではなく **`park()`（クロック Low で駆動したまま）**。Hi-Z だと内部プルアップで
  両線 High になり、せっかく resume したターゲットがリセットされる。
- `resume()` も halt と同じく **要求の繰り返し**。この部品は `allresumeack` を一切上げないので、
  「走っている（allrunning かつ not halted）」も成功と見なす。

### 読み出し保護は解除した（2026-09-23、利用者の許可のもと）

`FLASH_OBTKEYR` bit1 が立っていた（minichlink の判定条件と同じ）。フラッシュはどの番地も同じ語を返し、
ESIG・UID・RAM だけ正しく読める状態だった。

解除は option byte 消去 → `RDPR = 0x5aa5` の **ハーフワード書き込み**（word 書きでは隣の USER を巻き込む）。
`Ch32Dm::writeHalfWord()` と `target.memory` write の 2 byte 許容を追加。`FLASH_CTLR` に書く値は
**すべて絶対値**にした（化けた読み値を書き戻さないため）。手順は `scratchpad/l103_unprotect.py`。

結果: `OBTKEYR` bit1 クリア、main flash は全消去されて読めるようになった。ESIG `0xffff0040`（64 KiB）、
UID `0x3a6fabcd` / `0xa284bc48`、`DMCHIPID 0x20000410`（DEVID 0x410 = V103/L103 系）。

### CH32L103 でコアのスケッチが動いた

`Generic CH32L103` (`pnum=CH32L103C8T6`) でビルドしたスケッチを OEP で書き込み：

- 4 ページ 0.185 s、probe 側 CRC32 一致、リードバック一致
- reset 後 pc が進む、`target.control` の halt / resume / memory が通る
- コンソール未結線なので **0x20001000 のカウンタ**で実行を確認。resume 時間との比例が完全：
  100 ms → 57,563、300 ms → 171,742、1000 ms → 571,283（一定 572 kHz）

### リンク品質そのもの

500 ns 半周期で **読み 0/2000、書き戻し 0/2000**。100/200 ns は DM が起きない。飛び配線でも
steady state は綺麗で、これまでの「品質が悪い」という見立ては誤りだった。実際は上記の待機レベルの問題。
暫定として `setMinHalfNs(500)` を残してあるが、これは配線確定後に外して測り直す価値がある。

### 実測した配線マップ（2026-09-23）

推定ではなく実測。ターゲットを halt すれば L103 の GPIO レジスタは叩けるので、**L103 側の 1 ピンずつ駆動 →
Pico 側で受信**と、**Pico 側の 1 ピンずつ駆動 → L103 の `INDR` で受信**の両方向で取った。1:1 に割れるのは
逆方向（Pico が駆動）なので、そちらを正とする。手順は `scratchpad/l103_wiremap.py`。

| Pro Micro | CH32L103 | 主な用途 |
|---|---|---|
| GP0 | PA13 | SWDIO |
| GP1 | PA14 | SWCLK |
| **GP2** | **NRST** | 外部リセット（下記） |
| GP3 | PB5 | SPI1_MOSI(remap) / TIM3_CH2(remap) |
| GP4 | PB3 | SPI1_SCK(remap) / TIM2_CH2(remap) |
| GP5 | PA12 | **USB_DP** / TIM1_ETR |
| GP6 | PA10 | USART1_RX(既定) / TIM1_CH3 |
| GP7 | PA8 | TIM1_CH1 / MCO |
| GP10 | PC13 | TAMPER-RTC（ボードの LED の可能性） |
| GP11 | PB8 | I2C1_SCL(remap) / TIM4_CH3 |
| GP12 | PB7 | I2C1_SDA / USART1_RX(remap) / TIM4_CH2 |
| GP13 | PB6 | I2C1_SCL / USART1_TX(remap) / TIM4_CH1 |
| GP14 | PB4 | SPI1_MISO(remap) / TIM3_CH1(remap) |
| GP15 | PA15 | SPI1_NSS(remap) / TIM2_CH1(remap) |
| GP20 | PA9 | USART1_TX(既定) / TIM1_CH2 |
| GP21 | PA11 | **USB_DM** / TIM1_CH4 |

未接続: probe 側 GP8, GP9, GP16, GP17, GP18, GP22–GP29（GP19 は基板の PSRAM CS）。
L103 側 PA0–PA7, PB0–PB2, PB9–PB14, PC14/15, PD0/PD1。

**GP2 = NRST** は、空きチャンネルを 1 本ずつプルダウンして DM が消えるのを探して特定した。放置時に High に
浮いているのは L103 内部プルアップ。`target.control reset --mode pin` を有効にしたところ、リセット後の
`RCC_RSTSCKR` が `0x0c000000`（PINRSTF + PORRSTF）になり、外部リセットとして成立している。
**予約ピンでも起動時に Hi-Z にすること**: reserved mask に入れただけだと RP2 既定のプルダウンが残り、
ターゲットをリセットに握ったままになる（実際に一度踏んだ）。

### この配線で確認できること / できないこと

確認済み:

- **デバッグ一式** — attach / halt / resume / メモリ read・write / フラッシュ消去・書き込み・CRC verify /
  ndmreset / **ピンリセット**。
- **コンソール** — `Serial1.setPins(PB6, PB7)` で USART1 をリマップすると、PB6/PB7 は probe の GP13/GP12 に落ち、
  これは RP2350 の **UART0 のペア**。つまり両側ハードウェア UART で繋がる。実測で
  `l103_console READY` と tick を fixture.uart 経由で受信済み。
- **デジタル I/O 13 本**（両方向）。配線マップ自体がその証明。EXTI の刺激にも使える。

配線上は可能（未実施）:

- **PWM / タイマ**: TIM1_CH1–CH4 が PA8/PA9/PA10/PA11 ＝ GP7/GP20/GP6/GP21 に揃っている。TIM4_CH1–CH3 も
  PB6/PB7/PB8。probe 側は GPIO ポーリングでしか測れない（Pico の `fixture.capture` は未実装）。
- **I2C1** (PB6 SCL / PB7 SDA) ＝ GP13/GP12 は RP2 の **I2C0 のペア**でもある。ただしコンソールと同じ線なので排他。
  プルアップの有無は未確認。
- **SPI1 リマップ**（PA15 NSS / PB3 SCK / PB4 MISO / PB5 MOSI ＝ GP15/GP4/GP14/GP3）。probe 側は RP2 の
  SPI0 のピン役割と噛み合わないのでビットバンになる。

できない:

- **ADC** — ADC 入力（PA0–PA7, PB0/PB1）が 1 本も来ていない。
- **USB デバイス** — PA11/PA12 は USB DM/DP だが probe の GPIO に繋がっているだけで、USB として使うなら
  probe 側は入力のまま触らないこと。
- **フル smoke 実行** — 配線ではなくクライアント側の問題。`program_image()` の preflight が X03x/V003 の
  chip id しか知らず L103 を弾く。ここに L103 を足すのが次の一手（`DMCHIPID 0x20000410`、ESIG は
  `0x1ffff7e0` = 64 KiB）。

## 書き込みと USB bind の実際

- udev rule を入れた後は **picotool が WSL から使える**。ただし **複数の RP2 を同時に列挙すると picotool 2.3.0 は
  segfault する**ので、`--bus N --address M` で必ず 1 台を指定する。
- 再書き込みは `scratchpad/picoflash.sh <busid> <by-id node> <uf2> <BOOTSEL VID:PID>`（1200 bps touch → BOOTSEL →
  picotool → アプリ復帰 → attach）。
- **usbipd の bind は 4 つの identity すべてで取得済み**（各機のアプリ側と BOOTSEL 側）。bind は永続するので、
  以後の書き換えに管理者権限は要らない（2026-09-23 実測）。attach は非管理者で可。

## 未了

- `setMinHalfNs(500)` を外して、どこまで速くできるか測り直す（待機レベルの問題が解けた今なら 100 ns も通るかもしれない）。
- L103 のコンソール結線（UART）。今は RAM のカウンタでしか実行を確認できない。
- probe 起動直後の 1 回目の attach がまれに失敗する。2 回目以降は通る。
- ordinary SWD を OEP の service にするか決める（今は frame + survey sketch まで。`target.control` は CH32 DM 専用）。
- V003 を決めた pinout で繋いだ後のフルテスト（GPIO / UART / I2C / SPI / ADC / reset）。配線を決める際は、
  PIO を使う日のために **SWD/UART/SPI の各組を連番ピンに**寄せておくと後が楽。
- Pico の `fixture.capture`（PIO + DMA なら P4 級が狙えるが、配線確定まで着手しない）。
