# 参照の実装の値と限界（implementation limits）

[English](implementation-limits.md)

状態: **この実装の文書**（OEP の仕様ではない）。OEP の仕様（oep-spec）が probe に選び方を任せた値と、この実装（OpenEmbeddedProbe の
ライブラリと `examples/Firmware/OepProbe`）が platform ごとに持つ限界を、1 か所にまとめる。仕様の規範はこの文書に依らない: 別の probe は
別の値を選んでよく、host はここの値に頼らない（仕様の待ちと宣言だけに頼る）。

2026-10-07 の仕様の規則の見直し（oep-spec の `docs/v1-rule-review-2026-10-07.ja.md` §2、§4）で、仕様から外れて実装の値になったものを
ここに移した。値は main の 5ad85ce（0.0.28 の後）のもの。firmware がその見直しに追いつくまで、見直しで外れた規則（スロットの錠、
boot_reset、bind の mode、応答の ignored など）に沿った動きが残る（§2.6）。

## 1. どの platform にも共通

### 1.1 宣言する値

| 値 | この実装 | 仕様 |
|---|---|---|
| max_op_ms（fn 0 の describe） | 10000 ms | 1〜600000 ms、値は probe が決める（core §7.5） |
| restart_max_ms（`oep.probe.restart`） | ESP32-P4 3000 ms、classic ESP32 1500 ms、RP2040 / RP2350 2000 ms | probe が決める（oep-if-restart §1） |
| lease の既定（open の lease_ms 0） | 3000 ms | 1000〜60000 ms（core §6.4） |
| 送り直しの表 | 8 件、1 件の応答は 1029 byte まで（それより大きい応答の送り直しは result_lost） | max_inflight 件以上（core §5.2） |
| コンソールの送りの列 | ストリームごとに 256 byte | 大きさは probe が決める（oep-if-console §2） |
| model、chip | チップの名前をハイフンなしの小文字で（`esp32p4`、`esp32`、`rp2040`、`rp2350`）、型番とリビジョン（例 `esp32p4 v1.3`） | 任意の自由な文字列（core §7.5） |
| unit_id | チップの固有の番号を小文字の 16 進で | core §7.5 |

### 1.2 線の時間と回数

仕様（oep-if-debug §1〜§4、2026-10-07）は、attach、scan、riscv-dm の reset を max_op_ms のうちに答えることだけを求め、host はその
引数の時間を max_op_ms として待つ。この実装の実際の値:

| 値 | この実装 | 意味 |
|---|---|---|
| 1 つの要求の中の線の再試行 | 200 ms | 遅い速さでの再試行を含む。使い切れば status line。それだけでは線切れにしない |
| 線切れ | 実時間で 1000 ms | 線からの応答の無い失敗が続き、その間に成功が無い。リセットの線を保つ間と、それを解いてからの 1000 ms は数えない |
| attach の予算 | 1000 ms | 速さの探索と再試行。reset TLV の hold_ms と、下の DM の待ちは含まない |
| scan の予算 | 500 ms | 要求が届いてからこれより後に組を始めない（少なくとも 1 組は試す）。1 組は attach の予算で抑える |
| scan の 1 回の組 | 多くても 255 | tried は u8 |
| リセットの線を離した後の DM の待ち | 700 ms | DMSTATUS を最も遅い速さで読み直す。attach の reset TLV と riscv-dm の reset の op。線の再試行に数えない |
| 高水準の op の中の DM の待ち | 1 つの待ちにつき 100 ms | abstractcs.busy、allhalted、allresumeack |
| reset の op の hart の待ち | 1 回の手順につき 100 ms、手順のやり直しは 1 回まで | |
| DMI の busy | 100 回まで再試行、その後 status wait | |
| SWD の WAIT | 100 回まで再試行、その後 status wait | |
| arm-adi の TAR の書き直し | 1 KiB の境界ごと | ADI が自動の増加を保証する範囲 |
| read_block / write_block の max_length | max_frame − 24 以下 | 要求と応答が max_frame に収まる |
| コンソールの DMSTATUS の確かめ | 20 ms 以下ごと | 仕様（console §3、2026-10-07）は、要求に答えた後、次のコンソールの読みの前に DMSTATUS を読むことを求める |

この実装の attach の応答の待ちの上限は、1000 ms + hold_ms + 700 ms（max_op_ms の 10000 ms より短い）。

### 1.3 search_retries の数え方

attach の応答の search_retries（仕様では「診断用、数え方は実装が決める」）は、立ち上げ（wake / 設定の手順、速さを選ぶこと、それを
確かめること）の各段の最初の試しを超えて要った試しの数で、次のそれぞれを 1 と数える: やり直した確かめの読み出し 1 つ、または書き込みと
その読み返し 1 つ。やり直したパス（1 つの速さでの読み出しか書き込みの往復のまとまり）。選んだ速さで書き込みの確かめが失敗した後の、遅い
速さへの切り替え。立ち上げ全体のやり直し。wake のある線では、答えのあった wake より前の、答えの無かった wake。速い速さで失敗して速い速さの
探索を終わらせた読み出しは数えない。0xFFFF で止まる。立ち上げが行われた attach（新しい connection、reset TLV 付き、max_speed のために
速さを下げた既存の connection）でだけ送る。

### 1.4 dmi の要求ごとの線の確かめ

線によっては、hart の状態が変わった後に線が 0.7〜2 ms 落ち、その間、書き込みは消え、読み出しは前の値か全部 1 を返す（エラーにならない）。
この実装は、dmi の要求ごとに、手順の前と後に線を 1 度ずつ見る（DMSTATUS がモジュールのもので authenticated が立ち、DMCONTROL の bit 7 が
0: 落ちた線は全部 1 か前の値を両方に返すので両立しない）。間に PHY の立て直しが入っていないことも数で見る。確かめに通らない要求は、
値を返さずに status line で答え、done は確かめが失敗する前に済んだ手順の数（その中の書き込みは行われたかもしれない）。高水準の op
（block、run、step、reset）は手順をまとまりに分け、まとまりごとに線を確かめる（DMCONTROL の dmactive と hart 0 も見る）。仕様が求めるのは、
線の失敗を見つけた要求を success で返さないことと、書き込みを繰り返さないことだけ（oep-if-debug §2）。

### 1.5 そのほか

- **生のバイトの遅れ**: シリアルの口の受け方（transports §4）により、0x00 の後に来た生のバイトは、次の 0x00 が来るか入力が 200 ms 途切れた
  ときに初めて結んだ流れに届く。0x00 を含む二進の流れは、0x00 ごとに最長 200 ms 遅れる。
- **SDI と DMDATA の限界**: この 2 つのコンソールの方式は枠に番号を持たないので、線の書き込みが失われたこと（DATA0 の「受け取った」の
  0 が届かない、答えが届かない）を、probe も target も見分けられない。同じ枠が 2 回届くこと、DMDATA では答えに載せた入力の byte が
  失われることがある。重複も欠落も許されないなら dmseq を使う。
- **開いても再起動しない**: どの transport でも、開閉と DTR / RTS で probe は再起動しない（USB-Serial/JTAG の DTR / RTS のリセットは
  切ってある）。

## 2. 起動と更新

### 2.1 boot guard（止まったままにしない）

- RP2040 / RP2350: ハードウェアの watchdog（3000 ms）を、loop() が回っている間だけタイマー（250 ms ごと）で餌をやる。loop() が
  max_op_ms + 5000 ms（15000 ms）回らなければリセットする。panic と HardFault は、device を bus から外してからチップをリセットする。
- ESP32 / ESP32-P4: panic と割り込みの watchdog（300 ms）はチップをリセットする（core の sdkconfig）。loop() は task watchdog
  （15000 ms）の下で走る。
- **safe boot**: 落ちた起動（RP2: 動いている記録のまま watchdog でリセット。ESP: panic か watchdog）が、前の起動から 30 秒以内に来たら
  1 つ数える。3 回続いたら、その 1 回の起動は safe boot: 保存した設定の at boot のスロットの attach と、それに乗るコンソールを始めない。
  30 秒動いた起動、firmware が自分でした再起動、ほかのリセット（電源投入、リセットのピン）は数を 0 に戻す。仕様（oep-if-probe-config §3.1、
  2026-10-07）は、at boot のスロットを「試せるときに」attach するとし、試さなかったことは slot_state 1 と last_try_at_ns（全ビット 1）で見える。
- **USB の門**: 自分の USB の device を transport にする probe は、host が device を 1000 ms 構成したままになってから、host が居なければ
  起動から 5000 ms 後に、at boot の attach を始める。

### 2.2 前の起動の終わり方（firmware の文字列）

fn 0 の describe の firmware は、版の後ろに、前の起動を何が終わらせたかを括弧で付ける: `<version> (<note>)`。note は
`<reset> at <n> s`（reset は panic、task-wdt、int-wdt、wdt、brownout、usb、jtag、reset-pin、cpu-lockup、software（probe がしていないもの）、
other。RP2 は wdt）か、`update to <slot> did not reach setup: <reset>`（更新した image が setup() に届かなかった）。電源投入と probe が自分で
した再起動の後は何も付けない。

- 版を完全一致で比べる道具は、最初の空白から後ろを外して比べる。
- describe は宣言だけで、起動ごとに変わる情報を入れる所ではない（core §7.3）。2026-10-07 の仕様の見直し（Q4）で、この印は firmware の
  文字列から外し、見方をこの文書に置くことにした（firmware の変更はこれから）。

### 2.3 DFU で更新した image（ESP32-P4）

DFU で書いた image は、起動したときに確定する（試しの期間は無い）。arduino-esp32 3.3.x の initArduino が setup() の前に image を valid に
する（esp_ota_mark_app_valid_cancel_rollback）ので、その後の panic、watchdog、brownout、host の列挙し直しで前の image に戻ることはない。
bootloader がまだ前に戻しうるのは、新しい image が自分の確かめに通らないときと、core が確定する前にリセットが来たときだけで、そのときは
§2.2 の `update to <slot> did not reach setup: <reset>` が付く。壊れた image は ESP32-P4 の USB-Serial/JTAG と esptool で戻す。classic ESP32
と RP2040 / RP2350 には、動いている firmware からの更新は無い。

### 2.4 restart（`oep.probe.restart`）

restart の応答の後、probe は応答を送り終えてから再起動する（ESP: esp_restart。RP2: watchdog のリセット、60 ms 後）。target は reset せず、
止めていた hart は止めたまま。restart_max_ms は §1.1。

### 2.5 保存

設定は ESP32 / ESP32-P4 では NVS、RP2040 / RP2350 では flash の最後の sector に、丸ごと置き換えで保存する。

### 2.6 見直しに追いつくまでの動き（2026-10-07 の時点）

firmware はまだ次のものを持つ（仕様からは外れた。次の版で外す）:

- **boot_reset**（スロットのリセットでのやり直し）: at boot のスロットの自動の attach が線の応答をまったく得られなかったら、ロックが一度も
  取られていない起動の間だけ、スロットごとに 1 回、label `nrst` の channel を 20 ms 引いて attach をやり直す。
- スロットの錠、bind の mode（last-reset、manual、mixed。mixed の行は 128 byte か 100 ms の静けさで閉じる）、応答の ignored、corr_reused、
  list の prefix、port_speed の port と idle_ms、i2c-target の mode と arm_rx / reset、spi-target の reset、gpio の drive の kind と mode 7。

## 3. 線の PHY ごと

### 3.1 RVSWD / SWIO

- **立て直し**: 線が 300 µs 以上休んだ後の最初のフレームの前に DMSTATUS を読む。失敗したか、authenticated の立った「見つかった」で
  なければ、設定の組（DMI 0x7E、0x7D に 0x5AA50400）を 2 回送り（wake のパターンは無し）、DMSTATUS をもう一度読む。connection の上では
  wake のパターンを送らない（target そのものをリセットしうる。仕様 oep-if-debug §2 の、target の状態を変えない再試行）。
- **dmactive の後**: dmactive を書いた直後に、同じ速さで設定の組をもう 2 回書く。
- **受け入れ**: 速さは DMSTATUS を続けて 1000 回読んで同じ値とパリティが得られたら受け入れ、書き込みは scratch のレジスタ（PROGBUF0）への
  書き込みと読み戻しを 256 回往復して一致したら受け入れる。速さの探索は最も遅い速さの読みから始め、速い方へ進める。
- **ほかのデバッガ**は、つなぐときに状態の問い合わせの長い形（ビットの区切り 85 個）を送ることがある。この実装は送らない。取り込んだ
  波形でそれを誤りと読まない。

### 3.2 SWIO（classic ESP32）

- ビットの時間は CPU のサイクルで数える: 1 は low 262.5 ns、0 は low 862.5 ns、high は 262.5 ns（いま測った 1 つの target の系列で動く
  範囲の中。target の限界は分かっていない）。0 の区切りの速さを min_clock_hz に宣言する。
- low の駆動をやめた後と 0 を読んだ後に、30 ns 以下の充電のパルスで線を high に駆動する。
- attach の最初のフレームの前に、線をプルアップだけにして 2 ms おき、low に読めれば target は無いとする。
- フレームの間、割り込みを止める（多くても 200 µs）。

### 3.3 SWD（RP2040 / RP2350）

- 速さは読んで選べないので、`min(max_speed, describe の max_clock_hz)` から始める。最も遅い速さは 10 kHz（max_speed の無い scan も
  これで試す）。
- 線の再試行では line reset と dormant からの wake をやり直す。

## 4. platform ごと

### 4.1 classic ESP32

- **キャプチャの窓とデバッグの線**: logic の sampler は、窓の間（即時で最大 164 ms、トリガの探索は 1 回の区切りが最大 250 ms）、core 0 で
  割り込みを止めて GPIO を読み続ける。同じ周辺のバスを使う SWIO のフレームは、その間ずれうる（SWIO にはパリティが無く、DMI の書き込みは
  読み戻されないので、線の上で分からない）。この実装は、要求の SWIO のフレームを窓の後に回す（要求のフレームは窓で分けない。トリガの
  探索の区切りの間に待っている要求があれば 5 ms の番を譲る）。止めるのは窓の長さ以下。
- **コンソールの読み**: 今の firmware は、窓の間もコンソールの DMSTATUS と DATA0 を読む（試験用の OEP_SWIO_PAUSE_CONSOLE=1 で
  ビルドしたときだけ止める）。2026-10-07 の仕様（console §3）は、読みを止めてもどの規則にも反しない形になったので、窓の間は止める
  方へ直す（dmseq の target は長い待ち（1 s 以上）のうちに戻る）。
- fixture UART の受信の割り込みは loop() の core（sampler と別）で、FIFO の閾値は 32 byte。速さは 2000000 bps まで。

### 4.2 ESP32-P4

- **USB の device のスタック（TinyUSB の DWC2）**: EspUsbDevice 2.5.1 が使う TinyUSB の DWC2 のドライバは、control 転送の status stage の間に
  次の SETUP が届くと panic する（upstream の不具合）。P4 では panic はチップのリセットになるので、host には device が bus から外れて列挙し
  直し、boot_id が変わったように見える（firmware の文字列には `panic at <n> s`、§2.2）。セッション、資源、保存していない設定は失われる。
  host は boot_id の変化で知り、開き直す。直った版が出たら上げる。
- fixture UART の受信の割り込みは core 0（loop() は core 1）で、FIFO の閾値は 32 byte、ドライバの受信のバッファは 16 KiB。2000000 bps まで。
- i2c-target: ESP-IDF の slave ドライバ（v1）は受けた byte の数を知らせず、送信は区切りの無い流れなので、この実装は周辺の割り込みを
  自分で受け、トランザクションの byte を数え、TX FIFO を置き場から埋める（空なら 0xFF）。チップの FIFO の癖（読み出しの最後に余分な byte を
  出す、など）はこの実装が吸収する。stretch は 100000 µs まで（max_stretch_us）。

### 4.3 RP2040 / RP2350

- fixture UART: PL011 の overrun は、poll() で UARTRSR.OE を読んで lost（overflow）の印にする。FIFO の残りは 28 byte（2000000 bps で 140 µs）。
- 保存は flash の最後の sector（書く間、ほかの要求に答えない。max_op_ms のうち）。

## 5. host と変換器の側（この実装の利用者のために）

- **USB-UART の変換器**（UART bridge の口。CH340 の類）: 続けて流れる probe → host のバイトを、長い流れ（max_frame いっぱいの応答が
  続く書き込み、キャプチャの読み出し）で落とすことがある。短い確かめは通っても、長い転送で壊れる。測った変換器（CH340、FTDI 互換を名乗る
  CH552）では、921600 bps で 9 KiB ほどの書き込みのたびに max_frame いっぱいの応答が 1〜4 個壊れ、500000 bps はどちらでも壊れなかった。
  実用の速さは **500000 bps**（oep-spec の host 開発ガイド §17 の既定の上限）。CH340 は 115200 でも数秒まとめて 0〜10 % を落とすことがある。
  フロー制御が無いので、壊れたフレームは host が同じ corr で送り直す（probe の送り直しの表が二重の実行を防ぐ）。
- **Linux の cdc_acm**: probe → host の burst が約 8 KiB を超えると、エラーなしに黙って失う。oep-client-python は、同時に待つ応答の量を
  6 KiB、購読の min_bytes を 2 KiB に抑える。
- 変換器ごとに通った port_speed の速さの記録は、oep-spec の記録（`docs/link-measurements.ja.md`、`docs/uart-speed-negotiation.ja.md`）にある。
