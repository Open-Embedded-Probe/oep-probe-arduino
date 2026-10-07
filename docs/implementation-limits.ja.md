# 参照の実装の値と限界（implementation limits）

[English](implementation-limits.md)

状態: **この実装の文書**（OEP の仕様ではない）。OEP の仕様（oep-spec）が probe に選び方を任せた値と、この実装（OpenEmbeddedProbe の
ライブラリと `examples/Firmware/OepProbe`）が platform ごとに持つ限界を、1 か所にまとめる。仕様の規範はこの文書に依らない: 別の probe は
別の値を選んでよく、host はここの値に頼らない（仕様の待ちと宣言だけに頼る）。

2026-10-07 の仕様の規則の見直し（oep-spec の `docs/v1-rule-review-2026-10-07.ja.md` §2、§4）で、仕様から外れて実装の値になったものを
ここに移した。firmware は oep-spec 0f455a0 の規則に沿う（0.0.29 の開発版から）。その後の外部の見直しの再確認（2026-10-07）の変更も含めて、今は oep-spec 30b2b36（wifi の項目、TCP の見つけ方、unset の len はキーの長さを含む）に沿う。見直しで外した動き（boot_reset など）の記録は §2.6。
TCP と Wi-Fi（port、mDNS、Wi-Fi の設定の提案、測った値）は §6。

## 1. どの platform にも共通

### 1.1 宣言する値

| 値 | この実装 | 仕様 |
|---|---|---|
| max_op_ms（fn 0 の describe） | 10000 ms | 1〜600000 ms、値は probe が決める（core §7.5） |
| restart_max_ms（`oep.probe.restart`） | ESP32-P4 3000 ms、classic ESP32 15000 ms（Wi-Fi の build、§6.1。`-DOEP_WIFI=0` で 1500 ms）、RP2040 / RP2350 2000 ms | probe が決める（oep-if-restart §1） |
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
| コンソールの DMSTATUS | その connection の riscv-dm の要求（と線の attach）に答えた後、次の DATA0 の読みの前。ほかに 20 ms ごと | 仕様（console §3）が求めるのは前者だけ。20 ms ごとの読みは、止まった hart が走り出したことと target の自分の再起動（havereset）を見るためのこの実装の間隔 |

この実装の attach の応答の待ちの上限は、1000 ms + hold_ms + 700 ms（max_op_ms の 10000 ms より短い）。値はライブラリの
`src/OepLimits.h`（`oep::limits`）にまとめてある。

**target の状態を変えない再試行（仕様 oep-if-debug §2）**: connection がある間、線の立て直しは設定の組（RVSWD は DMI 0x7E / 0x7D、SWIO は
同じ組と、DMCONTROL が dmactive を 0 と読むときの dmactive）だけを送る。target そのものをリセットしうる RVSWD の wake のパターンは、
connection の上では attach（新しい connection の立ち上げと、答えない既存の connection の立て直し、reset TLV）と riscv-dm の reset の op の
中でだけ送る（`DmiPhy::holdWakes` / `DmiPhy::WakeScope`）。コンソールの読みの中の立て直しは wake を送らないので、wake でしか戻らない線は
線切れ（1000 ms）で閉じ、host の attach で戻る。SWD の line reset と dormant からの wake は target の状態を変えないので、この制限の外。

**書き込みを繰り返さない（仕様 oep-if-debug §2）**: DMI の書き込みは PHY が一度だけ送る（線の再試行は読み出しだけ）。host の dmi の手順は
やり直さず、失敗したら done をそれより前の手順の数にして答える。write_block の語の書き込み（target の記憶への store）は一度だけ。
SWD の書き込みの転送をやり直すのは、ACK が無かったとき（target は書き込みを受けていない）と WAIT のときだけ。高水準の op の中で、
線の落ちを見たまとまり（§1.4）をやり直すのは、probe が選んだ値をレジスタに置く設定（a0 / a1、program buffer、DATA1 の番地、dcsr、
run の host のレジスタと dpc、abstractauto と DATA0 / DATA1 の書き戻し）だけで、同じ値をもう一度置く。target の記憶への書き込みと、
一度で意味が変わる書き込み（resumereq、ndmreset）はやり直さない。

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

### 1.5 DATA0 / DATA1 の書き戻し

コンソールの方式は DATA0 / DATA1 を target との郵便受けに使う。probe は、自分の高水準の op の抽象コマンドで使った DATA0 / DATA1 を
応答の前に戻す（仕様 oep-if-debug §4）: read_block / write_block、step、reset の確認、attach の dpc の読み。run は、準備（dcsr、host の
レジスタ、dpc）の前に DATA0 / DATA1 を覚え、hart を走らせる前に書き戻し、止まった後の dpc と値の読みの後にもう一度戻す。準備が
失敗したら書き戻してから stopped 3（走らせなかった）で答える。

### 1.6 そのほか

- **生のバイトの遅れ**: シリアルの口の受け方（transports §4）により、0x00 の後に来た生のバイトは、次の 0x00 が来るか入力が 200 ms 途切れた
  ときに初めて結んだ流れに届く。0x00 を含む二進の流れは、0x00 ごとに最長 200 ms 遅れる。
- **SDI と DMDATA の限界**: この 2 つのコンソールの方式は枠に番号を持たないので、線の書き込みが失われたこと（DATA0 の「受け取った」の
  0 が届かない、答えが届かない）を、probe も target も見分けられない。同じ枠が 2 回届くこと、DMDATA では答えに載せた入力の byte が
  失われることがある。重複も欠落も許されないなら dmseq を使う。
- **max_frame はどの経路も同じ**: 1 つの `Endpoint` の max_frame / window / max_inflight は、シリアルの口、USB、TCP の接続のすべてで同じ値
  （`Limits`）。describe の TLV と max_length はその値から決めるので、どの経路の max_frame にも収まる（core §7.3、§7.4）。経路ごとに
  違う max_frame は持たない。wifi の項目を持つ probe はどの経路でも max_frame 112（`wifi_min_max_frame`）以上を答える
  （probe.config §1.4: いちばん長い wifi の項目 1 つの set が 10 + 3 + 3 + 32 + 64 = 112 byte）。`ProbeConfig::setWifi` は
  `Endpoint` の max_frame が 112 未満なら false を返して wifi の項目を宣言せず、Wi-Fi を持つ sketch（classic ESP32 の firmware の
  `kMaxFrame` 512、`03.Transports/WifiTcp` の 1024）は static_assert で確かめる。
- **フレームは途中で止めずに書く**: 応答と通知は 1 つの `poll()` の中で 1 フレームずつ丸ごと書き、フレームの途中でほかの仕事をしない
  （transports §2）。ESP32-P4 の vendor bulk はフレームを丸ごと 1 つのバッファに入れ、入らなければ丸ごと捨てる。止まりうるのは、host が
  読まなくなって経路のバッファが埋まったときだけである。
- **開いても再起動しない**: どの transport でも、開閉と DTR / RTS で probe は再起動しない（USB-Serial/JTAG の DTR / RTS のリセットは
  切ってある）。

## 2. 起動と更新

### 2.1 boot guard（止まったままにしない）

- RP2040 / RP2350: ハードウェアの watchdog（3000 ms）を、loop() が回っている間だけタイマー（250 ms ごと）で餌をやる。loop() が
  max_op_ms + 5000 ms（15000 ms）回らなければリセットする。panic と HardFault は、device を bus から外してからチップをリセットする。
- ESP32 / ESP32-P4: panic と割り込みの watchdog（300 ms）はチップをリセットする（core の sdkconfig）。loop() は task watchdog
  （15000 ms）の下で走る。
- **safe boot**: firmware 自身が落ちて終わった起動（RP2: `BootGuard::crashed()` が印を付けた watchdog のリセット。ESP: panic、割り込みの
  watchdog、task watchdog、CPU lockup）が、前の起動から 30 秒以内に来たら 1 つ数える。3 回続いたら、その 1 回の起動は safe boot: 保存した設定の at boot のスロットの attach と、それに乗るコンソールを始めない。
  30 秒動いた起動、firmware が自分でした再起動、ほかのリセット（電源投入、リセットのピン、外からのリセット）は数を 0 に戻す。
  RP2 では再起動はどれも watchdog のリセットで、印の無いもの（picotool、boot ROM、debugger）は外からの再起動として扱う。割り込みを
  切ったまま止まった RP2（餌のタイマーも止まり、印を付ける前に watchdog が落とす）は、外からの再起動と区別できない（数えず、印も付けない）。仕様（oep-if-probe-config §3.1、
  2026-10-07）は、at boot のスロットを「試せるときに」attach するとし、試さなかったことは slot_state 1 と last_try_at_ns（全ビット 1）で見える。
- **USB の門**: 自分の USB の device を transport にする probe は、host が device を 1000 ms 構成したままになってから、host が居なければ
  起動から 5000 ms 後に、at boot の attach を始める。

### 2.2 前の起動の終わり方（firmware の文字列、この実装の動き）

fn 0 の describe の firmware は、版の後ろに、前の起動を何が終わらせたかを括弧で付ける: `<version> (<note>)`。note は
`<reset> at <n> s` か、`update to <slot> did not reach setup: <reset>`（更新した image が setup() に届かなかった）。`<reset> at <n> s` は
firmware が意図しなかったリセット（落ちた、止まった）と brownout だけ: ESP は panic、task-wdt、int-wdt、cpu-lockup、brownout、RP2 は crash
（`BootGuard::crashed()` が印を付けたもの: panic、HardFault、loop() の停止）。電源投入、probe が自分でした再起動、外からのリセットの後は
何も付けない。外からのリセット: リセットのピン（esptool の DTR / RTS を含む）、USB-Serial/JTAG のリセット、debugger の JTAG リセット、
probe がしていない software の再起動、panic の無い RTC watchdog のリセット（esptool の watchdog リセット）、RP2 の印の無い watchdog
のリセット（`picotool load -x` などの再起動、boot ROM、debugger）。

- 版を完全一致で比べる道具は、最初の空白から後ろを外して比べる。
- describe は宣言だけで（core §7.3）、仕様はこの印を定めない（見直しの Q4）。この実装は利用者の判断で印を firmware の文字列に
  残す（実装の動きで、仕様の規則ではない）。host は firmware を自由な文字列として扱い、印に頼らない。

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

### 2.6 見直しで外した動きの記録（0.0.28 まで）

oep-spec 0f455a0 の規則に合わせて 0.0.29 の開発版で外した。どれも今の firmware には無い。

- **boot_reset**（スロットのリセットでのやり直し）: at boot のスロットの自動の attach が線の応答をまったく得られなかったら、ロックが一度も
  取られていない起動の間だけ、スロットごとに 1 回、label `nrst` の channel を 20 ms 引いて attach をやり直した。
- スロットの錠（target_id の mask / value）と slot_state 2 / 3、bind の mode（last-reset、manual、mixed。mixed の行は 128 byte か 100 ms の
  静けさで閉じた）と、セッションの後に最後の host の reset から再開すること、応答の ignored、corr_reused（送り直しの表の CRC-32）、
  list の prefix、describe の reserved / profile / resets_on_open / discoverable / implementation、port_speed の port と idle_ms と壊れの数え、
  i2c-target の mode と arm_rx / reset、spi-target の reset、gpio の drive の kind と mode 7、uart の status の configured、コンソールの
  send_queue の宣言、捕捉の timing / rate_accuracy と宣言（rate_list など）、riscv-dm の reset の method と attempts、
  OEP_SWIO_PAUSE_CONSOLE の試験用の仕掛け（今は窓の間いつも止める、§4.1）。

## 3. 線の PHY ごと

### 3.1 RVSWD / SWIO

- **立て直し**: 線が 300 µs 以上休んだ後の最初のフレームの前に DMSTATUS を読む。失敗したか、authenticated の立った「見つかった」で
  なければ、設定の組（DMI 0x7E、0x7D に 0x5AA50400）を 2 回送り（wake のパターンは無し）、DMSTATUS をもう一度読む。それでも戻らない
  線に wake のパターンを送るのは、connection が無いときと、attach と reset の op の中だけ（§1.2 の「target の状態を変えない再試行」）。
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
  読み戻されないので、線の上で分からない）。
- **規則: 即時のキャプチャの窓と、SWIO の線の行き来（要求もコンソールも）は同時に起きない。** 即時（trigger type 0）の窓がサンプルして
  いる間、SWIO のフレームは 1 つも出ない: 要求（riscv-dm、線の op）は窓が終わるのを待ち、コンソールは DMSTATUS も DATA0 も読まない（線の
  番を断られ、窓が閉じた後の poll で読む）。窓は進行中の要求が終わるのを待ってから開く。即時の窓の長さは samples ÷ 実際の rate で、
  いっぱいの 65408 サンプルなら 2 MHz で 33 ms、1 MHz で 65 ms、400 kHz で 164 ms。だから**キャプチャを始めた後に送ったコンソールの
  コマンドやデバッグのリセットは、即時のキャプチャには写らない**（窓が閉じてから target に届く）。
- **トリガの探索**: 区切り（1 回が最大 250 ms）の中は同じく線を通さないが、その中で線に番を譲る: 要求が線を待てば次のサンプルで
  （区切りごとに何度でも）、コンソールの読みが断られれば区切りごとに 1 回、最大 5 ms（`kWireTurnMs`）。番の間も sampler は読み続け、
  フレームはそれと 1 つずつ交互に通る（フレームの間は読まず、その後で遅れを取り戻す: そのサンプルは遅れ、区画に入れば slipped）。
  コンソールは送るものを target に渡し終えたら番を早く終える（target はその後、線の行き来の無いところで動く）。区切りと区切りの間の
  すき間（約 1 ms、割り込みを戻して core 0 のタスクを回す）には線を取らせない（そこで送ったコマンドで target が動くと、誰もサンプル
  していない）。トリガが立った後の区画の残りは排他（番の中で始まっていた要求はフレームごとに続く）。arm の後で target に何かをさせて、
  それを写したい host は、トリガを使う: arm してからコマンドかリセットを送ると、それは番の中で target に届き、それが起こす出来事で
  トリガが立ち、キャプチャに写る。区切りの最初のサンプルからトリガを見る（エッジは 2 つ目から）: プリトリガの分がまだ溜まっていない
  ときは区画が短く、trigger_index が小さい（capture §3.3）。見逃すのは、すき間に来た出来事と、フレームの間（1 つ約 60〜150 µs）に
  始まって終わるパルス。
- 経緯: 0.0.29 の開発版 51360ea は番を区切りと区切りの間に置き、その間は誰もサンプルしなかった。コマンドやリセットはその番の中で
  target に届き、target もその中で動くので、立ち下がりのトリガが立たなかった（V003 の台、8bcecca: ソフトウェアのリセットの印の low は
  約 0.57 ms で 5 ms の番に収まり、reset_probe の "no capture for the software reset"。TOGGLE の 20 回も同じ）。トリガが立たないままの
  キャプチャは state 2 のままで、次の configure は unavailable cause 6 で断られる（capture §3.2 の表のとおり。samples の切り下げとは関係
  ない）。
- slipped は、本当に遅れたサンプル（割り込みや周辺のバスの待ち、番の中のフレーム）が区画に入ったときだけ示す。即時の窓と、トリガの後の
  区画の残りには、新しいフレームは入らない。
- コンソールの読みを窓の間止めることは、仕様（console §3）の規則に反しない（読む間隔を決めない）。target の側では、SDI / DMDATA は
  その間書き込みを待ち、dmseq の target は長い待ち（1 s 以上）のうちに戻る。
- 経緯: 0.0.29 の開発版 589acd5 は、窓の中でもフレームを 1 つずつ通した（sampler がフレームの間だけ読むのを止める）。classic の V003 の
  台では、コンソールの読みが窓ごとにフレームを入れるため、速く続く信号（pwm、tone、速い切り替え）が slipped で崩れ、ソフトウェアの
  リセットのキャプチャの 5 回に 1 回が空になり、試験の一式が 446 s から 1040 s に延びた。キャプチャの目的（正しいサンプル）を守るため、
  この排他の規則に戻した。
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

## 6. TCP と Wi-Fi（classic ESP32）

OEP を TCP で運ぶ（oep-spec transports §1、§2、§3）。待ち受ける port は probe が決める（この実装の値をここに書く）。見つけ方（DNS-SD の
`_oep._tcp`、TXT `unit_id`）は transports §3、Wi-Fi の項目は probe.config §1.4 が定める。
TCP の probe が自分を知らせるかは probe が選ぶ（transports §3）。この実装は知らせる。
TCP は信頼できる手元のネットワークか、認証したトンネルの中でだけ使う（OEP は認証を持たない、transports §1、security §1）。

### 6.1 経路

- **port は 7450**（`examples/Firmware/OepProbe` の `kTcpPort`、`03.Transports/WifiTcp` の `kPort`）。待ち受けの socket 1 つが fn 0 の describe の
  経路 1 つ（kind 6、interface 0xFF）。classic ESP32 の firmware では UART bridge が 0、TCP が 1。待ち受けは、ほかの経路をすべて足した後に
  足す（`Endpoint::addTcpListener`。後から足す経路は断られる: シリアルの口の index が describe の index のままであるため）。
- **同時の接続は 3 つ**（`TcpListener<3>`）。どの接続も別の経路で、confirm の transport TLV は 1 を返す。max_frame / window / max_inflight は
  接続ごとに同じ値（classic ESP32: 512 / 1024 / 2）。4 つ目の接続は受けてすぐ閉じる。
- 接続ごとに受けの 1 KiB と送りの 2 KiB のバッファ。1 回の poll の応答は flush でまとめて送る（長さと本体が 1 つの segment で出る）。送りの
  バッファがいっぱいで、socket が 2000 ms（`TcpSlot::kWriteWaitMs`）受け取らなければ、その接続は死んだとして閉じる（読まなくなった host に
  probe が止められない上限）。TCP keepalive: 10 s 黙ったら 5 s ごとに 3 回、答えが無ければ閉じる。
- max_frame を超える長さを読んだら、その接続を閉じる（transports §1）。フレームの途中の休みでは読み直さない（transports §2）。
- 接続が閉じても、セッション、ロック、購読、送り直しの表は残る（transports §3）。閉じた接続に送るはずの応答と通知は捨てる。同じ slot に
  来た次の接続は、前の接続の通知を受けない（読みかけのフレームも捨てる）。同じ session id の open が別の接続から来れば、lease を始め直し、
  通知はその接続に移る（core §6.2）。
- restart（`oep.probe.restart`）: 応答を送った後、待ち受けを閉じ（再起動する前の probe が新しい接続を受けないように）、送りのバッファを
  socket に渡し、100 ms 待ってから再起動する。**restart_max_ms は Wi-Fi の build で 15000**: 起動、接続、アドレスまでで（2026-10-07 の測りは、まだ接続の前に scan をしていたとき）、応答から
  新しい接続の confirm が答えるまで ATOM で 5.5〜6.9 s（10 回、2026-10-07）。その倍。設定の前の方の entry が
  つながらないと、1 つあたり最長 15 s 延びる（§6.3）。

### 6.2 見つけ方（mDNS、transports §3）

- host 名 `oep-<unit_id>.local`、service `_oep._tcp`（port 7450）、instance 名 `OEP <unit_id>`、TXT `unit_id=<unit_id>`。
  host は名指した unit_id の probe を、ほかの経路と同じく describe の unit_id で確かめる。
- アドレスは、ほかの経路（シリアルの口）から probe.config の state の wifi の TLV（§6.3）でも分かる。

### 6.3 Wi-Fi の設定（probe.config の wifi の項目、probe.config §1.4）

ビルドに認証情報は入れない。どのネットワークに入るかは probe の設定で、ほかの項目と同じく set / save / unset する。項目の形と規則は
oep-spec の probe.config §1.4、§3.3、§4（c2b8007 から）。この実装の値と動きは次のとおり。

| 何 | 値 |
|---|---|
| 項目 0x08 wifi | index(u8)、ssid_len(u8)、ssid、pass_len(u8)、passphrase。キーは index |
| describe 0x46 wifi_max | u8: entry の数の上限（この実装は 4） |
| state の TLV 0x01 wifi | state(u8: 0 切、1 接続中、2 接続、3 どれも失敗して待ち)、entry(u8: 使っている / 試している index、0xFF なし)、reason(u8: 0 なし、1 見つからない、2 認証、3 アドレスが来ない、4 そのほか)、rssi(i8、dBm)、ipv4(4 byte)。rssi と ipv4 は state 2 のときだけ、ほかは 0 |

- ssid は 1〜32 byte（0x00 を含むものは unsupported）。passphrase は無し（pass_len 0、開いたネットワーク）、8〜63 byte の 0x20〜0x7E、
  または 16 進の 64 文字。ほかは malformed。index は wifi_max 未満（ほかは unsupported、受け取ったままの項目の tag）。
- **passphrase は書くだけ**: get は passphrase を返さない（pass_len は、あれば 0xFF、無ければ 0、後ろに何も付けない）。set の pass_len 0xFF は
  「その index の今の passphrase のまま」（その index が無ければ malformed）: get の形をそのまま送り返しても変わらない。hash は passphrase の
  byte を含まず、passphrase が変わるたびに変わる乱数を含む（ロックなしの get から passphrase を推せないため）。state にも出ない。
- 保存は NVS（ほかの設定と同じ、暗号化しない）。flash を読める人には読める。
- 動き（時間はこの実装の選び方。仕様は間隔を probe に任せる、probe.config §1.4）: set か起動時の適用の 300 ms（`kApplyDelayMs`）後
  （set の応答が先に出る）に、entry を index の順にすべて試す（scan で飛ばさない: 隠した SSID は scan に出ないため）。1 つあたり最長 15 s
  （`kTryMs`。driver がネットワークが無い、認証の失敗と知らせればそこで次へ）。最初に IPv4 のアドレスが来たものを使う。全部だめなら 5 s
  （`kRetryMs`）待って初めからやり直す。つながった後に切れたら、すぐ初めからやり直す。使っている entry を変えるか消す set / unset は、応答を送ってから（300 ms 後）切って初めからやり直す。使っている entry が変わらない set では切らない（新しい並びは次にやり直すときに使う）。つながっていない間（試している、待っている）の変更は、300 ms 後に初めからやり直す。entry が 0 個なら
  radio を止める。modem sleep は切る（要求が次の beacon まで待たされないように）。Wi-Fi の driver 自身の保存は使わない。
- 失敗の reason は driver の理由を丸めたもの。passphrase も SSID もログに出さない（UART は OEP の経路でログを出さない、probe guide §3）。

### 6.4 測った値（M5Stack ATOM、2026-10-07、家庭の AP、host は WSL2 の NAT の後ろ）

- clock の往復: 中央値 8.7 ms、99 % 44 ms、最大 56 ms（20 s で 1558 回）。
- `oep linktest`（505 byte のフレーム、各 300）: in x1 26〜71 KB/s、in x2 103 KB/s、out 23〜31 KB/s、duplex 22〜28 KB/s。どれも壊れ 0、
  失い 0。UART の 115200（約 11 KB/s）より速い。
- 2 つ in flight で max_frame いっぱいの応答を続けると、ときどき 1〜2 s（最長 4 s）答えが止まってから続きが来る（Wi-Fi の再送。Windows から
  直接でも同じ）。答えは失われない。host の TCP の待ち（oep-client-python は 15 s）はこれを越える。modem sleep を切る前は ping でも 0.5 s
  を越えた。
- 2 つの接続: 一方が open、もう一方の open は locked、lock_state で持ち主が見える。接続が閉じてもロックは残り、別の接続から同じ id の
  open で続けられる。シリアルの口も同時に答える。
- 起動から接続まで: 約 4〜7 s（scan、接続、DHCP）。

### 6.5 限界

- **sampler の窓**: logic の sampler は窓の間（最大 164 ms / 250 ms）core 0 の割り込みを止める。Wi-Fi と lwIP の task も core 0 なので、その
  間 TCP は止まる（落ちはしない。hardware の試験でキャプチャは TCP でも通った）。
- build の大きさ: classic ESP32 の firmware は Wi-Fi で 1.07 MB（app の区画 1.31 MB の 81 %）、RAM 107 KB。`-DOEP_WIFI=0` で外せる
  （0.42 MB）。Wi-Fi の無い build に wifi の項目は無い（describe の items に出ない）。
- **ESP32-P4** は radio を持たない。arduino-esp32 には別のチップ（ESP32-C6 など）を SDIO で使う ESP-Hosted の道があるが、この firmware は
  使わない（`OepWifi.h` は SOC_WIFI_SUPPORTED のチップだけ）。手元の P4 のボードには co-processor が無く、試していない。
- **RP2040 / RP2350**（Pro Micro RP2350 を含む）: Wi-Fi なし。Pico W / Pico 2 W の CYW43 は使っていない。
- IPv4 だけ。TLS も認証も無い（OEP の外。要るならトンネルの中で使う）。
