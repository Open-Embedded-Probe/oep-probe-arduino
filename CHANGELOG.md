# Changelog / 変更履歴

## Unreleased
- (EN) Follows oep-spec f8bb2de (the external review re-check of 2026-10-07 and what followed it: 0f455a0..f8bb2de; the
  registry header and the test vectors synced, tools/sync_registry.sh with OEP_SPEC_REF=f8bb2de). Interface names are
  1 to 48 bytes (registry interface_name_max_bytes 48; `interfaceName` refuses a 49-byte name); resend_max is gone from
  the registry (nothing here used it). The vector "rvswd scan: count > 0 ignores skip" now answers the listed scan, and
  test_vectors runs it as written (the local exception for the stale vector removed). Checked, no change needed: one
  max_frame / window / max_inflight for every transport of an `Endpoint` (serial ports, USB, each TCP connection), so
  every describe TLV and max_length fit each transport's max_frame (core §7.3, §7.4); fn 0's describe always carries
  channels, and the firmware's channel numbers are 0 .. channels - 1 (40 classic ESP32, 55 ESP32-P4, 30 RP2040 /
  RP2350; a probe without channels writes 0); a frame is written whole within one poll() and never paused inside (the
  ESP32-P4's vendor bulk puts a frame in one buffer or drops it whole; transports §2); a closed transport ends nothing
  (only end, lease expiry and force do; transports §3); i2c-target's errors grows by 1 per failed write (a write both
  too long and dropped for a full queue counts 1); link sink takes up to max_frame - 12 bytes and source answers up to
  max_frame - 7. Host test test_core_conformance: a 48-byte name added and a 49-byte one refused, source at max_frame -
  7 and a sink of max_frame - 12 (a request of exactly max_frame). docs/implementation-limits §1.6 (EN / JA)
- (JA) oep-spec f8bb2de（2026-10-07 の外部の見直しの再確認と、その後: 0f455a0..f8bb2de。registry のヘッダと試験のベクタを
  tools/sync_registry.sh、OEP_SPEC_REF=f8bb2de で同期）に合わせました。インターフェースの名前は 1〜48 byte（registry の
  interface_name_max_bytes 48。`interfaceName` は 49 byte の名前を断る）。registry から resend_max が無くなった（ここでは使って
  いなかった）。ベクタ「rvswd scan: count > 0 ignores skip」が並べた scan の答えになったので、test_vectors はそのとおりに試す（古い
  ベクタのための例外を削除）。確かめて変更の要らなかったもの: 1 つの `Endpoint` の max_frame / window / max_inflight はどの経路
  （シリアルの口、USB、TCP の各接続）でも同じなので、describe の TLV と max_length はどの経路の max_frame にも収まる（core §7.3、
  §7.4）。fn 0 の describe は channels を必ず付け、firmware の channel の番号は 0〜channels − 1（classic ESP32 40、ESP32-P4 55、
  RP2040 / RP2350 30。channel の無い probe は 0）。フレームは 1 回の poll() の中で丸ごと書き、途中で止めない（ESP32-P4 の vendor
  bulk はフレームを 1 つのバッファに入れるか丸ごと捨てる。transports §2）。経路が閉じても何も終わらない（終わるのは end、lease
  切れ、force だけ。transports §3）。i2c-target の errors は失敗した書き込み 1 回につき 1（長すぎ、かつ列があふれた書き込みも 1）。
  link の sink は max_frame − 12 byte まで、source は max_frame − 7 byte まで。host の試験 test_core_conformance: 48 byte の名前が
  通り 49 byte が断られること、source の max_frame − 7 と、sink の max_frame − 12（ちょうど max_frame の要求）。
  docs/implementation-limits §1.6（EN / JA）
>>>>>>> 34708e8 (Follow oep-spec f8bb2de (external review re-check of 2026-10-07, 0f455a0..f8bb2de): registry and vectors synced (tools/sync_registry.sh, OEP_SPEC_REF=f8bb2de). Interface names 1 to 48 bytes (interface_name_max_bytes 48); resend_max gone from the registry (unused here); the vector "rvswd scan: count > 0 ignores skip" answers the listed scan, so test_vectors runs it as written (the local exception removed). Checked without change: one max_frame / window / max_inflight for every transport of an Endpoint (serial, USB, each TCP connection), so describe TLVs and max_length fit every transport's max_frame (core §7.3, §7.4); channels always in fn 0's describe and channel numbers 0 .. channels - 1 in the firmware; frames written whole within one poll(), never paused inside (transports §2); a closed transport ends no session, lock, subscription or resend table (transports §3); i2c-target errors +1 per failed write; link sink up to max_frame - 12, source max_frame - 7. Host test test_core_conformance: 48-byte name accepted, 49 refused; source at max_frame - 7, sink of max_frame - 12. docs/implementation-limits §1.6 (EN / JA); CHANGELOG (EN / JA))
- (EN) OEP over TCP (oep-spec transports §1-§3 as of af3d52b): `Endpoint::addTcpListener` adds a listening socket as one
  describe entry (kind 6, interface 0xFF) whose connection slots are each a transport of their own - confirm names the
  listener's entry, answers go back on the connection, notifications go to the subscriber's connection only (none to a
  later connection in that slot, none after it closed), a length over max_frame closes the connection
  (`Connection::drop`, `FrameReader::overlong`), no gap rule on TCP; a closed connection ends no session, lock,
  subscription or resend table, and an open with the same id from another connection moves the notifications there.
  No transport is added after a listener (the serial ports keep their describe index); kMaxTransports 8.
  `src/OepTcp.h` (ESP32, lwIP sockets, header-only): `TcpListener<N>` / `TcpSlot` - non-blocking accept, a connection
  beyond N closed, 2 KiB send buffer flushed per poll, 2000 ms write wait then close, TCP keepalive, `stopListening`
  before a restart. `src/OepWifi.h` (ESP32 with a radio, header-only): `WifiStation` joins the networks of the settings,
  announces `_oep._tcp` by mDNS (host `oep-<unit_id>`, instance `OEP <unit_id>`, TXT `unit_id`), modem sleep off.
  oep.probe.config: the wifi item, a proposal not yet in oep-spec (item 0x08 index ssid_len ssid pass_len passphrase,
  key index; describe 0x46 wifi_max 4; state TLV 0x01 state entry reason rssi ipv4): up to four networks tried in index
  order after a scan (those seen; all when none is), each up to 15 s, again after a loss; the passphrase is write-only
  (get shows pass_len 0xFF, a set with 0xFF keeps it, the hash carries a token instead of it); kMaxItems 800.
  Firmware/OepProbe classic ESP32: TCP port 7450, three connections, the wifi item (no credentials in the build;
  `-DOEP_WIFI=0` drops Wi-Fi), restart_max_ms 15000 with Wi-Fi (5.5-6.9 s measured). New example
  `03.Transports/WifiTcp`. Host tests: test_tcp (26 checks), test_config's wifi item (41 more). Checked on the M5Stack
  ATOM: settings over the serial port (a non-existent SSID at index 0, the real one at 1: joined index 1), mDNS
  answered, confirm / list / describe / clock / open / end / gpio / config over TCP, two connections (locked,
  lock_state), the session kept across a closed connection, serial alongside, `oep linktest` (no loss), oep-client-python
  tests/hw over TCP and over serial (all passed), restart over TCP. docs/implementation-limits §6 (EN stub), README
  (EN / JA), guides getting-started §6, writing-a-probe, boards (EN / JA)
- (JA) TCP で OEP（oep-spec af3d52b の transports §1〜§3）: `Endpoint::addTcpListener` は待ち受けの socket を describe の
  経路 1 つ（kind 6、interface 0xFF）として足し、その接続の slot はそれぞれ別の経路になります。confirm は待ち受けの index を返し、応答は
  その接続に、通知は subscribe した接続だけに送ります（同じ slot の次の接続にも、閉じた後にも送らない）。max_frame を超える長さは接続を
  閉じ（`Connection::drop`、`FrameReader::overlong`）、TCP には途切れの規則がありません。接続が閉じてもセッション、ロック、購読、送り直しの表は
  残り、別の接続から同じ id の open で通知はそこに移ります。待ち受けの後には経路を足せません（シリアルの口の index を保つ）。kMaxTransports 8。
  `src/OepTcp.h`（ESP32、lwIP の socket、header だけ）: `TcpListener<N>` / `TcpSlot`。止まらない accept、N を超える接続は閉じる、送りの 2 KiB を
  poll ごとに flush、2000 ms 書けなければ閉じる、TCP keepalive、再起動の前の `stopListening`。`src/OepWifi.h`（radio のある ESP32、header だけ）:
  `WifiStation` が設定のネットワークに入り、mDNS で `_oep._tcp`（host `oep-<unit_id>`、instance `OEP <unit_id>`、TXT `unit_id`）を名乗る。modem sleep は切る。
  oep.probe.config: wifi の項目（oep-spec にまだ無い提案: 項目 0x08 index ssid_len ssid pass_len passphrase、キー index。describe 0x46 wifi_max 4。
  state の TLV 0x01 state entry reason rssi ipv4）。4 つまでのネットワークを scan の後に index の順に試す（見えたものを。1 つも見えなければ全部）、
  1 つ最長 15 s、切れたらやり直す。passphrase は書くだけ（get は pass_len 0xFF、0xFF の set はそのまま保つ、hash は passphrase の代わりに乱数）。
  kMaxItems 800。Firmware/OepProbe の classic ESP32: TCP の port 7450、3 接続、wifi の項目（ビルドに認証情報なし、`-DOEP_WIFI=0` で Wi-Fi を外す）、
  Wi-Fi の build の restart_max_ms 15000（測って 5.5〜6.9 s）。例 `03.Transports/WifiTcp` を追加。host の試験: test_tcp（26）、test_config の
  wifi の項目（41 増）。M5Stack ATOM で確認: シリアルの口から設定（index 0 に無い SSID、1 に本物: 1 に接続）、mDNS の応答、TCP で confirm /
  list / describe / clock / open / end / gpio / config、2 つの接続（locked、lock_state）、閉じた接続を越えてセッションが残る、シリアルの口も同時に、
  `oep linktest`（失いなし）、oep-client-python の tests/hw を TCP とシリアルで（すべて通過）、TCP での restart。docs/implementation-limits §6
  （英語は stub）、README（EN / JA）、ガイド getting-started §6、writing-a-probe、boards（EN / JA）
=======
- (EN) Classic ESP32: a logic capture records again what a console command or a debug reset sent after its start does
  (bench, the V003 jig, f32a3ef: test_timing 0 rising edges in all 6 captures, reset_probe no low run in any of 10;
  5ad85ce / ff847a4 passed; the ESP32-P4 passed with f32a3ef). Cause: f32a3ef paused the console's reading for the whole
  sampler window (up to 164 ms immediate) and requests' SWIO frames already waited windows out, so the host's command
  (13-26 ms after the start) and the reset reached the target only after the capture had ended - the captured data was
  the line's constant level (all high / all low), not a sampler fault (the debug resets' READY came 177-227 ms after
  the start). The gate between the core-0 sampler and the SWIO frames (`OepWireGate.h`) now works frame by frame:
  inside a window a frame waits for the sampler to stop reading GPIO.in (one sample at most), the sampler waits for the
  frame's end and catches up - those samples late, the slipped flag - so no frame meets a GPIO.in read and neither
  requests nor the console wait for a window. `DmiPhy::backgroundTurn` / `backgroundDone`, the wire holder, its release
  in `SamplerCapture::poll()` / `waitIdle()` and the trigger search's `kWireTurnMs` turn are gone; a trigger search's
  slipped flag is its last burst's. Host test test_wire_gate: the gate's rules, a two-thread stress (no frame during a
  sample), 3000 request frames inside one window, and a console line (DmConsole dmseq on Ch32Dm and SwioPhy against a
  simulated target) delivered while one window stays open - with f32a3ef's pause the line never arrived; test_console's
  paused-wire case removed. Guides getting-started / writing-a-probe and docs/implementation-limits §4.1 (EN / JA);
  not run on hardware yet
- (JA) classic ESP32: ロジックのキャプチャが、開始の後に送ったコンソールのコマンドやデバッグのリセットの結果を再び写すように
  しました（ベンチ、V003 の治具、f32a3ef: test_timing は 6 つのキャプチャすべてで立ち上がり 0、reset_probe は 10 回すべてで low
  が無い。5ad85ce / ff847a4 は通り、ESP32-P4 は f32a3ef で通った）。原因: f32a3ef はコンソールの読みを sampler の窓の間ずっと
  止め（即時で 164 ms まで）、要求の SWIO のフレームはもともと窓の後に回していたので、host のコマンド（開始の 13-26 ms 後）と
  リセットはキャプチャが終わってから target に届いた。取れたデータは線の一定のレベル（すべて high / すべて low）で、sampler の
  故障ではない（デバッグのリセットの READY は開始の 177-227 ms 後）。core 0 の sampler と SWIO のフレームの間の門（`OepWireGate.h`）
  をフレームごとにしました: 窓の中では、フレームは sampler が GPIO.in を読むのを止めるのを待ち（多くてサンプル 1 つ）、sampler は
  フレームの終わりを待ってから追いつく（その間のサンプルは遅れ、slipped を立てる）。どのフレームも GPIO.in の読みと重ならず、
  要求もコンソールも窓を待たない。`DmiPhy::backgroundTurn` / `backgroundDone`、線の持ち主とその `SamplerCapture::poll()` /
  `waitIdle()` での手放し、トリガの探索の `kWireTurnMs` の番は無くなり、トリガの探索の slipped は最後の区切りのもの。ホストの試験
  test_wire_gate: 門の規則、2 スレッドの負荷（サンプル中にフレームが無い）、1 つの窓の中の要求のフレーム 3000、1 つの窓が開いた
  ままの間のコンソールの行の配達（模擬の target に対する Ch32Dm と SwioPhy の上の DmConsole dmseq。f32a3ef の止め方では届かない）。
  test_console の止まった線の場合は削除。ガイド getting-started / writing-a-probe と docs/implementation-limits §4.1（EN / JA）。
  実機ではまだ動かしていない
>>>>>>> 589acd5 (Classic ESP32: a logic capture records again what a console command or a debug reset sent after its start does (bench, the V003 jig, f32a3ef: test_timing 0 rising edges in all 6 captures, reset_probe no low run in any of 10; 5ad85ce / ff847a4 passed; the P4 passed with f32a3ef). Cause: f32a3ef paused the console's reading for a whole sampler window (up to 164 ms immediate) and requests' SWIO frames already waited windows out, so the command (13-26 ms after the start) and the reset reached the target after the capture had ended; the data was the line's constant level, not a sampler fault (debug resets' READY 177-227 ms after the start). OepWireGate.h now gates frame by frame: inside a window a SWIO frame waits for the sampler to stop reading GPIO.in (one sample at most), the sampler waits for the frame's end and catches up (those samples late: slipped) - no frame meets a GPIO.in read, and neither requests nor the console wait for a window. DmiPhy::backgroundTurn / backgroundDone, the wire holder and its release in SamplerCapture::poll() / waitIdle(), and kWireTurnMs are gone; a trigger search's slipped flag is its last burst's. Host test test_wire_gate: the rules, a two-thread stress, 3000 request frames in one window, a console line (DmConsole dmseq on Ch32Dm / SwioPhy, simulated target) delivered while one window stays open (fails with f32a3ef's pause); test_console's paused-wire case removed. Guides getting-started / writing-a-probe, docs/implementation-limits §4.1 (EN / JA); CHANGELOG (EN / JA); not run on hardware yet)
- (EN) Follows oep-spec 0f455a0 (the rule review of 2026-10-07, §2 and §7; the registry header and the test vectors synced
  from it). Core: no ignored TLV - an unknown non-critical TLV is ignored with nothing in the answer, an unknown critical
  one is unsupported; a TLV this probe implements is checked the same with or without bit 7 (another length: malformed;
  a value it does not handle: unsupported with the tag as received; `Tail::refuse`, no `finish` / `room` / `ignore`);
  a repeated tag: the first is used; the refusals keep header, resend table and session first, then every check before
  any change and any one reason; the resend table is (corr, answer) - no corr_reused, no CRC-32; list takes first(u16)
  only; fn 0's describe loses implementation, reserved, profile, resets_on_open and discoverable (`describeCore` has no
  reserved bitmap, model is free text; `setDiscoverable` removed); unavailable loses holder_fn / holder_kind; resource
  numbers go +1, skipping live ones; booleans read non-zero as true; request text no longer checked. Debug: the times
  and counts are this implementation's (`src/OepLimits.h`, docs/implementation-limits §1.2); P1 - on a connection the
  RVSWD wake goes out only inside attach and reset (`DmiPhy::holdWakes` / `WakeScope`), a link only a wake brings back
  closes on wire loss; P2 - no write that may have reached the target is repeated (documented per wire); Q1 - run whose
  preparation fails answers stopped 3 not_run with the hart still halted; run puts DATA0 / DATA1 back before the hart
  runs; reset answers status, flags (bit0 / bit1), pc - no method TLV, no attempts; timeout_ms 0 taken; a count > 0 scan
  ignores skip; a scan without max_speed runs at the wire's slowest speed; `wch_dmi_7f` is `dmi_7f`; arm-adi's TAR
  rewrite at `limits::kTarRewriteBytes`. Console: DMSTATUS is read before the next DATA0 read after a riscv-dm request
  (or attach) of the connection is answered (`Ch32Dm::noteRequest`), DATA0 / DATA1 untouched while halted, the 20 ms
  status read kept as this implementation's interval; no send_queue declaration; dmseq host rule 1's answer after three
  invalid words removed; the classic ESP32 pauses the console's reads during a sampler window (the old
  OEP_SWIO_PAUSE_CONSOLE behaviour, now always; the hook removed). Common: read from 3 with arg > 0xFF reads from now,
  write count 0 is success. Capture: no timing / rate_accuracy in the configure answer, no rate_list / rate_limit /
  max_read / segment_ring / frontend_shared / background / layout candidates / max_tracks / budget / start_skew; no
  always-critical TLVs. Fixture: gpio drive is a u8 level (0xFF default; past drive_levels or without them:
  unsupported), no read drive, no mode 7; uart status is baud and format; i2c-target has one form (address only, a
  write with data is one frame, reads from preload_tx slots else 0xFF, stretch kept as an optional op), no arm_rx /
  reset / modes / pullup_ohms; spi-target loses reset and counts errors once per transfer. probe.config: no slot lock,
  no boot_reset, bind is port(u8) kind(u8) id(u16) and resumes where it stopped; slot_state / bind_state shorter; idle
  is 4 bytes; the hash is the probe's own u32 (no canonical form); disable and idle applied before every other item at
  start-up; saved-settings format items6 / OEP6 (an older save reads as unreadable). port_speed is the handshake only:
  baud(u32) step(u8) verify_ms(u16) on the port it came on, idle fixed at port_speed_idle_ms, a rate further than
  port_speed_tolerance_pct refused unsupported, no revert on broken candidates. Kept as implementation behaviour: the
  reset note in the firmware text (Q4) and the safe boot. Host tests (tests/host/run.sh) all pass; one spec vector
  ("rvswd scan: count > 0 with skip", still malformed at 0f455a0) contradicts debug §1 and is checked against the text;
  docs/implementation-limits (EN / JA), guides and README follow; not run on hardware yet
- (JA) oep-spec 0f455a0（2026-10-07 の規則の見直し §2、§7。registry のヘッダと試験のベクタもそこから同期）に合わせました。本体:
  ignored の TLV は無くなり、知らない非 critical の TLV は応答に何も付けずに無視、知らない critical の TLV は unsupported。この probe が
  実装する TLV は bit 7 によらず同じに確かめます（長さ違いは malformed、扱わない値は受け取ったままの tag で unsupported。
  `Tail::refuse`。`finish` / `room` / `ignore` は廃止）。繰り返された tag は最初を使います。断りは見出し、送り直しの表、セッションの後、
  何も変える前にすべてを確かめ、当たった理由のどれか 1 つで断ります。送り直しの表は (corr、応答) で、corr_reused と CRC-32 は
  ありません。list は first(u16) だけ。fn 0 の describe から implementation、reserved、profile、resets_on_open、discoverable を外しました
  （`describeCore` は予約の bitmap を取らず、model は自由な文字列。`setDiscoverable` は削除）。unavailable から holder_fn / holder_kind を
  外しました。資源の番号は +1 で進み使用中を飛ばします。真偽値は 0 でなければ真、要求の text は確かめません。debug: 時間と回数はこの
  実装のもの（`src/OepLimits.h`、implementation-limits §1.2）。P1: connection の上で RVSWD の wake を送るのは attach と reset の中だけ
  （`DmiPhy::holdWakes` / `WakeScope`）。wake でしか戻らない線は線切れで閉じます。P2: target に届いたかもしれない書き込みは繰り返しません
  （線ごとに文書に記載）。Q1: 準備が失敗した run は stopped 3 not_run で、hart は止まったまま。run は hart を走らせる前に DATA0 / DATA1
  を書き戻します。reset の応答は status、flags（bit0 / bit1）、pc（method の TLV と attempts は無し）。timeout_ms 0 を受けます。count > 0
  の scan は skip を見ません。max_speed の無い scan は線の最も遅い速さ。`wch_dmi_7f` は `dmi_7f` に。arm-adi の TAR の書き直しは
  `limits::kTarRewriteBytes`。コンソール: その connection の riscv-dm の要求（と attach）に答えた後、次の DATA0 の前に DMSTATUS を
  読み（`Ch32Dm::noteRequest`）、止まっている間は DATA0 / DATA1 に触れません。20 ms ごとの DMSTATUS はこの実装の間隔として残しました。
  send_queue の宣言と、dmseq の host 規則 1 の「無効な語が 3 回で答える」を外しました。classic ESP32 は sampler の窓の間コンソールの
  読みを止めます（前の OEP_SWIO_PAUSE_CONSOLE の動きを常に。試験用の仕掛けは削除）。common: from 3 で arg > 0xFF は今から読み、
  write の count 0 は success。捕捉: configure の応答の timing / rate_accuracy、宣言の rate_list / rate_limit / max_read / segment_ring /
  frontend_shared / background / layout の候補 / max_tracks / budget / start_skew、常に critical の TLV を外しました。fixture: gpio の
  drive は u8 の段（0xFF は既定。drive_levels を越えるか宣言が無ければ unsupported）、read の drive と mode 7 は無し。uart の status は
  baud と format。i2c-target は 1 つの形（configure は address だけ、データのある書き込み 1 回 = 1 フレーム、読み出しは preload_tx の
  置き場か 0xFF、stretch は任意の op で残す）で、arm_rx / reset / mode / pullup_ohms は無し。spi-target は reset を外し、errors は転送 1 回に
  1 まで。probe.config: スロットの錠と boot_reset は無し。bind は port(u8) kind(u8) id(u16) で、止めた位置から再開。slot_state /
  bind_state は短く、idle は 4 byte。hash は probe が決める u32（正規形は無し）。起動時は disable と idle をほかのどの項目より先に。
  保存の形は items6 / OEP6（前の保存は読めないものになります）。port_speed は握手だけ: 来た口で baud(u32) step(u8) verify_ms(u16)、
  決めた後は port_speed_idle_ms 固定、port_speed_tolerance_pct を越えてずれる速さは unsupported、壊れたフレームでは戻りません。
  実装の動きとして残したもの: firmware の文字列のリセットの印（Q4）と safe boot。host の試験（tests/host/run.sh）はすべて通ります。
  仕様のベクタの 1 つ（「rvswd scan: count > 0 with skip」、0f455a0 でもまだ malformed）は debug §1 の本文と食い違うので、本文の
  とおりに確かめています。implementation-limits（EN / JA）、ガイド、README も合わせました。実機ではまだ動かしていません
- (EN) docs/implementation-limits.ja.md (new; docs/implementation-limits.md an English stub): the values the OEP
  specification leaves to the probe and this implementation's limits, by platform - gathered after oep-spec's rule review
  of 2026-10-07 moved them out of the specification (the wire's times and counts, max_op_ms 10000, restart_max_ms,
  search_retries' counting, the per-request dmi link check, the boot guard and safe boot, the reset note in the firmware
  text, the P4's DFU image valid at start-up, the RVSWD / SWIO / SWD notes, the classic ESP32's sampler windows and the
  wire / console, the P4's TinyUSB DWC2 panic on a SETUP in a status stage (EspUsbDevice 2.5.1, a reboot), SDI / DMDATA
  not telling a lost write, USB-UART bridges losing bytes on long probe-to-host streams with 500000 the practical rate).
  Documentation only; no code change
- (JA) docs/implementation-limits.ja.md（新規。docs/implementation-limits.md は英語の stub）: OEP の仕様が probe に任せた値と、
  この実装の限界を platform ごとにまとめました。oep-spec の 2026-10-07 の規則の見直しで仕様から外れたもの（線の時間と回数、
  max_op_ms 10000、restart_max_ms、search_retries の数え方、dmi の要求ごとの線の確かめ、boot guard と safe boot、firmware の
  文字列のリセットの印、P4 の DFU の image は起動で確定、RVSWD / SWIO / SWD の注、classic ESP32 の sampler の窓と線 / コンソール、
  P4 の TinyUSB DWC2 が status stage の SETUP で panic すること（EspUsbDevice 2.5.1、再起動になる）、SDI / DMDATA が失われた
  書き込みを見分けられないこと、USB-UART の変換器が長い probe → host の流れで byte を落とし 500000 が実用の速さであること）。
  文書だけで、コードは変えていません
- (EN) ESP32-P4: a DFU update's image is confirmed as it starts, with no trial (the user's decision: an update is taken
  as working, the firmware is not designed for a broken image). The sketch no longer defines verifyRollbackLater, so
  arduino-esp32 3.3.x's own (false) stands and initArduino marks the image valid
  (esp_ota_mark_app_valid_cancel_rollback) before setup() - the earliest point an app can; a panic, a watchdog, a
  brownout or a host's re-enumeration after that never goes back to the image before. Removed: the confirmation at
  BootGuard::stable() (30 s of loop() rounds) and stable() itself, oep.probe.restart's "confirm first", and with them
  the declared "a DFU update during the 30 s trial is refused (errTARGET)" and "a power cycle within it goes back".
  Kept: the fast crash-boot count and the safe boot (3 crash-boots within 30 s each: the at-boot slots' attach
  skipped), which guards against a crash loop the saved settings set off. BootGuard::lastBoot(): "rolled back from
  <slot>: ..." is gone (the RTC record no longer keeps a slot on trial); a panic / watchdog / brownout is "<reset> at
  <n> s" as before, and what the bootloader can still do - go back because the new image failed its check, or a reset
  came before the core confirmed it - is "update to <slot> did not reach setup: <reset>" (that boot's begin() never
  ran). Same mechanism elsewhere: the classic ESP32 has bootloader rollback in the core's sdkconfig too, but no in-app
  update and the core's verifyRollbackLater, so nothing pending; the RP2040 / RP2350 have no in-app update. Host test
  test_boot_guard (37 checks: an update that started then a task watchdog stays on app1, the kStableMs edge of a fast
  crash-boot); guides getting-started / writing-a-probe (EN / JA); not run on hardware yet; CHANGELOG (EN / JA)
- (JA) ESP32-P4: DFU で更新した image は、起動したときに確定するようにしました。試しの期間はありません（利用者の決定: 更新は
  動くものとして扱い、壊れた image のための作りはしない）。スケッチは verifyRollbackLater を定義しなくなったので、
  arduino-esp32 3.3.x のもの（false）が効き、initArduino が setup() の前に image を valid にする
  （esp_ota_mark_app_valid_cancel_rollback）- app ができる最も早いところ。その後の panic、watchdog、brownout、host の列挙
  し直しで前の image に戻ることはない。外したもの: BootGuard::stable()（loop() が 30 秒回った）での確定と stable() 自体、
  oep.probe.restart の「先に確定」、それに伴い宣言していた「30 秒の試しの期間の DFU 更新は断る（errTARGET）」と「その間の
  電源の入れ直しで前に戻る」。残したもの: すぐに落ちた起動の数え方と safe boot（30 秒以内に落ちた起動が 3 回続いたら at boot の
  スロットの attach を飛ばす）。保存した設定が起こす crash の繰り返しへの守り。BootGuard::lastBoot(): "rolled back from
  <slot>: ..." は無くなった（RTC の記録は試しの期間の slot を持たない）。panic / watchdog / brownout は前のとおり
  "<reset> at <n> s"。bootloader がまだしうること - 新しい image が自分の確認に通らないか、core が確定する前にリセットが来て
  前に戻る - は "update to <slot> did not reach setup: <reset>"（その起動の begin() は走っていない）。同じ仕組みのほか:
  classic ESP32 も core の sdkconfig に bootloader の rollback があるが、app の中の更新は無く、core の verifyRollbackLater の
  ままなので待ちの image は無い。RP2040 / RP2350 に app の中の更新は無い。host のテスト test_boot_guard（37 checks: 始まった
  更新の後の task watchdog は app1 のまま、すぐに落ちた起動の kStableMs の境）。guide の getting-started / writing-a-probe
  （EN / JA）。実機ではまだ動かしていない。CHANGELOG (EN / JA)
- (EN) Classic ESP32: no SWIO frame of a request while the core-0 sampler has a window open (bench, the V003 jig,
  aca403e with ch32rv e94dae6: uart_sweep with the capture on failed in 2 of 3 runs - a console command the DUT never
  acted on, once the DUT silent from just after one - every time in the wire step, where the command goes out 20 ms
  after the capture's start, inside its window; the capture-off runs passed but never send that command). From the
  code (not yet seen on the jig): the sampler reads GPIO.in back to back with interrupts off, the SWIO frames of
  loop()'s core are bit times made of GPIO register writes and reads over the same peripheral bus, and an access of one
  core waits for the other's in flight - the sampler already sees it from its side as samples taken late (slipped);
  one GPIO.in read is tens of cycles against a 262.5 ns short pulse, so a one can read as a zero, a pulse shrink or
  vanish and a frame be parsed as another register or value, with no parity on the wire and no read-back of a DMI
  write. OepWireGate.h: the sampler announces a window and waits for the wire's holder; loop()'s core takes the wire
  before a frame and holds it until loop() comes round (SamplerCapture::poll, and waitIdle before it waits for the
  sampler) - a request's frames wait out a window (up to 164 ms immediate, one burst of a search) and are never split
  by one; between the bursts of a trigger search a request kept waiting gets 5 ms (kWireTurnMs) before the next. The
  immediate segment's start_ns is taken as its window begins (it was the start op's time, before the task ran). The
  console's poll (DmiPhy::backgroundTurn) still reads through a window: oep-if-console §3 lets the probe stop reading
  only for the connection's riscv-dm request or a halted hart; pausing it there needs a spec change (proposed, not
  made). Test hook OEP_SWIO_PAUSE_CONSOLE=1: the console's poll skips while a window is open (the bench's check of the
  mechanism). Host tests test_wire_gate (two threads; built with and without the hook: no frame inside a window, 1579
  of 3000 inside without the gate) and test_console (a paused wire reads nothing, the queued line arrives whole after);
  guides getting-started / writing-a-probe (EN / JA); not run on hardware yet
- (JA) classic ESP32: core 0 の sampler が窓を開いている間、要求の SWIO のフレームを出さないようにしました（bench、V003 の
  jig、aca403e と ch32rv e94dae6: キャプチャ有りの uart_sweep が 3 回中 2 回失敗 - DUT が動かなかったコンソールのコマンド、
  1 回はその直後から DUT が黙った - どれも wire の段で、コマンドはキャプチャの start の 20 ms 後、その窓の中で出る。
  キャプチャ無しの回は通ったが、そのコマンドを一度も送らない）。コードから（jig ではまだ見ていない）: sampler は割り込みを止めて
  GPIO.in を続けて読み、loop() の core の SWIO のフレームは同じ周辺のバス越しの GPIO のレジスタの書き込みと読み出しでできた
  ビットの時間で、片方の core のアクセスはもう片方の進行中のアクセスを待つ - sampler は自分の側でそれを遅れたサンプル
  （slipped）として既に見ている。GPIO.in の読み 1 回は数十サイクルで、短いパルスは 262.5 ns なので、1 が 0 に読まれ、パルスが
  縮んだり消えたりし、フレームが別のレジスタや値と解釈されうる。線にパリティは無く、DMI の書き込みは読み戻さない。
  OepWireGate.h: sampler は窓を告げてから線の持ち主を待つ。loop() の core はフレームの前に線を取り、loop() が一回りするまで
  持つ（SamplerCapture::poll、および sampler を待つ前の waitIdle）- 要求のフレームは窓が終わるのを待ち（即時で 164 ms まで、
  探索なら区切り 1 つ）、窓に割られない。トリガの探索の区切りの間では、待たされた要求に次の区切りの前に 5 ms（kWireTurnMs）を
  渡す。即時の区画の start_ns は窓が始まるときに取る（start の op の時刻だった。タスクが動く前）。コンソールの poll
  （DmiPhy::backgroundTurn）は窓の間も読み続ける: oep-if-console §3 が読みを止めてよいとするのは、その接続の riscv-dm の要求と
  止まった hart だけ。そこで止めるには spec の変更が要る（提案のみ、していない）。テスト用の仕掛け OEP_SWIO_PAUSE_CONSOLE=1:
  窓が開いている間コンソールの poll を飛ばす（bench で仕組みを確かめるため）。host のテスト test_wire_gate（2 つのスレッド。
  仕掛けの有りと無しで build: 窓の中のフレームは 0、門が無いと 3000 中 1579）と test_console（止まった線は何も読まず、待っていた
  行は後で丸ごと届く）。guide の getting-started / writing-a-probe（EN / JA）。実機ではまだ動かしていない
- (EN) What ended the boot before is in fn 0's describe firmware text, after the version (bench, the X035 P4: b55bc68
  -> 462e180 by DFU, the HS port back within seconds, nothing touching the board, describe about 35 s later said
  b55bc68 - a rollback with no visible cause; the same DFU 30 s later stayed). BootGuard::lastBoot(): the reset
  (esp_reset_reason: panic, task-wdt, int-wdt, wdt, brownout, usb, jtag, reset-pin, cpu-lockup, software not made by the
  probe, other; RP2: wdt) and the seconds that boot was up, "<reset> at <n> s"; an update on trial that the next reset
  rolled back, "rolled back from <slot>: <reset> at <n> s"; an update the bootloader did not start, or that ended
  before setup(), "update to <slot> did not reach setup: <reset>"; nothing after a power-on or a restart the probe made
  (oep.probe.restart, a DFU update's). The record a reset leaves (ESP: RTC memory, now with the seconds up, the slot on
  trial and the slot a planned restart boots; RP2: watchdog scratch 2 for the seconds) is written each second by
  poll(); planned() keeps its own state. describeCore takes it as an optional note ("<version> (<note>)"); the
  Firmware/OepProbe sketches pass it, their fn 0 describe buffer 256 bytes. No wire field is added. From the code,
  the candidates for that reset within the trial (none confirmed): a panic or a watchdog in the first 30 s (the at-boot
  attach and USB settle under the 15 s loop watchdog; the fixture UART's begin task on core 0, only with a UART planned
  at boot), a brownout as the HS port and the target come up, a reset over USB-Serial/JTAG before setup() turns its
  DTR / RTS reset off, a restart the probe did not make; the DFU restart runs once (dfuDone is cleared by the reset)
  and the otadata writes are IDF's in order (esp_ota_end, esp_ota_set_boot_partition; the mark valid at stable()).
  esp_restart on the P4 resets the CPUs and some peripherals but not the USB controllers (IDF
  esp_system_reset_modules_on_exit). Host test test_boot_guard (45 checks: the notes for a power-on, a panic, a restart
  not by the probe, oep.probe.restart, an update not started, one rolled back by the task watchdog, one confirmed then
  the reset pin); guides getting-started / writing-a-probe (EN / JA); not run on hardware yet; CHANGELOG (EN / JA)
- (JA) 前の起動が何で終わったかを、fn 0 の describe の firmware の文字列に版の後ろに付けました（bench、X035 の P4: b55bc68 から
  462e180 へ DFU、HS の口は数秒で戻り、基板には何も触れず、約 35 秒後の describe は b55bc68 - 見える理由のない rollback。
  30 秒後の同じ DFU は残った）。BootGuard::lastBoot(): リセットの種類（esp_reset_reason: panic、task-wdt、int-wdt、wdt、
  brownout、usb、jtag、reset-pin、cpu-lockup、probe がしていない software、other。RP2: wdt）とその起動が上がっていた秒数を
  "<reset> at <n> s"。試しの期間の更新が次のリセットで戻されたら "rolled back from <slot>: <reset> at <n> s"。bootloader が
  始めなかった、または setup() の前に終わった更新は "update to <slot> did not reach setup: <reset>"。電源の投入と probe 自身の
  再起動（oep.probe.restart、DFU の更新の再起動）の後は何も付けない。リセットが残す記録（ESP: RTC のメモリ。上がっていた秒数、
  試しの期間の slot、計画した再起動が起こす slot を足した。RP2: 秒数は watchdog の scratch 2）は poll() が毎秒書く。planned()
  は自分の状態を持つ。describeCore は任意の注記として受ける（"<version> (<note>)"）。Firmware/OepProbe のスケッチが渡し、fn 0 の
  describe のバッファを 256 byte にした。線の上の項目は足していない。コードから見た、試しの期間のそのリセットの候補（どれも
  確かめていない）: 最初の 30 秒の panic か watchdog（15 秒の loop の watchdog の下の起動時の attach と USB の落ち着き待ち。
  core 0 の fixture の UART の begin のタスクは、起動時に UART を plan しているときだけ）、HS の口と target が上がるときの
  brownout、setup() が USB-Serial/JTAG の DTR / RTS のリセットを止める前のそのリセット、probe がしていない再起動。DFU の再起動は
  一度だけ走る（dfuDone はリセットで消える）。otadata の書き込みは IDF の順（esp_ota_end、esp_ota_set_boot_partition。valid
  にするのは stable()）。P4 の esp_restart は CPU と一部の周辺を reset するが USB のコントローラは reset しない（IDF の
  esp_system_reset_modules_on_exit）。host のテスト test_boot_guard（45 checks: 電源の投入、panic、probe がしていない再起動、
  oep.probe.restart、始まらなかった更新、task watchdog で戻された更新、確かになった後の reset ピン の注記）。guide の
  getting-started / writing-a-probe（EN / JA）。実機ではまだ動かしていない。CHANGELOG (EN / JA)
- (EN) Fixture UART: a lost mark is at or before the first byte after the loss, never after it, and where the driver
  tells, exactly at it (bench, b55bc68, the X035 P4 at 2000000: a 256-byte burst came back 251 bytes, the first wrong
  one at offset 4, one mark lost overflow at 132). The spec gives a lost mark's position no meaning of its own (common
  §1.3); the reading taken: the bytes from the position on came after the loss (for a receive loss: the gap's next byte
  or before it; a ring's push-out and the console's TO mark already are). Before, the ESP-IDF driver's error callback
  only counted, from arduino-esp32's event task, and poll() marked the count after taking all the driver held - after
  every byte that followed the gap. ESP32 / ESP32-P4: FixtureUart takes the driver's event queue itself, in order, in a
  task of its own on the UART interrupt's core at arduino-esp32's event-task priority (UartRxLedger, OepUartRx.h):
  UART_DATA / UART_BUFFER_FULL count the bytes into the driver's ring, a FIFO overflow is placed after them (the driver
  reads the FIFO before it resets it: exact), a framing / parity error or a break at the start of the chunk read with it
  in the same interrupt (at or before the bad byte); poll() takes no byte past what the events counted, so a late event
  never puts its mark behind bytes already in the stream. A buffer full is no longer marked (the driver stashes the
  chunk and loses nothing; what it cannot take comes as a FIFO overflow). The driver's event queue (20) found full: a
  loss after the events it held (dropped events cannot be read); bytes found in the ring that no event counted put the
  count right. More losses than the ledger's 16 before a poll: the rest merged into one at the 17th's place. No
  onReceiveError: arduino-esp32 makes its own event task only for that. RP2 (arduino-pico): its receive queue's overflow
  exactly at the gap (the queue stays full and unread from the drop until poll() looks before taking; a drop while poll()
  took: at the end of what it counted); the PL011's overrun and a break, read as flags, at the bytes counted at the look
  before, which came before them. A configure takes what the UART received, and its losses, before it starts again.
  Same mechanism elsewhere: the console streams' lost marks (the ring's push-out at the write position, the dmseq TO
  after the frame's payload) already mean the same - unchanged; oep.fixture.logic (ESP32-P4 PARLIO): a DMA chunk the
  harvest queue (128) had no room for was counted nowhere, and the chunks after it took its positions with no gap flag
  (and wrong times) - each chunk now carries produced_ after it, the harvest sees the missing bytes, a repeat / streaming
  segment or frame starts after them with the gap flag (the segment before cut short, as an overrun), a triggered
  segment copies them from the ring; oep.fixture.analog's lost conversions only flag the segment, which cannot come
  after them (unchanged). Host tests: test_fixture_uart in two builds - ESP-IDF style (the bench's case, 400 rounds with
  late events, errors, a full event queue, 24 losses before a poll, a configure with a loss waiting; 38 checks, 9 fail
  on 462e180) and arduino-pico style (OEP_HOST_FAKE_UART_RP2; 15 checks, 3 fail on 462e180); test_capture
  testLostChunk (2 fail on 462e180); guide writing-a-probe (EN / JA); not run on hardware yet; CHANGELOG (EN / JA)
- (JA) fixture の UART: lost のマークを、抜けた所の直後のバイトの位置かそれより前に、後ろには付けないようにしました。driver が
  教える所ではちょうどその位置です（bench、b55bc68、X035 の P4 で 2000000: 256 byte のバーストが 251 byte で返り、最初に違う
  byte が offset 4、lost overflow のマークが一つ 132）。仕様は lost のマークの位置に固有の意味を定めていない（common §1.3）。
  取った読み: その位置から後のバイトは失われたものより後に来た（受信で抜けたなら、抜けた所の次のバイトかその前。リングの
  押し出しとコンソールの TO のマークはもうそうなっている）。これまでは ESP-IDF の driver の誤りの callback が arduino-esp32 の
  event のタスクから数えるだけで、poll() は driver が持つ分をすべて取ってからその数をマークにしていた - 抜けた所の後のバイトを
  すべて越えて。ESP32 / ESP32-P4: FixtureUart が driver の event の列を自分で順に、UART の割り込みの core にある自分のタスクで
  arduino-esp32 の event のタスクと同じ優先度で取る（UartRxLedger、OepUartRx.h）。UART_DATA / UART_BUFFER_FULL は driver の
  リングに入ったバイトを数え、FIFO のあふれはその後ろに置く（driver は FIFO を空にする前に読む: ちょうど）。framing / parity
  の誤りと break は、同じ割り込みで一緒に読んだ塊の頭に置く（誤ったバイトの位置かその前）。poll() は event が数えた所より先の
  バイトを取らないので、遅れた event のマークがもうストリームにあるバイトの後ろに付くことはない。buffer full はマークにしない
  （driver は塊を退避して何も失わない。入りきらない分は FIFO のあふれとして来る）。driver の event の列（20）が満ちていたら、
  そこにあった event の後ろに lost を置く（落ちた event は読めない）。どの event も数えていないバイトがリングにあれば数を直す。
  poll までに ledger の 16 を超える lost が来たら、残りは 17 番目の所に一つにまとめる。onReceiveError は使わない
  （arduino-esp32 はそのためだけに自分の event のタスクを作る）。RP2（arduino-pico）: 受信の列のあふれは抜けた所ちょうど
  （列は落とした時から poll() が取る前に見るまで満ちたまま読まれない。poll() が取っている間に落ちたら、見たときに数えた分の
  終わり）。PL011 の overrun と break は印としてしか読めないので、一つ前に見たときに数えたバイト（それより前に来た）の所。
  configure は UART が受けた分とその lost を取ってから UART を始め直す。同じ仕組みのほかの所: コンソールのストリームの lost
  （リングの押し出しは書く位置、dmseq の TO はフレームの中身の後ろ）はもう同じ意味 - 変えない。oep.fixture.logic（ESP32-P4 の
  PARLIO）: 取り込みの列（128）に入らなかった DMA の塊はどこにも数えられず、後の塊がその位置を gap の印なしに（時刻も違えて）
  取っていた - 塊がその後の produced_ を持つようにし、取り込み側が抜けたバイトに気づく。リピートとストリーミングは区画・
  フレームをその後ろから gap の印で始め（前の区画は overrun と同じく短く切る）、トリガー付きの区画はリングからその分を写す。
  oep.fixture.analog の失われた変換は区画に印を立てるだけで、それより後に来ることはない（変えない）。host のテスト:
  test_fixture_uart を 2 つのビルドで - ESP-IDF の形（bench の場合、遅れた event・誤り・満ちた event の列を交えた 400 回、
  poll までに 24 の lost、lost が待つ間の configure。38 checks、462e180 では 9 つ落ちる）と arduino-pico の形
  （OEP_HOST_FAKE_UART_RP2。15 checks、462e180 では 3 つ落ちる）。test_capture の testLostChunk（462e180 では 2 つ落ちる）。
  guide の writing-a-probe（EN / JA）。実機ではまだ動かしていない。CHANGELOG (EN / JA)
- (EN) Fixture UART: a DUT's continuous burst at 2000000 8N1 is received without the hardware dropping bytes on the
  ESP32-P4, and an overrun the RP2's UART had is marked lost (bench, the X035 and WeAct P4 jigs: uart_sweep at 2000000,
  DUT to probe only, about 125 bytes of a burst wrong, intermittent - X035 passed on 47e4b05 / c75884b and failed on
  b55bc68, WeAct failed on 47e4b05 and passed on b55bc68; 1500000 and 1959983 passed; the echo was right). From the
  code (not yet seen on the jigs): the fixture UARTs' driver was installed from loop(), so their receive interrupt ran
  on core 1, where the SWIO / RVSWD frames turn interrupts off one frame at a time (SWIO about 50 us plus up to 100 us
  for the line to rise, RVSWD about 55 us at attach's period, up to about 1.1 ms at the slowest max_speed); and
  arduino-esp32 sets the RX FIFO interrupt at 120 of 128 bytes above 57600, so 8 bytes - 40 us at 2000000, 53 us at
  1500000 - were left before the FIFO overflowed and the hardware dropped what came next (ESP-IDF resets the FIFO on
  the overflow). ESP32-P4: the fixture UARTs' interrupt runs on core 0 (FixtureUart::setInterruptCore,
  platformUartBegin's irq_core: begin() in a task pinned there, as the driver allocates its interrupt on the core that
  installs it; static_assert that it is not loop()'s core), where the probe turns no interrupts off. Every ESP32: the
  RX FIFO interrupt at oep::kUartRxFifoFull = 32 bytes (96 left: 480 us at 2000000); the driver's receive buffer on the
  P4 16 KiB (82 ms at 2000000 of loop() away; was 4 KiB, 20 ms). Classic ESP32: the fixture UART stays on loop()'s
  core (core 0 is the sampler's, interrupts off for up to 250 ms) with the same threshold, and a static_assert that the
  FIFO's rest at kMaxBaud outlasts SwioPhy::kIrqOffMaxUs twice (480 us against 400). Lost bytes are marked: the ESP-IDF
  driver reports its FIFO overflow and a full receive buffer as events, marked lost overflow as before (66345b7), and a
  flash write (settings, a DFU update), which holds every non-IRAM interrupt off on both cores, still drops what the
  FIFO cannot hold - marked so. Whether the bench's wrong bytes came with a lost mark is not known from the report.
  RP2 (arduino-pico 6.1.1): SerialUART's interrupt at 4 of 32 bytes leaves 28 (140 us at 2000000) against an RVSWD
  frame on loop()'s core (55-80 us at attach's period, 11-20 us attached, longer at a slow max_speed); its handler
  keeps the byte that carries the PL011's overrun flag and says nothing of those dropped before it - now poll() reads
  the receive status's OE and clears it (platformUartTakeOverrun), marked lost overflow. 2000000 stays the fastest rate
  configure takes on every probe. Host test test_fixture_uart (fake UART, OEP_HOST_FAKE_UART: the core every begin
  runs on, 2000001 refused, 60 rounds of bytes and overruns - every byte in order, one lost mark per overrun within its
  window, none without; 15 checks; without the change 3 fail); guide writing-a-probe (EN / JA); not run on hardware
  yet; CHANGELOG (EN / JA)
- (JA) fixture の UART: ESP32-P4 で、DUT が 2000000 8N1 で続けて送るバイトを、ハードウェアが落とさずに受けるようにしました。
  RP2 の UART の overrun を lost のマークにしました（bench、X035 と WeAct の P4 の治具: 2000000 の uart_sweep、DUT から probe
  の向きだけ、バーストの約 125 byte が違う、ときどき - X035 は 47e4b05 / c75884b で通り b55bc68 で落ち、WeAct は 47e4b05 で
  落ち b55bc68 で通った。1500000 と 1959983 は通った。echo は正しかった）。コードから（治具ではまだ見ていない）: fixture の
  UART のドライバは loop() から入れていたので、受けの割り込みは core 1 で動いていた。そこでは SWIO / RVSWD のフレームが
  フレームごとに割り込みを止める（SWIO は約 50 us と線が上がるのを待つ 100 us まで、RVSWD は attach の周期で約 55 us、最も遅い
  max_speed で約 1.1 ms まで）。arduino-esp32 は 57600 より上で RX FIFO の割り込みを 128 byte 中 120 byte に置くので、FIFO が
  あふれてハードウェアが次を落とすまでの余りは 8 byte - 2000000 で 40 us、1500000 で 53 us - だった（ESP-IDF はあふれで FIFO
  を空にする）。ESP32-P4: fixture の UART の割り込みを core 0 で動かす（FixtureUart::setInterruptCore、platformUartBegin の
  irq_core: ドライバは入れた core に割り込みを置くので、その core に固定した task で begin() する。loop() の core でないことの
  static_assert）。core 0 では probe は割り込みを止めない。すべての ESP32: RX FIFO の割り込みを oep::kUartRxFifoFull = 32 byte
  に（余り 96 byte: 2000000 で 480 us）。P4 のドライバの受けのバッファを 16 KiB に（loop() が離れていられるのは 2000000 で
  82 ms。これまで 4 KiB、20 ms）。classic ESP32: fixture の UART は loop() の core のまま（core 0 は sampler のもので、最大
  250 ms 割り込みを止める）で、同じ閾値。kMaxBaud での FIFO の余りが SwioPhy::kIrqOffMaxUs の 2 倍より長いことの
  static_assert（480 us 対 400）。落ちたバイトはマークになる: ESP-IDF のドライバは FIFO のあふれと受けのバッファが満ちたことを
  event で知らせ、これまでどおり lost overflow にする（66345b7）。flash の書き込み（設定、DFU の更新）は両方の core の IRAM に
  無い割り込みをすべて止めるので、その間 FIFO に入りきらない分はなお落ちる - そのとおりマークになる。bench の違うバイトに
  lost のマークが付いていたかは、報告からは分からない。RP2（arduino-pico 6.1.1）: SerialUART の割り込みは 32 byte 中 4 byte で
  起き、余りは 28 byte（2000000 で 140 us）。loop() の core の RVSWD のフレーム（attach の周期で 55〜80 us、attach した後
  11〜20 us、遅い max_speed ではもっと長い）と比べる。その handler は PL011 の overrun の印を持つ byte を残し、その前に落ちた分を
  何も言わなかった - これから poll() が受けの状態の OE を読んで消し（platformUartTakeOverrun）、lost overflow にする。
  configure が受ける最も速い値はどの probe も 2000000 のまま。host のテスト test_fixture_uart（偽の UART、OEP_HOST_FAKE_UART:
  begin がどの core で動くか、2000001 を断る、バイトと overrun を 60 回 - バイトはすべて順に、overrun ごとに lost のマークが
  一つその範囲に、無ければ無い。15 checks。変更が無いと 3 つ落ちる）。guide の writing-a-probe（EN / JA）。実機ではまだ
  動かしていない。CHANGELOG (EN / JA)
- (EN) ESP32-P4: a DFU update is confirmed once its boot is stable, not when a host first configures the HS port
  (bench, the WeAct P4 on a WSL host: 9942787 over 3d83ffa and c75884b over 47e4b05 twice - "628848 bytes in 154
  blocks", the reboot, then describe over HS said the image before; no replug, restart or USB-Serial/JTAG use; usbipd
  attached the HS port only after many tries (about 60 s once); the X035 jig, attached at once, kept the same images).
  The core's P4 sdkconfig has BOOTLOADER_APP_ROLLBACK_ENABLE; the sketch confirmed the image (markValid) in loop() at
  usbDevice.ready(), and IDF's bootloader marks every image still pending verification aborted at the next boot of any
  kind, so the trial lasted as long as the host took and some reset inside it rolled the image back (which reset is not
  known yet: esp_restart and a panic reset on the P4 reset only the CPUs, the USB controllers keep their state; the
  image itself checks: header and hash valid, chip revision 0 - max, its RAM segments end at 0x4ff149a0, the
  bootloader's start at 0x4ff29ed0). Now: confirmed at BootGuard::stable() (kStableMs = 30 s up with loop() coming
  round under the task watchdog), whatever the host does with USB; a crash, a panic or a stall before then still rolls
  back (static_assert: kStableMs > kAttachGraceMs + kUsbSettleMs + kStallMs, so a stall in the at-boot attach resets
  inside the trial); oep.probe.restart confirms first (the firmware answered it). Declared: a DFU update during the
  30 s trial is refused (errTARGET; esp_ota_begin on an image pending verification), and a power cycle within it goes
  back to the image before. BootGuard::stable(); host tests (test_boot_guard, 36 checks); guides getting-started /
  writing-a-probe (EN / JA); not run on hardware yet; CHANGELOG (EN / JA)
- (JA) ESP32-P4: DFU の更新を、host が HS の口を最初に configure したときではなく、起動が安定したときに確かなものにしました
  （bench、WSL の host の WeAct P4: 3d83ffa の上の 9942787、47e4b05 の上の c75884b を 2 回 - "628848 bytes in 154 blocks"、
  再起動、そのあと HS の describe は前の image だった。抜き差しも restart も USB-Serial/JTAG の使用も無い。usbipd が HS の口を
  attach できたのは何度も試したあと（1 回は約 60 秒）。すぐ attach された X035 の治具は同じ image を保った）。core の P4 の
  sdkconfig は BOOTLOADER_APP_ROLLBACK_ENABLE。sketch は loop() で usbDevice.ready() のときに image を確かにしていた
  （markValid）。IDF の bootloader は、確かめ待ちのままの image を、次のどんな起動でも aborted にする。だから試しの期間は host が
  かかっただけ続き、その間のなにかのリセットで前に戻った（どのリセットかはまだ分からない: P4 の esp_restart と panic の
  リセットは CPU だけをリセットし、USB のコントローラは状態を保つ。image 自体は確かめた: header と hash は正しく、chip
  revision 0 - max、RAM の segment の終わりは 0x4ff149a0、bootloader の始まりは 0x4ff29ed0）。これから: BootGuard::stable()
  （task watchdog の下で loop() が回りながら kStableMs = 30 秒たった）で確かにする。host が USB で何をしても同じ。それより
  前に落ちる、panic、止まるなら、これまでどおり前に戻る（static_assert: kStableMs > kAttachGraceMs + kUsbSettleMs +
  kStallMs。起動時の attach で止まっても試しの期間の中でリセットされる）。oep.probe.restart は先に確かにする（firmware が
  それに答えた）。宣言: 30 秒の試しの期間の DFU 更新は断る（errTARGET。確かめ待ちの image での esp_ota_begin）。その間の電源の
  入れ直しでも前に戻る。BootGuard::stable()。host のテスト（test_boot_guard、36 checks）。guide の getting-started /
  writing-a-probe（EN / JA）。実機ではまだ動かしていない。CHANGELOG (EN / JA)
- (EN) Classic ESP32: UART0's RX interrupt at 32 bytes in the 128-byte FIFO, not arduino-esp32's 120 (bench, the V003 jig,
  47e4b05, a CH340 bridge: frames broke at 500000 and the probe reverted to the boot speed mid-upload; 500000 was
  clean there on 2026-10-02). The UART's interrupt runs on loop()'s core, which each SWIO frame keeps from it
  (portENTER_CRITICAL); at 120 the FIFO had 8 bytes left - 160 us at 500000, 87 us at 921600 - against a frame of about
  60 us plus its wait for the line to rise (per bit, 32 x 1000 polls at most, until the commit before; now per frame).
  At 32 it has 96: 1.9 ms at 500000, 1.04 ms at 921600, 480 us at 2000000. A byte lost from a request breaks its frame
  and three broken in a row revert a raised speed (oep-if-link §3). SwioPhy::kIrqOffMaxUs (200 us, the classic's
  1000 polls estimated at about 0.1 us each, not measured) and a static_assert that the rest of the FIFO outlasts it
  twice at 2000000. From the code, the probe's sending side cannot drop or reorder bytes: one writer (loop()), the
  driver's 8 KiB TX ring with a blocking write, a bind's raw bytes only between frames (Endpoint::rawOut after the
  results and pushes) and none on a port a session holds, nothing else on UART0 (IDF log off, no printf); an
  interrupt-off frame only pauses the line (the TX FIFO refills at 10 bytes left, 200 us at 500000), far below any gap
  rule (probe_frame_gap_ms 200 ms). Answers breaking in the probe -> host direction are not explained by the code; not
  run on hardware yet; CHANGELOG (EN / JA)
- (JA) classic ESP32: UART0 の受けの割り込みを、128 byte の FIFO に 32 byte たまったときにしました（arduino-esp32 の既定は
  120）。（bench、V003 の治具、47e4b05、CH340 のブリッジ: 500000 でフレームが壊れ、upload の途中で probe が起動時の速さに
  戻った。2026-10-02 にはそこで 500000 がきれいだった。）UART の割り込みは loop() の core で動き、SWIO のフレームはそれを
  止める（portENTER_CRITICAL）。120 では FIFO の残りが 8 byte - 500000 で 160 us、921600 で 87 us - で、フレームは約 60 us に
  線が上がるのを待つ時間（一つ前の commit まではビットごとで最大 32 x 1000 回、今はフレームごと）。32 では残りが 96 byte:
  500000 で 1.9 ms、921600 で 1.04 ms、2000000 で 480 us。要求の 1 byte が失われるとそのフレームが壊れ、3 つ続けて壊れると
  上げた速さが戻る（oep-if-link §3）。SwioPhy::kIrqOffMaxUs（200 us。classic の 1000 回は 1 回約 0.1 us の見積もりで、測って
  いない）と、FIFO の残りが 2000000 でその 2 倍より長いことの static_assert。コードからは、probe の送る側はバイトを落とすことも
  順を変えることもできない: 書き手は 1 つ（loop()）、ドライバの 8 KiB の送りのリングに止まって書く、bind の生のバイトはフレームの
  間だけ（Endpoint::rawOut は応答と push の後）で、セッションが持つ口には出さない、UART0 にはほかに何も書かない（IDF の log は
  切ってある、printf は無い）。割り込みを止めたフレームは線を止めるだけ（送りの FIFO は残り 10 byte で補う。500000 で 200 us）
  で、どの途切れの規則（probe_frame_gap_ms 200 ms）よりずっと短い。probe -> host の向きで応答が壊れることはコードでは説明が
  つかない。実機ではまだ動かしていない。CHANGELOG (EN / JA)
- (EN) A probe never stays wedged, and the saved settings cannot crash it at every boot (bench, 0.0.29-dev+3c0cd99, SWD
  on a Pro Micro RP2350, arduino-pico 6.1.1, usbstack=picosdk: after a restart, during the host's first GET_DESCRIPTOR,
  the USB interrupt ended in TinyUSB 0.18's panic "Can't continue xfer on inactive ep" (dcd_rp2040_irq ->
  hw_handle_buff_status -> hw_endpoint_xfer_continue); the breakpoint escalated to a HardFault, whose own breakpoint
  stopped core 0 with the pull-up on - the host failed the device descriptor request until a replug, no watchdog ran.
  The interrupted thread was a slot's console from the saved settings, polling its target over RVSWD from boot
  (RvswdPhy::writeRaw, at the end of a frame with interrupts off): the USB interrupts came late and batched while the
  host enumerated the device). BootGuard (OepBootGuard.h / .cpp): RP2 - the hardware watchdog (3000 ms, paused under a
  debugger) fed from a 250 ms timer interrupt only while loop() comes round within kStallMs (max_op_ms + 5 s = 15 s);
  the firmware points arduino-pico's weak _exit (where the SDK's panic() ends) and crt0's weak isr_hardfault at BootGuard::crashed(),
  which takes the device off the bus (the pull-up off), waits kRestartDetachMs and resets; a stall does the same.
  ESP32 / ESP32-P4 - a panic and the interrupt watchdog already reset (the core's sdkconfig: PANIC_PRINT_REBOOT,
  INT_WDT 300 ms); the task watchdog, which watched only CPU0's idle task, now also watches loop() (enableLoopWDT) with
  its timeout raised from 5 s to 15 s. Fast crash-boots are counted (RP2: watchdog scratch 0 / 1 and a watchdog reset;
  ESP: RTC memory and esp_reset_reason panic / watchdog): a crash within 30 s of boot counts one more, a boot up 30 s, a
  restart on purpose (platformRestart, the P4's restart and DFU restart) or any other reset starts at 0; after 3 in a
  row the boot is a safe one - ProbeConfig::skipBootAttach: the saved at-boot slots are not attached by the probe in
  that boot (their state reads 1 with last_try_at_ns all ones, "never tried"; a set of the slot or the host's attach
  connects them); idles, plans, uarts and binds are applied as usual. The at-boot attach of a USB probe waits for the
  host (ProbeConfig::setAttachGate, BootGuard::attachReady): the RP2 until tud_mounted() has held for 1 s, the P4 until
  the HS device is configured as long, or 5 s after boot without a host; the classic ESP32 (a UART bridge) does not
  wait. Interrupts off: one frame at a time on every PHY. RVSWD (RP2, P4): 109 half periods - at the slowest period a
  link may be held to (max_speed 50 kHz) about 1.1 ms (static_assert), at attach's 500 ns about 55 us (RP2350 ~80 us a
  read with the call), attached at 0-100 ns 11-20 us. SWIO (P4, classic): a write <= 46 us, a read about 50 us plus at
  most one wait for the line to rise per frame - 100 us on the P4, 1000 polls of GPIO.in on the classic; that wait was
  per bit (up to 32 x 100 us = 3.2 ms with interrupts off on a line rising slowly at every bit) and is now the frame's.
  SWD (RP2): none (no interrupts masked). The classic's sampler keeps its own bounds (164 ms / 250 ms bursts on core 0,
  under the 300 ms interrupt watchdog). Upstream: TinyUSB 0.21.0 (2026-06-30, the rp2 rework of PR #3561) no longer
  panics there (an idle endpoint's buffer status is ignored); pico-sdk 2.3.1 and arduino-pico 6.1.1 / 6.2.0 still pin
  TinyUSB 0.18.0 (86ad6e56c). Host tests: test_boot_guard (the count over resets, the safe boot, planned restarts, the
  USB gate: 26 checks), test_config (the gate closed holds the saved at-boot attach, open attaches; a skipping boot never
  tries until the slot is set again); the rest as before. Not run on hardware yet; CHANGELOG (EN / JA)
- (JA) probe が止まったままにならず、保存した設定が起動のたびに probe を落とし続けることもないようにしました（bench、
  0.0.29-dev+3c0cd99、Pro Micro RP2350 を SWD で、arduino-pico 6.1.1、usbstack=picosdk: restart の後、host の最初の
  GET_DESCRIPTOR の最中に USB の割り込みが TinyUSB 0.18 の panic "Can't continue xfer on inactive ep"（dcd_rp2040_irq ->
  hw_handle_buff_status -> hw_endpoint_xfer_continue）で終わり、breakpoint が HardFault に上がり、その HardFault の
  breakpoint で core 0 が pull-up を付けたまま止まった。host は抜き差しまで device descriptor の要求に失敗し、watchdog は
  動いていなかった。割り込まれていたのは保存した設定のスロットのコンソールで、起動時から RVSWD で target を読んでいた
  （RvswdPhy::writeRaw、割り込みを止めたフレームの終わり）: host がデバイスを列挙している間、USB の割り込みが遅れてまとめて
  来ていた）。BootGuard（OepBootGuard.h / .cpp）: RP2 は hardware watchdog（3000 ms、debugger が止めている間は止まる）を、
  loop() が kStallMs（max_op_ms + 5 s = 15 s）以内に回っている間だけ 250 ms の timer 割り込みから送る。firmware は arduino-pico の弱い
  _exit（panic() の行き着く先）と crt0 の弱い isr_hardfault を BootGuard::crashed() に向け、デバイスをバスから外し（pull-up を
  切る）、kRestartDetachMs 待って reset する。loop() が止まったときも同じ。ESP32 / ESP32-P4 は panic と割り込みの watchdog で
  もともと reset する（core の sdkconfig: PANIC_PRINT_REBOOT、INT_WDT 300 ms）。CPU0 の idle task しか見ていなかった task
  watchdog に loop() も見させ（enableLoopWDT）、期限を 5 s から 15 s にした。すぐに落ちた起動を数える（RP2 は watchdog の
  scratch 0 / 1 と watchdog の reset、ESP は RTC のメモリと esp_reset_reason の panic / watchdog）: 起動から 30 s 以内に
  落ちれば 1 つ増え、30 s 動いた起動、意図した restart（platformRestart、P4 の restart と DFU の restart）、ほかの reset では
  0 に戻る。3 回続いたら安全な起動にする: ProbeConfig::skipBootAttach で、その起動では保存した at boot のスロットに probe から
  attach しない（state は 1、last_try_at_ns は全ビット 1 =「試していない」。スロットの set か host の attach でつながる）。idle、
  plan、uart、bind はいつもどおり掛ける。USB の probe の at boot の attach は host を待つ（ProbeConfig::setAttachGate、
  BootGuard::attachReady）: RP2 は tud_mounted() が 1 s 続くまで、P4 は HS デバイスが同じだけ構成されているまで、host が
  いなければ起動から 5 s。classic ESP32（UART ブリッジ）は待たない。割り込みを止めるのはどの PHY もフレーム 1 つずつ。RVSWD
  （RP2、P4）: 109 半周期 - 線を抑えられる最も遅い周期（max_speed 50 kHz）で約 1.1 ms（static_assert）、attach の 500 ns で
  約 55 us（RP2350 は呼び出し込みで 1 read 約 80 us）、attach 後の 0〜100 ns で 11〜20 us。SWIO（P4、classic）: write は
  46 us 以下、read は約 50 us に、線が上がるのを待つ時間をフレームで 1 回分まで - P4 は 100 us、classic は GPIO.in を 1000 回。
  この待ちはビットごとだった（ビットごとにゆっくり上がる線で、割り込みを止めたまま最大 32 x 100 us = 3.2 ms）のを、フレーム
  全体のものにした。SWD（RP2）: 割り込みを止めない。classic の sampler は自分の上限のまま（core 0 で 164 ms / 250 ms の
  まとまり、300 ms の割り込み watchdog の内側）。上流: TinyUSB 0.21.0（2026-06-30、PR #3561 の rp2 の作り直し）はそこで
  panic しない（idle の endpoint の buffer status を無視する）。pico-sdk 2.3.1 と arduino-pico 6.1.1 / 6.2.0 はまだ TinyUSB
  0.18.0（86ad6e56c）。host テスト: test_boot_guard（reset をまたいだ数え方、安全な起動、意図した restart、USB の待ち: 26
  checks）、test_config（待ちが閉じていれば保存した at boot の attach をしない、開けばする。飛ばす起動ではスロットを set し
  直すまで試さない）。ほかは前と同じ。実機ではまだ動かしていない。CHANGELOG (EN / JA)
- (EN) riscv-dm's checked groups cost fewer DMI accesses (bench, 0.0.29-dev+526a881, the V003 jig - classic ESP32,
  SWIO bit-banged, a UART bridge; ch32rv's upload of a 9168-byte image, traced A/B against 3c0cd99 with no broker
  between: the same 86 riscv-dm requests took 0.77 s longer on the probe - write_block 122 words 21.3 -> 36.9 ms,
  write_block 6 words 9.4 -> 17.0 ms, run 57.6 -> 66.7 ms, read_block 122 words 21.2 -> 28.4 ms, x18 each. Fitted to
  the access counts: a read about 70 us, a write about 41 us on this PHY - every read is a whole frame, a second one
  too. Of the 0.77 s: the block ops' fixed cost about 0.47 s, write_block's DATA1 check after every word about 0.15 s,
  run about 0.17 s, the console's DMSTATUS look (3 reads instead of 1 every 20 ms while a slot's console is open)
  about 0.015 s. The rest of that traced run's 6.56 s - 3.0 s with no answer to one read_block and the reads at the
  boot speed after the host fell back - is the UART link: answers of 491 bytes came broken at 921600 in both
  firmwares, 6 of 18 with 3c0cd99). A value the probe has just written or puts back, known to it, is read back once
  against it, the read before it made sure not to be that value - a DMCONTROL read, and DMSTATUS when that is it too,
  put between when it is (Ch32Dm::readIs; DmiPhy::lastRead) - and an abstract register over the value inverted in
  DATA0, written twice (Ch32Dm::readRegisterIs): a lost write and a read missed can no longer agree on it, as two
  missed reads could not before; all ones (a line's own value) is still read twice. Used for the GPRs and the mailbox
  given back, the run's GPRs and dpc, write_block's first word in DATA0 and ABSTRACTAUTO = 0; what is kept or
  answered (an unknown value) and dcsr's bits are read twice as before. ABSTRACTAUTO seen 0 in an op is not written
  and read again until the op itself writes it (auto_seen_off_: only a write sets it, and inside an op only the probe
  writes it), and keepAuto writes nothing when it reads 0. DMI accesses (test_wire's fake; ch32rv's loader run with
  a0-a3, a5 and mstatus in, a0 out): read_block 182 + n -> 136 + n, write_block 191 + 2n -> 139 + 2n (1 word: 193 ->
  137), run 230 -> 188 (3c0cd99: 71 + n, 78 + n, 74); the upload estimated 0.27 s shorter on the probe, about 2.6 s
  where 3c0cd99 took 2.07 s. Host tests (test_wire): the block ops with glitches in pairs also with values alike what
  a missed read gives or a line reads (s0 = 2 - ABSTRACTCS here, the bench's a1 -> 0x00000002 -, a0 = 1, a1 all ones,
  DATA0 a halted DMSTATUS, DATA1 0, words all ones and 2) - 0 of 92928 wrong; a run with glitches in pairs, also with
  a0 = 1 and a1 = 2 in: 0 of 36992 wrong (16 answer a failure); without the read put between, 4 / 2 wrong there; the
  drop-inside block-op cases' values no longer all ones in the counting run (every drop met); the rest as before.
- (JA) riscv-dm の確かめたまとまりの DMI アクセスを減らす（bench、0.0.29-dev+526a881、V003 の治具 - classic ESP32、ソフトで
  叩く SWIO、UART bridge。ch32rv による 9168 バイトのイメージの書き込みを、間にブローカーを置かずに 3c0cd99 と A/B で trace:
  同じ 86 の riscv-dm の要求が probe 側で 0.77 秒長くかかった - 122 語の write_block 21.3 -> 36.9 ms、6 語の write_block
  9.4 -> 17.0 ms、run 57.6 -> 66.7 ms、122 語の read_block 21.2 -> 28.4 ms、それぞれ 18 回。アクセスの数に合わせると、この
  PHY で読み 1 回が約 70 us、書き 1 回が約 41 us - 読みは毎回まるごと 1 フレーム、2 回目の読みも同じ。0.77 秒のうち、block op
  の固定の分が約 0.47 秒、write_block の 1 語ごとの DATA1 の確かめが約 0.15 秒、run が約 0.17 秒、コンソールの DMSTATUS の
  確かめ（スロットのコンソールが開いている間 20 ms ごとに 1 回でなく 3 回の読み）が約 0.015 秒。その trace の 6.56 秒の残り -
  read_block の 1 つに答えが来なかった 3.0 秒と、host が起動時の速さに戻ったあとの読み - は UART の link のもの: 491 バイトの
  答えが 921600 で壊れていた、どちらの firmware でも（3c0cd99 で 18 回中 6 回））。probe が書いたばかりの値・戻す値（probe が
  知っている値）は、それと比べて 1 回だけ読み戻す。その前の読みがその値でないことを確かめてから - そうなら DMCONTROL の読みを、
  それも同じなら DMSTATUS の読みを間に入れる（Ch32Dm::readIs、DmiPhy::lastRead）。抽象レジスタは値を反転した sentinel を
  DATA0 に 2 回書いてから（Ch32Dm::readRegisterIs）。書き込みの消失と取りこぼした読みがそろってその値になることはない - 前に
  取りこぼした 2 回の読みがそろわなかったのと同じ。全部 1（何もつながっていない線の値）は今までどおり 2 回読む。使うのは、戻す
  GPR とメールボックス、run の GPR と dpc、write_block の最初の語の DATA0、ABSTRACTAUTO = 0。取っておく値・答える値（わからない
  値）と dcsr のビットは今までどおり 2 回読む。op の中で 0 と確かめた ABSTRACTAUTO は、op 自身が書くまでは書き直しも読み直しも
  しない（auto_seen_off_: 立てるのは書き込みだけで、op の中で書くのは probe だけ）。keepAuto は 0 と読めたら何も書かない。DMI
  アクセス（test_wire の偽物。ch32rv の loader の run は a0-a3、a5、mstatus を入れ a0 を出す）: read_block 182 + n -> 136 + n、
  write_block 191 + 2n -> 139 + 2n（1 語: 193 -> 137）、run 230 -> 188（3c0cd99: 71 + n、78 + n、74）。書き込みは probe 側で約
  0.27 秒短くなる見込み、3c0cd99 の 2.07 秒に対して約 2.6 秒。host の試験（test_wire）: block op にグリッチを 2 つずつ、取りこぼした
  読みが返す値や線の値に似た値でも（s0 = 2 - ここでの ABSTRACTCS、bench の a1 -> 0x00000002 -、a0 = 1、a1 = 全部 1、DATA0 = 止まった
  hart の DMSTATUS、DATA1 = 0、語は全部 1 と 2）- 92928 回中 0 回の誤り。run にグリッチを 2 つずつ、a0 = 1、a1 = 2 を入れる場合も:
  36992 回中 0 回の誤り（16 回は失敗を答える）。間に入れる読みを外すと、それぞれ 4 回 / 2 回の誤り。落ちる link の block op の試験の
  数える回の値が全部 1 でなくなった（どの落ち方も op に当たる）。ほかは今までどおり。
- (EN) Fix: riscv-dm run on a CH32V003 (bench, 0.0.29-dev+82e1ec9, the V003 jig - classic ESP32, SWIO, ch32rv 9cb4f08:
  every upload, 5 of 5, failed "run: timeout" at the loader's run, stopped 2; 3c0cd99 passed). ch32rv's loader run sets
  mstatus = 0, and the V003's mstatus reads 0x00001800 before and after that write (read on the jig: MPP fixed at M,
  WARL). e7b903c's check of a register other than a GPR or dpc (read before, written, read back) took a value read back
  as it was, and not as asked, for a lost write: every try of the set-up failed and the run never started - answered
  stopped 2 (could not be halted), the form an early return of runUntilHalt takes. Parts with U mode (X035, L103) read
  back 0. Now (Ch32Dm::writeRegisterTaken) such a register is written again and read back again (each read twice over
  the sentinels): the same value twice is what the register holds once a write of it landed - one missed access cannot
  lose both writes; a second read back that differs (the first write lost) fails the group, whose redo reads the
  register as it now is. x0 (0x1000) goes the same way (it reads 0 whatever is written); x1..x31 and dpc are read back
  whole as before. Host tests (test_wire; the fake's mstatus with MPP fixed or writable, and a run that reaches its
  ebreak only with mstatus.MIE clear): a run with mstatus = 0 in, MPP fixed / writable, 0x1800 / 0x1888 / 0 before,
  with a glitch at every access (cmderr 6 set or not): 0 of 1524 wrong (before: 595 of 1692 answered timeout, stopped
  2); the run with a drop anywhere inside now also sets mstatus = 0 (MIE set before) on both kinds: 0 of 4124 (before:
  84 timeouts - a redo after a drop read 0x1800 as it was). The cost of a GPR-only run is unchanged.
- (JA) 修正: CH32V003 での riscv-dm の run（bench、0.0.29-dev+82e1ec9、V003 の治具 - classic ESP32、SWIO、ch32rv 9cb4f08: どの
  書き込みも 5 回中 5 回、loader の run で "run: timeout"、stopped 2 で失敗。3c0cd99 では通っていた）。ch32rv の loader の run は
  mstatus = 0 を設定し、V003 の mstatus はその書き込みの前も後も 0x00001800 と読める（治具で読んだ: MPP が M に固定、WARL）。
  e7b903c の GPR と dpc 以外のレジスタの確かめ（前に読み、書き、読み戻す）は、頼んだ値でなく前のままに読み戻った値を、書き込み
  の消失と取っていた: 準備のどの試みも失敗し、run は始まらず、stopped 2（止められなかった）と答えていた。runUntilHalt の早い
  return がとる形。U モードのある部品（X035、L103）では 0 が読み戻る。いまは（Ch32Dm::writeRegisterTaken）そういうレジスタは
  もう一度書いてもう一度読み戻す（どちらの読みも sentinel を挟んで 2 回）: 2 回とも同じ値なら、それがその値の書き込みが通った
  あとのレジスタの値 - 1 回のアクセスの取りこぼしでは 2 回の書き込みを両方失えない。2 回目の読み戻しが違えば（1 回目の書き込みが
  消えた）まとまりを失敗にし、やり直しがその時のレジスタを読む。x0（0x1000）も同じ扱い（何を書いても 0 と読める）。x1..x31 と
  dpc は今までどおり全体を読み戻す。host の試験（test_wire。偽物の mstatus は MPP 固定または書ける、run は mstatus.MIE が 0 の
  ときだけ ebreak に着く）: mstatus = 0 を渡す run を、MPP 固定 / 書ける、前の値 0x1800 / 0x1888 / 0、すべてのアクセスで
  glitch（cmderr 6 あり / なし）: 1524 通り中 0 が誤り（前: 1692 中 595 が timeout、stopped 2）。中のどこかで link が落ちる run
  も、両方の種類で mstatus = 0（前は MIE が立っている）を渡す: 4124 中 0（前: 84 が timeout - 落ちた後のやり直しが 0x1800 を
  前のままと読んだ）。GPR だけの run の費用は変わらない。
- (EN) The same discipline as the block ops (a value acted on, answered or written from is read twice and taken only
  when both agree) everywhere else the probe acted on one DMI read, for a link that misses a single access and is up
  again at the next (a glitch: a write lost, a read answering the read before it). The console (DmConsole): a word it
  acts on - takes bytes from, answers, writes 0 over - is read again before it acts: DATA0 (the poll's read), DATA1 when
  the frame reaches it, another register (SDI: DMSTATUS, whose low byte is no length; DMDATA / dmseq: DMCONTROL, bit 7
  clear), DATA1 again, DATA0 again, both pairs agreeing; not agreeing, nothing is done and the next poll reads again.
  Read once, a stale DATA0 (the frame already taken, a DATA1 of bytes, DMSTATUS) was taken as a new frame - its bytes
  twice and the receipt / answer written over the frame the target had just posted (lost) - or DMSTATUS (bit 7, L 2) as
  a DMDATA word to clear over the target's slot; dmseq had only its CRC-8 against a stale word. The DMSTATUS look every
  console_dmstatus_poll_ms is read twice (DMCONTROL between; not agreeing: asked again at the next poll). An idle poll
  still costs one read; a frame 4 / 6 accesses instead of 2 / 3 (with the answer), the DMSTATUS look 3 instead of 1. A
  lost write stays a declared limit of SDI and DMDATA, which carry no sequence number: a lost receipt / answer cannot be
  told from the same bytes posted again, so one frame's bytes come twice (DMDATA: that answer's input bytes lost); dmseq
  covers it. Ch32Dm: checkHalted, halt's "already halted" and ackHaveReset read DMSTATUS twice (readStatusSure:
  DMSTATUS, DMCONTROL, DMSTATUS); the end of each wait for a change of hart state (halt, resume, run, step, the resets'
  halt, awaitModule, attach under reset) is seen twice (statusConfirms: DMCONTROL, DMSTATUS once more). riscv-dm /
  the wire: attach's answered DMSTATUS and its halt decision, attachRunning's, scan's live pair and target_id
  (readDmiSure, also what a slot's lock is checked against) read twice; whether a module answers (attach's revive of a
  live link - a re-attach may wake and so restart an L103 -, scan's live pair, the failure status of an op) is either of
  two DMSTATUS reads answering (moduleAnswersSure). Already safe, unchanged: scan of a pair not attached (both PHYs read
  DMCONTROL just before DMSTATUS: a missed read gives version 1, no module), the slot's liveness check and the wire-loss
  clock (wire_lost_ms of failures with no good exchange: one read cannot close a connection), selectHart0 (its one write
  is hart 0 / dmactive, what the op wants anyway; a missed selection fails the op's looks), waitAbstract (a stale
  ABSTRACTCS only fails a group, and every value through it is read back or counted), readWords' run count (a stale
  DATA1 only fails the check). Host tests (test_wire, the fake's glitch at one access, singly and in pairs 1-3 apart): the
  console's three mechanisms against targets that play by their rules, at every access of a 13-frame run with input
  both ways - with only reads missed 0 of 931 / 1916 / 1930 cases wrong for SDI / DMDATA / dmseq (before: 46 of 698 / 42
  of 521 / 0 of 399), dmseq 0 of 798 with a write missed, SDI 65 of 65 and DMDATA 66 of 500 with a write missed (the
  declared limit; before 78 / 78); attach (method 0 on a live connection, a running hart), halt, resume that never
  takes, read_block over a running hart and scan of the live pair with a misleading word read before them (halted-looking
  with and without havereset, 0, all ones): 0 of 5856 wrong (before: 52 of 5488 - attach 36: the hart halted, a revive,
  a restart counted or a wrong DMSTATUS / target_id answered; scan 16; halt 1). Costs (the fake, DMI accesses):
  read_block 182 + n (805645b 180 + n, 4c310a2 71 + n), write_block 191 + 2n (189 + 2n, 78 + n), run 166 (162, 67),
  step 144 (140, 61), attach joining a live connection 16 (7).
- (JA) block op と同じ規律（それを元に動く・答える・書く値は 2 回読み、両方が一致したときだけ採る）を、probe が 1 回の DMI の
  読みで動いていたほかのすべての所に当てる。1 回だけアクセスを取りこぼして次にはまた繋がっている link（glitch: 書き込みが消える、
  読みが前の読みの値を返す）に対して。コンソール（DmConsole）: 動く元になる word - byte を取る、答える、0 を書く - は動く前に読み
  直す: DATA0（poll の読み）、frame が届けば DATA1、別のレジスタ（SDI: 下の byte が長さにならない DMSTATUS。DMDATA / dmseq:
  bit 7 が 0 の DMCONTROL）、DATA1 をもう一度、DATA0 をもう一度、両方の組が一致すること。一致しなければ何もせず、次の poll で読み
  直す。1 回の読みでは、古い DATA0（取り終えた frame、byte の入った DATA1、DMSTATUS）を新しい frame と取り - byte を 2 回、target
  が出したばかりの frame の上に受け取り / 答えを書いて（失う） - あるいは DMSTATUS（bit 7、L 2）を DMDATA の消す word と取って
  target の枠の上に 0 を書いていた。dmseq は古い word に対して CRC-8 しか無かった。console_dmstatus_poll_ms ごとの DMSTATUS も
  2 回読む（間に DMCONTROL。一致しなければ次の poll で聞き直す）。何も無い poll は今までどおり 1 回の読み。frame は（答えを含めて）
  2 / 3 回でなく 4 / 6 回、DMSTATUS の見に行きは 1 回でなく 3 回。書き込みの消失は、順番の番号を持たない SDI と DMDATA の宣言した
  限界のまま: 消えた受け取り / 答えは同じ byte がまた出されたのと見分けられず、1 つの frame の byte が 2 回来る（DMDATA: その答えの
  入力の byte は失う）。dmseq はそれを覆う。Ch32Dm: checkHalted、halt の「もう止まっている」、ackHaveReset は DMSTATUS を 2 回読む
  （readStatusSure: DMSTATUS、DMCONTROL、DMSTATUS）。hart の状態の変化を待つ所の終わり（halt、resume、run、step、reset の halt、
  awaitModule、reset 中の attach）は 2 回見る（statusConfirms: DMCONTROL、もう一度 DMSTATUS）。riscv-dm / 線: attach が答える
  DMSTATUS と halt するかの判断、attachRunning、scan の生きている組、target_id（readDmiSure。slot の lock もこれと比べる）は 2 回
  読む。module が答えるか（attach の生きた link の立て直し - attach し直すと wake で L103 が再起動しうる -、scan の生きている組、
  op の失敗の status）は、2 回の DMSTATUS の読みのどちらかが答えればよい（moduleAnswersSure）。もともと安全で変えない所: attach
  していない組の scan（どちらの PHY も DMSTATUS の直前に DMCONTROL を読む: 取りこぼした読みは version 1 で module ではない）、slot
  の生存確認と線の喪失の時計（wire_lost_ms の間良い交換が無いこと: 1 回の読みでは connection は閉じない）、selectHart0（書くのは
  hart 0 / dmactive で、op がどのみち欲しいもの。選び損ねは op の確認で失敗になる）、waitAbstract（古い ABSTRACTCS はまとまりを
  失敗させるだけで、通る値はすべて読み戻すか数える）、readWords の実行回数（古い DATA1 は確認を失敗させるだけ）。host の試験
  （test_wire、偽物の 1 回のアクセスの glitch、1 つずつと 1〜3 離れた 2 つ）: コンソールの 3 つの方式を規則どおりの target に対して、
  両方向の入出力がある 13 frame の流れのすべてのアクセスで - 読みだけが取りこぼされたとき SDI / DMDATA / dmseq で 931 / 1916 /
  1930 通り中 0 が誤り（前: 698 中 46 / 521 中 42 / 399 中 0）、書き込みの消失を含むとき dmseq は 798 中 0、SDI は 65 中 65、
  DMDATA は 500 中 66（宣言した限界。前 78 / 78）。attach（生きた connection に method 0、hart は走っている）、halt、効かない
  resume、走っている hart の read_block、生きている組の scan を、直前に紛らわしい word（havereset の有無の止まった風、0、全部 1）を
  読ませて: 5856 通り中 0 が誤り（前: 5488 中 52 - attach 36: hart が止まった、立て直し、再起動を数えた、誤った DMSTATUS /
  target_id を答えた。scan 16。halt 1）。費用（偽物、DMI のアクセス数）: read_block 182 + n（805645b 180 + n、4c310a2 71 + n）、
  write_block 191 + 2n（189 + 2n、78 + n）、run 166（162、67）、step 144（140、61）、生きた connection に加わる attach 16（7）。
- (EN) oep.probe.restart takes a USB device off the bus before the chip resets (bench, 0.0.29-dev+3c0cd99: after a restart
  the RP2350 came back failing its device descriptor request - Windows: "unknown USB device (device descriptor request
  failed)" - until a replug; the WeAct ESP32-P4 the same after a DFU update's reboot). The reset came with the device
  still on the bus, the host mid-transfer: RP2 reset at once (rp2040.reboot), the P4 20 ms after tud_disconnect. Now
  oep::platformRestart on an RP2 does tud_disconnect (under the core's USB mutex), waits kRestartDetachMs (60 ms, Oep.h)
  and then rp2040.reboot() (its watchdog fires 10 ms on, kRestartResetMs); the P4's handler does tud_disconnect,
  kRestartDetachMs, esp_restart; a classic ESP32 (USB through a bridge that stays on the bus) only esp_restart. The
  endpoint calls the handler kRestartSettleMs (20 ms) after the answer, so the reset starts about 90 ms after it - within
  restart_after_answer_ms (100), checked by a static_assert and the host test (the wait is 60 ms, not 100, to keep that).
  The P4's DFU update restarts from loop(): EspUsbDevice's own restart (tud_disconnect, 20 ms, esp_restart) is off
  (restartWhenComplete false); once the image verified (onComplete), 500 ms for the host's last GETSTATUS, then
  tud_disconnect, 100 ms, esp_restart. Host test (test_core_conformance): the handler's call plus kRestartDetachMs and
  kRestartResetMs ends before restart_after_answer_ms. Not on the bench yet.
- (JA) oep.probe.restart は、chip を reset する前に USB の device を bus から外す（bench、0.0.29-dev+3c0cd99: 再起動の後、RP2350 が
  device descriptor の要求に失敗して戻り - Windows では「不明な USB デバイス（デバイス記述子要求の失敗）」 - 抜き差しが要った。
  WeAct の ESP32-P4 も DFU の更新の後の再起動で同じ）。reset は device が bus に居て host が転送の途中のまま来ていた: RP2 はすぐ
  （rp2040.reboot）、P4 は tud_disconnect の 20 ms 後。いま RP2 の oep::platformRestart は（core の USB の mutex の下で）
  tud_disconnect、kRestartDetachMs（60 ms、Oep.h）待ってから rp2040.reboot()（watchdog は 10 ms 後、kRestartResetMs）。P4 の
  handler は tud_disconnect、kRestartDetachMs、esp_restart。classic ESP32（USB は bus に残る bridge 越し）は esp_restart だけ。
  endpoint は応答の kRestartSettleMs（20 ms）後に handler を呼ぶので、reset は応答のおよそ 90 ms 後に始まる -
  restart_after_answer_ms（100）のうち。static_assert と host の試験で確かめる（待ちを 100 でなく 60 ms にしたのはそのため）。
  P4 の DFU の更新は loop() から再起動する: EspUsbDevice 自身の再起動（tud_disconnect、20 ms、esp_restart）は切り
  （restartWhenComplete false）、image が検証できたら（onComplete）host の最後の GETSTATUS に 500 ms、それから tud_disconnect、
  100 ms、esp_restart。host の試験（test_core_conformance）: handler を呼ぶまでに kRestartDetachMs と kRestartResetMs を足しても
  restart_after_answer_ms の前に終わる。bench ではまだ試していない。
- (EN) riscv-dm's block ops, run, step and the dpc reads stand up to a link that misses a single access and is up
  again at the next one (a glitch: a write lost - the module may take the frame for one with a bad parity and set
  cmderr 6 - or a read answering the value of the read before it), which the look after a held group does not see.
  Bench (0.0.29-dev+4c310a2, tests/hw test_wire on the CH32L103 through the RP2350, oep-client-python eb2e128): a1
  0x20004f6e came back 0x00000002 after a read_block answered ok, read so twice over two sentinels by the host. The op
  keeps s0 / s1 / a0 / a1 from one read each (command, ABSTRACTCS, DATA0) and puts back what it kept, read back; 2 is
  no register's value there but ABSTRACTCS's datacount, the register read just before DATA0 - a missed DATA0 read is
  the likely way in (the whole register reads 0x08000002 on the bench; what a missed read returns exactly is not
  measured). bd19b00's four s1 -> 0x00000002 fit the same, and its a0 -> s1's value is the access-register command lost.
  Now a value that is kept, given back or answered is read twice and taken only when both agree: a register read twice
  with DMCONTROL / DMSTATUS before them (readSure: two missed reads cannot agree), an abstract register read twice over
  sentinels 0 / all ones in DATA0 (readRegisterSure, as oep-client-python's read_register) - abstractauto, the mailbox,
  s0 / s1 / a0 / a1, dpc, dcsr, run's outputs, step's dpcs. Every write an op relies on is read back first: abstractauto
  off (a lost write left a host's autoexec running the last command on the op's DATA0 accesses), the block program's
  set-up before it runs (HARTINFO read twice, a0 / a1, the program buffer, DATA1 = address: a lost a0 / a1 write had the
  program store into the target's memory at its own pointers, a lost program word ran another program), run's dcsr
  (ebreakm, prv), registers and dpc, step's dcsr.step, what goes back (over the sentinels). write_block reads DATA1
  after every store of its autoexec stream and stops at the first that did not move it on: read once at the end, a
  DATA0 write lost in the middle stored every later word one place back and the last one again in the last place, and
  the op answered success. A cmderr 6 met there is a missed access, not the writer's fault: cleared, and the op goes on
  from DATA1's count (cmderr 3 still ends it with fault). A block op leaves no cmderr behind (ABSTRACTCS read twice
  last). checkHalted's re-sync no longer clears abstractauto before the op keeps it. A block op costs more DMI accesses
  (read_block of 8 words 79 -> 188, write_block 86 -> 207). The "cmderr 6" of 4c310a2 runs 5 / 6 ("read_register
  0x100a / 0x1009 failed (cmderr 6)", the host's first read after its halt) is QingKe's "parity bit error during
  communication": a frame of the host's group the module took for a bad one - the same glitch - not a hart that left
  halt (QingKe answers a hart that is not halted with cmderr 4); a host should take it for a failed try and run its
  group again (oep-client-python raised it).
  Host tests (test_wire, the fake's glitch: a write lost - cmderr 6 set or not - or a read giving the read before it, at
  one access, the link up again at once): a glitch at every access of read_block and write_block, singly and in pairs,
  a host's abstractauto set or not - 0 of 790 single and 0 of 78592 in all with state changed, a stray store or wrong
  words (before: 61 of 330 single; 2958 answered ok with a GPR, the mailbox or abstractauto changed - s0 -> 0x00000002
  among them -, 982 with stores outside the block, 793 with wrong words); a glitch at every access of run and step:
  every run stops at its ebreak with its arguments and outputs right, or where it started (the resumereq lost), every
  step answers the right dpcs and leaves dcsr.step clear (before: 18 of 136 runs, 8 of 124 steps wrong); the fakes model
  the program buffer, ABSTRACTAUTO, the registers and dpc / dcsr as read back.
- (JA) riscv-dm のブロック操作、run、step、dpc の読みは、1 回のアクセスだけが抜けて次にはリンクが戻る線（glitch: 書き込みが
  消える - module が parity 誤りの frame と取って cmderr 6 を立てることがある - か、読みが前に読んだ値を返す）に耐える。held
  group の後の見張りではこれは見えない。bench（0.0.29-dev+4c310a2、RP2350 越しの CH32L103 で tests/hw test_wire、
  oep-client-python eb2e128）: read_block が ok で答えた後、a1 0x20004f6e が 0x00000002 になっていた（host が 2 つの番兵で
  2 回読んで同じ値）。操作は s0 / s1 / a0 / a1 をそれぞれ 1 回の読み（command、ABSTRACTCS、DATA0）で保存し、保存した値を
  書き戻して読み戻す。2 はそこでどのレジスタの値でもなく、DATA0 の直前に読む ABSTRACTCS の datacount - DATA0 の読みが抜けた
  のが最もありうる経路（bench ではレジスタ全体は 0x08000002 と読める。抜けた読みが正確に何を返すかは測っていない）。bd19b00 の
  s1 -> 0x00000002 の 4 回も同じ形で、a0 -> s1 の値は access-register の command が消えたもの。今は保存し、書き戻し、答える値は 2 回読み、一致したときだけ取る: レジスタは DMCONTROL / DMSTATUS を
  前に置いて 2 回（readSure: 2 回抜けても誤った値で一致しない）、abstract のレジスタは DATA0 に番兵 0 / 全 1 を置いて 2 回
  （readRegisterSure、oep-client-python の read_register と同じ）- abstractauto、mailbox、s0 / s1 / a0 / a1、dpc、dcsr、run の
  出力、step の dpc。操作が頼る書き込みは、頼る前に読み戻す: abstractauto の 0（書き込みが消えると host の autoexec が操作の
  DATA0 アクセスで前の command を実行していた）、ブロック program の準備（HARTINFO を 2 回、a0 / a1、program buffer、
  DATA1 = address: a0 / a1 の書き込みが消えると program は target 自身のポインタの先の memory に store していた。program の語が
  消えると別の program が走った）、run の dcsr（ebreakm、prv）・レジスタ・dpc、step の dcsr.step、書き戻すもの（番兵越し）。
  write_block は autoexec の流れの中で store ごとに DATA1 を読み、進まなかった最初の所で止まる: 最後に 1 回読むだけだと、途中の
  DATA0 書き込みが消えたとき以降の語が 1 つずつ前にずれて store され、最後の語がもう一度最後の場所に書かれ、操作は success と
  答えていた。そこで会う cmderr 6 は抜けたアクセスで writer の fault ではない: 消して DATA1 の数から続ける（cmderr 3 は今まで
  どおり fault で終える）。ブロック操作は cmderr を残さない（最後に ABSTRACTCS を 2 回読む）。checkHalted の同期の取り直しは、
  操作が abstractauto を保存する前にそれを消さなくなった。ブロック操作の DMI アクセスは増える（8 語の read_block 79 -> 188、
  write_block 86 -> 207）。4c310a2 の 5 / 6 回目の「cmderr 6」（「read_register 0x100a / 0x1009 failed (cmderr 6)」、halt 後の
  host の最初の読み）は QingKe の「通信時の parity 誤り」: host の group の frame を module が誤りと取ったもの - 同じ glitch -
  で、hart が halt を離れたのではない（止まっていない hart に QingKe は cmderr 4 を返す）。host はこれを失敗した試行として group
  をやり直すべき（oep-client-python は例外にしていた）。host test（test_wire。fake の glitch: 1 回のアクセスで書き込みが消える - cmderr 6 あり / なし - か読みが
  前の読みの値を返し、次にはリンクが戻る）: read_block と write_block の全アクセスに 1 つずつ・2 つずつ glitch、host の
  abstractauto あり / なし - 状態が変わる・範囲外の store・誤った語は 1 つの glitch で 790 通り中 0、全体で 78592 通り中 0
  （前: 1 つで 330 通り中 61。2958 通りが GPR・mailbox・abstractauto を変えたまま ok - s0 -> 0x00000002 を含む -、982 通りが
  ブロック外へ store、793 通りが誤った語）。run と step の全アクセスに glitch: run はすべて ebreak で引数と出力が正しく止まるか、
  始めた所で止まる（resumereq が消えた）。step はすべて正しい dpc を答え dcsr.step を 0 に戻す（前: run 136 通り中 18、step
  124 通り中 8 が誤り）。fake は program buffer、ABSTRACTAUTO、レジスタと dpc / dcsr を読み戻せるようにした。
- (EN) README / guides / PID-USE (EN / JA): the spec this implements is oep-spec 498ae95 (the 2026-10-06 structure: the
  nameless core with clock, oep.probe.plan, oep.probe.restart, oep.probe.link, subscribe on the emitting interface, no
  heartbeat), the interface documents under interfaces/; writing a probe: the endpoint's own interfaces listed after the
  sketch's, setRestart before the first poll, an interface offers at least one op, notifies() and the data-only batching.
- (JA) README / ガイド / PID-USE（EN / JA）: 実装している仕様は oep-spec 498ae95（2026-10-06 の構成: clock を持つ名前の無い本体、
  oep.probe.plan、oep.probe.restart、oep.probe.link、送り出すインターフェースへの subscribe、heartbeat なし）、インターフェースの
  文書は interfaces/ にある。probe の書き方: endpoint 自身のインターフェースはスケッチのものの後に出る、setRestart は最初の poll
  の前、インターフェースは op を 1 つ以上持つ、notifies() と、まとめて送るのはデータだけ。
- (EN) The 2026-10-06 structure (breaking; oep-spec 289bde0..498ae95: 2e5dc4c, 0bce222, c475dad, 0304f37, e0d9dc6):
  registry and vectors synced from 498ae95 (tools/sync_registry.sh; ops_encoding.json new). The core has no name: list
  never returns fn 0 (a probe with no interface lists nothing) and fn 0's ops are confirm, list, describe, clock, open,
  end, keepalive and lock_state - no plan, restart or subscription there (op 0x05 / 0x14 / 0x30 / 0x32 on fn 0:
  unknown_operation). clock (0x04, lock-free, no session needed): boot_id(u32) uptime_ns(u64), the clock read while the
  request is handled, just before its answer is built. New interfaces the endpoint lists itself, after every interface
  the sketch added, at the first poll(): oep.probe.plan (ProbePlan: plan_apply 0x01, plan_release 0x02, describe 0x40
  plan_roles) when an interface has plan roles, then oep.probe.restart (ProbeRestart: restart 0x01, describe 0x40
  restart_max_ms) when setRestart gave a handler - which now has to be called before the first poll(); planFn() /
  restartFn() give their fns. The same firmware lists the same fns and instances at every boot; a firmware that adds an
  interface moves these two, and saved settings (named by name, instance, revision) are renumbered as before.
  oep.link is oep.probe.link. subscribe / unsubscribe (0x30 / 0x32: min_bytes(u16) max_delay_ms(u32) [TLV] / [TLV], no
  target fn) are ops of the interface that sends the notifications: Interface::notifies() puts both in its ops and the
  endpoint answers them (the lock holder's); logic (P4 PARLIO and the classic ESP32 sampler), analog and capture-group
  notify, every other fn answers them unknown_operation (was: fn 0 subscribe with a target fn, unsupported for a fn
  that emits nothing). The heartbeat (fn 0 event 1) is gone; features bit2 notify is gone (logic, analog and
  capture-group send no features tag: revision 1 defines no bit). min_bytes / max_delay_ms batch data only; events go as
  soon as the answers ahead of them are sent (they already did). The ops tag is canonical (core §7.4: base the lowest
  op, the last byte non-zero): Endpoint::add refuses an interface that offers no op (no valid encoding). A plan_apply or
  a settings plan item naming a fn whose interface has no plan role (oep.probe.plan itself, a wire) is refused
  unsupported (was: unavailable cause 6). Endpoint::kMaxInterfaces 16 -> 24 (the ESP32-P4 firmware lists 17);
  Endpoint::listHash (unused) removed. Firmware: fns unchanged (oep.probe.link last of the sketch's), then
  oep.probe.plan and oep.probe.restart (restart_max_ms as before: classic ESP32 1500, RP2 2000, P4 3000). Host tests:
  test_vectors runs ops_encoding.json (an independent decoder; the endpoint's ops tag for every valid offerable set is
  the vector's bytes), the new plan / restart / subscribe / clock cases (clock's uptime_ns checked against the fake
  clock), every fn of the vectors' probe canonical; test_core_conformance (fn 0's ops, list without fn 0, restart as
  its own fn: listed only with a handler set before the first poll, an event queued is not sent after the answer);
  test_serial_share (subscribe on the emitting fn, clock, plan_roles in oep.probe.plan's describe, plan_apply's order
  with a fn without plan roles); test_config (the endpoint's two interfaces in the same place at every boot; a saved
  plan renumbered on a firmware with an interface added before it, the endpoint's two moving after it); CHANGELOG
  (EN / JA).
- (JA) 2026-10-06 の構成（破壊的。oep-spec 289bde0..498ae95: 2e5dc4c、0bce222、c475dad、0304f37、e0d9dc6）: registry とベクタを
  498ae95 から写した（tools/sync_registry.sh。ops_encoding.json が増えた）。本体は名前を持たない: list は fn 0 を返さず（インター
  フェースの無い probe は何も載せない）、fn 0 の op は confirm、list、describe、clock、open、end、keepalive、lock_state だけ - plan、
  restart、購読は無い（fn 0 の op 0x05 / 0x14 / 0x30 / 0x32 は unknown_operation）。clock（0x04、ロック不要、セッション不要）:
  boot_id(u32) uptime_ns(u64)。時計は要求を処理する中で、応答を作る直前に読む。endpoint が自分で出す新しいインターフェースを、
  最初の poll() で、スケッチが足したすべてのインターフェースの後に置く: plan の役を持つインターフェースがあれば oep.probe.plan
  （ProbePlan: plan_apply 0x01、plan_release 0x02、describe 0x40 plan_roles）、次に setRestart で handler があれば
  oep.probe.restart（ProbeRestart: restart 0x01、describe 0x40 restart_max_ms）- setRestart は最初の poll() の前に呼ぶ。
  planFn() / restartFn() がその fn を返す。同じ firmware はどの起動でも同じ fn と instance を出す。インターフェースを足した
  firmware ではこの 2 つが動き、保存した設定（name、instance、revision で指す）はこれまでどおり読み替える。oep.link は
  oep.probe.link になった。subscribe / unsubscribe（0x30 / 0x32: min_bytes(u16) max_delay_ms(u32) [TLV] / [TLV]、相手の fn は
  持たない）は通知を送り出すインターフェースの op: Interface::notifies() で両方が ops に入り、endpoint が答える（ロックの持ち主
  のもの）。logic（P4 の PARLIO と classic ESP32 の sampler）、analog、capture-group が送り出す。ほかの fn は unknown_operation
  （以前: fn 0 の subscribe が相手の fn を持ち、何も送らない fn は unsupported）。heartbeat（fn 0 の出来事 1）は無くなった。
  features の bit2 notify も無くなった（logic、analog、capture-group は features の tag を送らない: revision 1 はビットを定めない）。
  min_bytes / max_delay_ms がまとめるのはデータだけで、出来事は先の応答を送り終えたらすぐ送る（もとからそうだった）。ops の
  tag は正規形（core §7.4: base は最小の op、最後の byte は 0 でない）: op を 1 つも持たないインターフェースは Endpoint::add が
  断る（正しい符号が無い）。plan の役を持たないインターフェースの fn（oep.probe.plan 自身、線）を挙げる plan_apply と設定の plan
  の項目は unsupported で断る（以前: unavailable cause 6）。Endpoint::kMaxInterfaces 16 -> 24（ESP32-P4 の firmware は 17 を出す）。
  使われていなかった Endpoint::listHash を消した。firmware: fn は変わらない（スケッチのものの最後が oep.probe.link）。その後に
  oep.probe.plan と oep.probe.restart（restart_max_ms は前と同じ: classic ESP32 1500、RP2 2000、P4 3000）。host test: test_vectors
  が ops_encoding.json を実行（別に書いた decoder。正しくて出せる集合ごとに、endpoint の ops の tag がベクタの byte と一致）、
  新しい plan / restart / subscribe / clock の場合（clock の uptime_ns は偽の時計と照合）、ベクタの probe のすべての fn の ops
  が正規形。test_core_conformance（fn 0 の ops、fn 0 の無い list、別の fn になった restart: 最初の poll の前に handler を置いたとき
  だけ出る、応答の後に溜まっていた出来事は送らない）。test_serial_share（送り出す fn への subscribe、clock、oep.probe.plan の
  describe の plan_roles、plan の役の無い fn を含む plan_apply の断りの順）。test_config（endpoint の 2 つはどの起動でも同じ場所。
  前にインターフェースを足した firmware で保存した plan が読み替わり、endpoint の 2 つはその後ろに動く）。CHANGELOG（EN / JA）。
- (EN) riscv-dm's dmi checks the link per request (the 2026-10-06 debug-link proposal P4; the spec text is not written
  yet) and oep.wire.rvswd's revive hands a link on only once it stays up. Bench (0.0.29-dev+bd19b00, tests/hw test_wire
  on the CH32L103 through the RP2350, oep-client-python af3789a): 5 of 8 runs read s1 or a0 wrong after a read_block
  (s1 0x00002da8 -> 0x00000002, a0 0x200000d4 -> 0x00002da8 = s1's value), with the host's looks around each read
  passing - the access-register command lost and DATA0 still the read before, in the requests where f594f04's revive
  had sent the wake and restarted the target (cmderr 6, 3 of 8). The revive handed the link on at the first DMSTATUS that
  answered; it now asks for 3 good looks in a row (kReviveLooks; a look: DMSTATUS a module's with authenticated, then
  DMCONTROL with bit 7 clear - dmLinkLook), re-syncing again after a bad one, as Ch32Dm::steady does, and its first
  look (is the link still there) is a look too, not one DMSTATUS read. dmi: a look before the steps and one after them,
  and no PHY revive between them (DmiPhy::revives - a wait step longer than the PHY's rest can have one); a request not
  seen held answers status line with no values and done = the steps completed before the check that failed (0 when the
  look before fails; all of them, or up to a timeout's step, when the look after or the count does - their writes may
  have been done). A step that fails on the line answers line with no values too (was: the values read before it). Host
  tests (test_attach_cycle H, the simulated target's drop now coming back through a flicker - one access in 2 - 8
  missed, a write lost or a read giving the value read before it, for 0.5 - 2 ms - and abstract register reads):
  oep-client-python's read_register meeting a 0.7 / 1.4 / 2.1 ms drop: 0 wrong values of 54 (before: 6); a revive
  between a request's looks with the command lost before it answers line, done 10, no values (before: ok with the
  stale DATA0); test_wire's dmi cases follow (a haltreq whose drop the look after meets: line, done 1; a request on a
  line that reads all zeros / ones: line with nothing run, the wire-loss clock as before).
- (JA) riscv-dm の dmi は要求ごとにリンクを確かめる（2026-10-06 の debug-link 提案 P4。仕様の文はまだ）。oep.wire.rvswd の
  立て直しは、リンクが保たれると確かめてから渡す。bench（0.0.29-dev+bd19b00、RP2350 越しの CH32L103 で tests/hw test_wire、
  oep-client-python af3789a）: 8 回中 5 回、read_block の後に s1 か a0 を誤って読んだ（s1 0x00002da8 -> 0x00000002、
  a0 0x200000d4 -> 0x00002da8 = s1 の値）。host の読みの前後の見張りは通っていた - access-register の command が消え、DATA0 は
  前に読んだ値のまま。f594f04 の立て直しが wake を送って target を起動し直していた（cmderr 6、8 回中 3 回）要求で起きた。
  立て直しは最初に答えた DMSTATUS でリンクを渡していた。今は見張り 3 回連続の成功を求め（kReviveLooks。見張り: DMSTATUS が
  module のもので authenticated、続けて DMCONTROL の bit 7 が 0 - dmLinkLook）、失敗したら同期を取り直す（Ch32Dm::steady と
  同じ）。最初の確かめ（リンクがまだあるか）も DMSTATUS 1 回の読みではなく見張り 1 回。dmi: 手順の前と後に見張り、その間に PHY の
  立て直しが無いこと（DmiPhy::revives - PHY の休みより長い待ちの手順があると起こりうる）。保たれたと確かめられない要求は status
  line で答え、値は返さず、done は失敗した確かめの前に終えた手順の数（前の見張りが失敗したら 0、後の見張りか回数が失敗したら
  全部、または timeout の手順まで - その書き込みは行われたかもしれない）。線で失敗した手順も値を返さず line（以前はその前に
  読んだ値を返した）。host test（test_attach_cycle H。模擬 target の落ちはちらつきながら戻る - 0.5 - 2 ms の間、2 - 8 回に
  1 回のアクセスが抜ける（書き込みが消えるか、読みが前に読んだ値を返す）- と、abstract のレジスタ読み）: oep-client-python の
  read_register が 0.7 / 1.4 / 2.1 ms の落ちに当たる: 54 通り中 誤った値 0（前: 6）。要求の見張りの間の立て直し（その前に command
  が消えた）は line、done 10、値なし（前: 古い DATA0 で ok）。test_wire の dmi の場合も合わせた（後の見張りが落ちに当たる haltreq:
  line、done 1。全 0 / 全 1 を読む線への要求: 何も実行せず line、wire-loss の時計は前と同じ）。
- (EN) write_block stores each word exactly once and answers success only with every word stored. ch32rv writes the
  CH32L103's flash keys (KEYR KEY1 / KEY2, MODEKEYR KEY1 / KEY2) as four 1-word write_blocks; in 1 of 60 uploads through
  the RP2350 (f594f04) CTLR stayed locked (0x00008080) after every write_block answered success. The writer was redone
  whole, from its set-up, whenever its check (DATA1's run count, a look) failed - also when the link dropped after the
  store had run: the key went in twice, the wrong sequence, and the redo answered success. Now the set-up (a0 / a1, the
  writer, DATA1 = address) and the first word in DATA0 are each read back over a held link before the writer runs
  (abstractauto off: they store nothing, and are redone freely); from the command on nothing is redone blind - after a
  drop the link is brought up again and DATA1, read over a held link, says how many words were stored (the writer bumps
  it in the same run as its store; a run that faults leaves it), and the op goes on from the first word not stored. A
  cmderr (the store's exception) or a DATA1 that is no count of the op's runs ends it with fault, and write_block's done
  is then the words stored, in order (was 0). Host test (test_wire; the fake counts the hart's stores per address and
  can drop the link before any DMI access, writes too): the link dropped at each access of a 1-word, an 8-word and an
  8-word write_block whose 4th store faults (stale or all ones, held 0 / 0.7 / 2.1 ms): no word stored twice, every
  success with every word stored once, every failure fault / line with done = the words stored, the faulting store fault
  with done 3, GPRs / DATA / abstractauto as found (before: 444 of 912 cases stored a word twice and answered success;
  the faulting store went in 3 times, done 0).
- (JA) write_block は各語をちょうど 1 回書き、すべての語を書いたときだけ success で答える。ch32rv は CH32L103 の flash の鍵
  （KEYR KEY1 / KEY2、MODEKEYR KEY1 / KEY2）を 1 語の write_block 4 回で書く。RP2350 越しの upload 60 回に 1 回（f594f04）、
  write_block がすべて success と答えたのに CTLR がロックのまま（0x00008080）だった。書き手は、確かめ（DATA1 の実行回数、見張り）
  が失敗するたびに準備から丸ごとやり直していた - 書き込みが走った後にリンクが落ちたときも: 鍵が 2 回入り（順序の誤り）、やり直しは
  success と答えた。今は、準備（a0 / a1、書き手、DATA1 = 番地）と DATA0 に入れた最初の語を、書き手を走らせる前に、それぞれ保たれた
  リンクで読み戻して確かめる（abstractauto は off: これらは何も書かないので、自由にやり直す）。command から先は確かめずにやり直さない
  - 落ちの後はリンクを戻し、保たれたリンクで読んだ DATA1 が書けた語の数を示す（書き手は同じ実行の中で書き込みの後に DATA1 を進める。
  例外になった実行は進めない）ので、まだ書いていない最初の語から続ける。cmderr（書き込みの例外）か、この op の実行回数でない DATA1
  では fault で終え、そのときの write_block の done は書けた語の数（順に。以前は 0）。host test（test_wire。fake は hart の書き込みを
  番地ごとに数え、書き込みを含むどの DMI アクセスの前でもリンクを落とせる）: 1 語、8 語、4 語目の書き込みが例外になる 8 語の
  write_block の各アクセスで順にリンクを落とす（stale か全 1、0 / 0.7 / 2.1 ms 保つ）: 2 回書かれた語は無く、success はすべての語を
  1 回ずつ書き、失敗はすべて fault / line で done = 書けた語の数、例外の書き込みは fault で done 3、GPR / DATA / abstractauto は
  元のまま（前: 912 通り中 444 通りで語を 2 回書いて success、例外の書き込みは 3 回試され done 0）。
- (EN) oep.wire.rvswd no longer restarts a CH32L103 under a live connection. The PHY's revive (the first transaction
  after 300 us of rest reads DMSTATUS; a link that does not answer is brought back in step) re-synced once and then sent
  the wake pattern - and the wake restarts the L103 itself, not just its debug interface. Its link drops for 0.7 - 2.1 ms
  after a change of hart state, not always at once, and a re-sync inside that time leaves it down: a host's request that
  came during such a drop (or the console's DMSTATUS look) went to the wakes, the target restarted, and a hart the host
  had halted left halt - tests/hw test_wire on the L103 through the RP2350 (0.0.29-dev+f594f04) failed 3 runs of 8 with
  "read_register 0x100a / 0x1009 / 0x100b failed (cmderr 6)" a few requests after its halt, the hart halted again
  afterwards (haltreq held). The revive now re-syncs, looking again after each, for up to 20 ms (kReviveResyncUs, past
  the drop with margin, as steady's bound) and sends the wakes only to a link still silent then (a target whose power
  went and came back); every re-sync and wake stays inside the request's wire_retry_ms. A revive inside a held group
  could also bring a dropped link up behind it, so the look after the group passed with a write lost before it: the PHY
  counts its revives (DmiPhy::revives) and Ch32Dm::held takes a group whose span saw one for a group that met a drop.
  Host test (test_attach_cycle, the RVSWD PHY on the simulated target, now with an L103's wake restart and its drop held
  against re-syncs): a riscv-dm dmi request after a rest meeting a drop of 0.7 / 1.4 / 2.1 / 5 ms right after attach's
  halt answers the hart halted, no havereset, no wake, no restart (before: 1 / 2 / 3 / 8 wakes, each a restart,
  havereset set); a link only a wake brings back still comes back.
- (JA) oep.wire.rvswd は、生きている connection の下で CH32L103 を再起動しなくなった。PHY の revive（300 us 以上休んだ後の最初の
  やり取りの前に DMSTATUS を読み、答えないリンクを同期し直す）は、1 回同期し直した後に wake のパターンを送っていた - そして wake は
  L103 のデバッグの口だけでなく L103 そのものを再起動する。L103 のリンクは hart の状態が変わった後 0.7 - 2.1 ms 落ち（すぐとは
  限らない）、その間の同期し直しでは戻らない: その落ちの間に来た host の要求（やコンソールの DMSTATUS の見張り）が wake に進み、
  target が再起動し、host が止めた hart が止まった状態を出た - RP2350 越しの L103 での tests/hw test_wire（0.0.29-dev+f594f04）は
  8 回中 3 回、halt の数要求後に「read_register 0x100a / 0x1009 / 0x100b failed (cmderr 6)」で失敗し、その後 hart はまた止まって
  いた（haltreq を保っているため）。revive は、同期し直してはもう一度見ることを 20 ms まで（kReviveResyncUs、落ちより余裕を持って
  長く、steady の上限と同じ）繰り返し、wake はそれでも答えないリンク（電源が落ちて戻った target）にだけ送る。同期し直しと wake は
  すべて要求の wire_retry_ms の中。held の組の中の revive は、落ちたリンクを組の裏で戻し、組の後の見張りが、その前に失われた書き
  込みがあっても通ってしまう: PHY は revive を数え（DmiPhy::revives）、Ch32Dm::held はその間に revive があった組を落ちに会った組と
  する。host test（test_attach_cycle: RVSWD の PHY と模擬 target。L103 の wake での再起動と、同期し直しに対して保たれる落ちを
  足した）: attach の halt の直後、休みの後の riscv-dm dmi の要求が 0.7 / 1.4 / 2.1 / 5 ms の落ちに会っても、hart は止まったまま、
  havereset 無し、wake 無し、再起動無しで答える（前: wake 1 / 2 / 3 / 8 回、そのたびに再起動、havereset が立つ）。wake でしか戻ら
  ないリンクは、これまでどおり戻る。
- (EN) The L103's "run: timeout" (bench, 0.0.29-dev+9942787 with steady(): 2 of about 12 ch32rv uploads through the
  RP2350, print_format and wire_selftest, as before steady) shares the a0 clobber's cause: a drop that comes later than
  steady's looks, met by the run's set-up. With the link down a write is lost and a read gives the last value read; a
  stale ABSTRACTCS read that showed neither busy nor cmderr made a lost write of dcsr (ebreakm) or dpc look done, the
  hart resumed into the application instead of the loader, and the run waited out its timeout. The held groups above
  (the set-up's writes, then a look, redone after a drop) cure it; and checkHalted, which every op runs first, now
  looks from a link that stays up (steady) instead of after one relink - a drop held against that relink answered line,
  or state on a stale "running", without running the op. Host test (test_wire): the fake's loader is reached only from
  dpc = pc with ebreakm set (else the hart runs on); the link dropped at every read of a run in turn (stale or all ones,
  held 0 / 0.7 / 2.1 ms): all 1600 runs stop at their ebreak with the arguments in place (before: a timeout, wrong
  arguments, 77 other failures in 1735); the block-op sweep's 558 cases all answer ok.
- (JA) L103 の「run: timeout」（ベンチ、steady() 入りの 0.0.29-dev+9942787: RP2350 越しの ch32rv の upload 約 12 回に 2 回、
  print_format と wire_selftest、steady の前と同じ率）は、a0 の書き換わりと原因が同じ: steady の見張りより遅れて来る落ちに、
  run の準備が会う。リンクが落ちている間は書き込みが失われ、読み出しは最後に読んだ値を返す。busy も cmderr も示さない古い
  ABSTRACTCS の読み出しが、失われた dcsr（ebreakm）や dpc の書き込みを済んだように見せ、hart は loader でなくアプリケーションに
  戻り、run は timeout まで待っていた。上の held の組（準備の書き込みの後に 1 回見て、落ちに会えばやり直す）で直る。また、
  どの op も最初に行う checkHalted は、relink 1 回の後でなく、立ち続けるリンク（steady）から見る - relink に対して保たれた落ちは
  line を、古い「走っている」は state を返し、op は走らなかった。host test（test_wire）: fake の loader は dpc = pc かつ
  ebreakm のときだけ届く（ほかは hart が走り続ける）。run の読み出しの 1 つずつで順にリンクを落とす（stale か全 1、
  0 / 0.7 / 2.1 ms 保つ）: 1600 回の run がすべて ebreak で止まり、引数も正しい（前: 1735 回中 timeout 1、引数の誤り、ほかの
  失敗 77）。block の op の 558 通りもすべて ok。
- (EN) oep.wire.swio's speed: describe declared min_clock_hz 888888 (a zero's slot, 862.5 + 262.5 ns, oep-if-debug
  §3.2) while a connection's speed_hz was the rate one read's wall time implies over its 41 slots - the read slots'
  waits for the line and the frame's set-up counted in: 732142 Hz on the ESP32-P4 bench (no max_speed, 2 MHz, 1 MHz),
  745454 Hz (900 kHz), so a max_speed of 800 kHz was refused while the connections said they ran slower. The link has
  one speed: speed_hz (attach's answer, connections) is that speed, and describe declares it as min_clock_hz and
  max_clock_hz (new `DmiPhy::maxClockHz()`, 0 = not declared; a fixed-speed link: min = max). A max_speed at or above it
  is taken, one under it refused unsupported, as before. RVSWD (a speed chosen by reading) reports the measured rate,
  at most the bit clock max_speed caps: unchanged. Host test (test_swio): describe min = max = 888888, the attach answer
  888888 with max_speed 2 MHz / 1 MHz / 900 kHz / 888888, 800 kHz and 888887 refused unsupported, an attach without max_speed malformed.
- (JA) oep.wire.swio の速さ: describe は min_clock_hz 888888（0 の区切り、862.5 + 262.5 ns、oep-if-debug §3.2）を宣言し、
  connection の speed_hz は読み出し 1 回の実時間を区切り 41 個で割った速さだった - 読み出しの区切りの線の待ちとフレームの
  準備を含む: ESP32-P4 のベンチで 732142 Hz（max_speed 無し、2 MHz、1 MHz）、745454 Hz（900 kHz）。そのため max_speed
  800 kHz は、connection がそれより遅いと言っているのに断られた。リンクの速さは 1 つ: speed_hz（attach の答え、
  connections）はその速さで、describe はそれを min_clock_hz と max_clock_hz で宣言する（新しい `DmiPhy::maxClockHz()`、
  0 = 宣言しない。速さが決まったリンクは min = max）。それ以上の max_speed は受け、下は unsupported で断る（変わらず）。
  RVSWD（読んで速さを選ぶ）は測った速さを返し、max_speed が抑えるビットのクロック以下: 変えない。host test（test_swio）:
  describe の min = max = 888888、max_speed 2 MHz / 1 MHz / 900 kHz / 888888 で attach の答えは 888888、800 kHz と 888887 は
  unsupported、max_speed 無しは malformed。
- (EN) Classic ESP32 GPIO34 / 35 were still offered to the driving roles: Firmware/OepProbe's input-only mask was
  `0xf0 << 32` - GPIO36-39 - so 5b7ffc5 left 34 / 35 in the role_channels of I2C SDA / SCL, UART TX, SPI MISO and
  attach's reset, and their plan checks (which follow the same mask) took them: tests/hw test_i2c_target (V003 jig)
  picked SDA 34 / SCL 35 from role_channels, plan_apply completed, configure answered completed failed. The plan checks
  and the declarations agreed; the mask was wrong. The mask is the library's now, `oep::kEsp32InputOnlyPins` (GPIO34-39)
  and `oep::platformInputOnlyPins()` (it on a classic ESP32, 0 elsewhere): the firmware uses it, and the FixtureProbe
  example marks those pins too (it offered them to UART TX). Checked every interface's plan check against its
  role_channels - gpio line, UART RX / TX, I2C SDA / SCL, SPI SCK / MOSI / MISO / CS, logic / sampler roles, analog
  inputs, the wires' pins and reset - on the classic ESP32, the ESP32-P4 and the RP2: each refuses unsupported exactly
  the channels its describe leaves out (the P4 and the RP2 have no input-only pins). Host tests: on the classic ESP32
  firmware's channel table with the library mask, planCheck agrees with role_channels channel by channel for I2C, SPI,
  UART and gpio; SDA 34 / SCL 35 is refused unsupported.
- (JA) classic ESP32 の GPIO34 / 35 が、まだ駆動する role に出ていた: Firmware/OepProbe の入力専用のマスクが `0xf0 << 32`
  - GPIO36-39 - で、5b7ffc5 のあとも 34 / 35 が I2C SDA / SCL、UART TX、SPI MISO、attach の reset の role_channels に残り、
  同じマスクに従う plan の確かめもそれを受けていた: tests/hw test_i2c_target（V003 の治具）が role_channels から SDA 34 /
  SCL 35 を選び、plan_apply は completed、configure は completed failed だった。plan の確かめと宣言は一致しており、マスクが
  誤っていた。マスクはライブラリのものにした: `oep::kEsp32InputOnlyPins`（GPIO34-39）と `oep::platformInputOnlyPins()`
  （classic ESP32 でそれ、ほかは 0）。ファームはこれを使い、FixtureProbe の例もこのピンに印を付ける（UART TX に出していた）。
  すべての interface の plan の確かめを role_channels と突き合わせた - gpio の線、UART RX / TX、I2C SDA / SCL、SPI SCK /
  MOSI / MISO / CS、logic / sampler の role、analog の入力、wire のピンと reset - classic ESP32、ESP32-P4、RP2 で: どれも
  describe が外したチャンネルをちょうど unsupported で断る（P4 と RP2 に入力専用のピンは無い）。host test: classic ESP32 の
  ファームのチャンネル表とライブラリのマスクで、I2C、SPI、UART、gpio の planCheck がチャンネルごとに role_channels と
  一致する。SDA 34 / SCL 35 は unsupported。
- (EN) riscv-dm on a link that drops late after a change of hart state (CH32L103): what a block op keeps and gives
  back, and the register accesses run / step / the resets act on, are done in held groups - each followed by a look at
  the link (DMSTATUS a module's with authenticated set, then DMCONTROL with dmactive and hart 0 selected: a dropped link
  reads all ones, or the same last value for both, which cannot pass both); a group that failed or met a drop is redone
  from a link brought up again (steady), 4 tries at most. read_block / write_block keep abstractauto (redone without
  clearing it), the mailbox and s0 / s1 / a0 / a1 that way, and give back the GPRs, the mailbox and abstractauto with
  each read back and compared; an op whose GPRs could not be seen back answers no words (fault), and one that could not
  keep them does not run. Bench (0.0.29-dev+9942787, L103 through the RP2350, tests/hw test_wire): in about 1 of 200
  read_blocks a0 0x000ec8fe came back 0x20000000 with s0 / s1 / a1 unchanged - a drop met while the GPRs were kept: the
  command reading a0 was lost and the DATA0 read gave the last value read (s1's), which was kept for a0 and written
  back as it. The same for run (dcsr read; dcsr, the host's registers and dpc written; dpc, the out registers and the
  dcsr fix-up read back), step (dpc / dcsr read, dcsr.step set and cleared), the mailbox around every op, and the dpc
  read of attach / reset (`Ch32Dm::readDpc`). Retries inside the block ops use steady instead of one relink. Host test
  (test_wire): the link dropped at every read of read_block and write_block in turn (stale or all ones, held 0 / 0.7 /
  2.1 ms, writes lost, a host's abstractauto set or not; the fake's program buffer now goes through a0 / a1 as they are
  and counts stores outside the block): every GPR, dpc, dcsr, DATA0 / DATA1 and abstractauto as found and no stray store
  in all 558 cases (before: 122 of 294 changed - a0 / a1 taken from a stale read, the mailbox lost), an ok answer always
  with the right words; the link-drop section and the upload loop check all 31 GPRs after every block op and the run's
  arguments (100 uploads, and 800 with other seeds, none failed).
- (JA) hart の状態が変わった後、遅れて落ちるリンク（CH32L103）での riscv-dm: block の op が取っておいて戻すものと、run / step /
  reset が触るレジスタのアクセスを、保たれた組（held）で行う。組のあとにリンクを 1 回見る（DMSTATUS が module のもので
  authenticated が立ち、続く DMCONTROL が dmactive と hart 0 の選択: 落ちたリンクは全 1 か、両方に同じ最後の値を返し、両方は
  通らない）。失敗した組、落ちに会った組は、リンクを立て直して（steady）やり直す（最大 4 回）。read_block / write_block は
  abstractauto（やり直しはそれを消さない）、mailbox、s0 / s1 / a0 / a1 をこうして取っておき、GPR、mailbox、abstractauto を
  戻して、それぞれ読み戻して比べる。GPR が戻ったと確かめられない op は語を返さない（fault）。取っておけない op は走らない。
  ベンチ（0.0.29-dev+9942787、RP2350 越しの L103、tests/hw test_wire）: read_block の約 200 回に 1 回、a0 0x000ec8fe が
  0x20000000 になり、s0 / s1 / a1 は変わらなかった - GPR を取っておく途中で落ちに会い、a0 を読むコマンドが失われ、DATA0 の
  読み出しが最後に読んだ値（s1 のもの）を返し、それを a0 として取っておいて書き戻していた。同じことを run（dcsr の読み出し、
  dcsr・host のレジスタ・dpc の書き込み、dpc・出力のレジスタ・dcsr の戻しの読み出し）、step（dpc / dcsr の読み出し、
  dcsr.step を立てて下ろす）、各 op の mailbox、attach / reset の dpc の読み出し（`Ch32Dm::readDpc`）にも行う。block の op の
  中のやり直しは relink 1 回ではなく steady を使う。host test（test_wire）: read_block と write_block の読み出しの 1 つずつで
  順にリンクを落とす（stale か全 1、0 / 0.7 / 2.1 ms 保つ、その間の書き込みは失われる、host の abstractauto あり / なし。
  fake の program buffer は a0 / a1 をそのまま通り、block の外への store を数える）: 558 通りすべてで GPR 全部・dpc・dcsr・
  DATA0 / DATA1・abstractauto が元のまま、外への store なし（前: 294 通り中 122 で変わった - stale の読み出しからの a0 / a1、
  失われた mailbox）、ok の答えはいつも正しい語。リンクの落ちの節と upload のループは、block の op のたびに GPR 31 本と run の
  引数を確かめる（upload 100 回、別の seed で 800 回、失敗なし）。
- (EN) fn 0 `restart` (op 0x14) and `restart_max_ms` (describe 0x4F) - oep-spec ecd1ab9 and 3c96daf (core §6.6, §7.5,
  §12; transports §1): `Endpoint::setRestart(fn, max_ms)` puts restart in fn 0's ops and restart_max_ms in its
  describe (raised to `restart_after_answer_ms`, 100, at least); without it the op stays unknown_operation and the tag
  is not sent. It needs the lock (session_required / no_session / locked); request [TLV], answer completed success with
  no payload, sent first. The session's notifications end and zero-copy data already queued goes out before the answer
  (200 ms at most); after it the transport is flushed (not USB-Serial/JTAG's: HWCDC's flush with no TX timeout drops what
  waits), the session ends, every interface's new `Interface::probeRestart()` lets go of what the settings keep (a
  wire closes its connection, a slot's included, without resetting the target - a halted hart stays halted; a console
  closes its bound stream), every plan goes, the settings' too (each channel to its free state, core §8), and 20 ms
  after the flush (the host's USB stack takes the last packet) the handler restarts the chip: nothing more is served
  or sent, on any transport (the request read behind the restart is dropped). `oep::platformRestart()`: esp_restart on
  an ESP32, `rp2040.reboot()` (a watchdog reset 10 ms on, the USB controller with it) on an RP2. A serial port is back
  at its boot speed because the chip boots (oep.link §3). Firmware/OepProbe offers it on every board, restart_max_ms
  estimated from the boot path with a margin (not yet measured on the bench): classic ESP32 1500 ms (the UART bridge
  stays on the bus; about 0.5 s of ROM, app image check and setup), RP2040 / RP2350 2000 ms (setup within about 0.1 s,
  then USB re-enumeration and the OS's CDC port, up to about 1 s), ESP32-P4 3000 ms (about 0.5 s to setup, then the
  HS composite device - HID, vendor bulk, CDC, DFU - enumerated and its drivers bound, about 1 s or more on Windows;
  the HS device detaches first, as EspUsbDevice's own restarts do). Registry and test vectors synced from oep-spec
  3c96daf: the four ops.json restart cases run against the endpoint (86 cases). Host tests: the ops bit and
  restart_max_ms only with a handler, the refusals (no handler: unknown_operation; session_required, no_session,
  locked, a critical TLV unsupported - no restart), the answer out before the handler with a non-critical TLV listed
  as ignored, the handler kRestartSettleMs (20 ms) after it, the session's and the settings' plans released and the
  lock free when it runs, nothing answered after it (the request behind it, a request on the other transport) and no
  heartbeat; on the whole probe a slot's connection and its bound console closed, the hart left halted, every channel
  free.
- (JA) fn 0 の `restart`（op 0x14）と `restart_max_ms`（describe 0x4F）- oep-spec ecd1ab9 と 3c96daf（core §6.6、§7.5、§12、
  transports §1）: `Endpoint::setRestart(fn, max_ms)` で restart が fn 0 の ops に入り、describe に restart_max_ms が出る
  （`restart_after_answer_ms` の 100 より小さければそこまで上げる）。設定しなければ op は unknown_operation のままで、tag も
  出さない。ロックが要る（session_required / no_session / locked）。要求は [TLV]、応答は payload の無い completed success で、
  先に送る。セッションの通知を止め、すでに列に入っていた zero-copy のデータは応答の前に送り出す（多くても 200 ms）。応答の後:
  経路を flush し（USB-Serial/JTAG は除く: TX の待ち時間 0 の HWCDC の flush は待っているものを捨てる）、セッションを終え、
  新しい `Interface::probeRestart()` でどのインターフェースも設定が持つものを放し（線は接続を閉じる - スロットのものも - が、
  target は reset しない: 止まっていた hart は止まったまま。コンソールは bind のストリームを閉じる）、すべての plan を、設定の
  ものも解き（どの channel も空きの状態、core §8）、flush から 20 ms 後に（host の USB の側が最後のパケットを取る）handler が
  チップを再起動する。その後はどの経路でも何も処理せず何も送らない（restart の後ろに読んでいた要求は捨てる）。
  `oep::platformRestart()`: ESP32 は esp_restart、RP2 は `rp2040.reboot()`（10 ms 後の watchdog リセット。USB のコントローラも
  リセットされる）。シリアルの口は起動するので起動時の速さに戻る（oep.link §3）。Firmware/OepProbe はどのボードでも持ち、
  restart_max_ms は起動の道筋から余裕を持たせて見積もった（bench ではまだ測っていない）: classic ESP32 1500 ms（UART bridge は
  bus に残る。ROM、アプリのイメージの確かめ、setup で約 0.5 s）、RP2040 / RP2350 2000 ms（約 0.1 s で setup、その後 USB の
  列挙し直しと OS の CDC の口で約 1 s まで）、ESP32-P4 3000 ms（setup まで約 0.5 s、その後 HS の複合 device - HID、vendor bulk、
  CDC、DFU - の列挙とドライバの結び付けで Windows では約 1 s 以上。EspUsbDevice 自身の再起動と同じく先に HS の device を外す）。
  registry と test vector を oep-spec 3c96daf から写した: ops.json の restart の 4 件を endpoint に当てる（86 件）。host test:
  handler のあるときだけの ops の bit と restart_max_ms、断り（handler 無し: unknown_operation。session_required、no_session、
  locked、critical の TLV は unsupported - どれも再起動しない）、critical でない TLV を ignored に載せた応答が handler より先に
  出ること、handler は応答から kRestartSettleMs（20 ms）後、そのときセッションの plan と設定の plan が解かれロックが空いて
  いること、その後は何にも答えないこと（後ろの要求、もう一つの経路の要求）とハートビートが出ないこと。probe 全体では、スロットの
  接続と bind のコンソールが閉じ、hart は止まったまま、どの channel も空き。
- (EN) riscv-dm on a link that drops at every change of hart state (CH32L103): after each change (halt, a run's or a
  step's stop, resume, a hart found halted) the probe brings the link up and looks until it stays up (DMSTATUS a
  module's, DMCONTROL dmactive with hart 0 - a stale read shows - 3 good looks in a row, 20 ms at most). The drop can
  come a few transactions after the change, past the one relink, and the register reads after a run's stop or the
  writes after a halt met it: ch32rv's uploads to the L103 through the RP2350 failed about 1 in 15 ("run: timeout",
  once "run: fault"), 3 in 12 with other sessions between. Host test: ch32rv's upload sequence 100 times over the wire
  fake with the drop at random timing - 71 failed before, none now. Bench trace, off by default: build with
  `OEP_DEBUG_LOG=1` (and on RP2 `OEP_DEBUG_LOG_TX=<GP>`, Serial2 at 921600) for a line per request and the run's steps.
- (JA) hart の状態が変わるたびに link が落ちる線（CH32L103）での riscv-dm: 変わるたびに（halt、run や step の停止、resume、
  止まっていた hart を見つけたとき）probe は link を立て直し、落ちたままでないと分かるまで見る（DMSTATUS がモジュールのもの、
  DMCONTROL が dmactive で hart 0 - 古い値の読みはここで分かる - を 3 回続けて、多くても 20 ms）。落ちるのは変わってから
  何回か後のやりとりのことがあり、1 回の立て直しの後に来て、run の停止の後のレジスタの読みや halt の後の書き込みが
  それに当たっていた: ch32rv の RP2350 を通した L103 への書き込みが約 15 回に 1 回（"run: timeout"、1 回は "run: fault"）、
  間にほかのセッションがあると 12 回に 3 回失敗していた。host test: ch32rv の書き込みの手順を、落ちる時機が乱数の wire の
  fake で 100 回 - 前は 71 回失敗、今は 0。bench の trace（既定は無効）: `OEP_DEBUG_LOG=1`（RP2 は `OEP_DEBUG_LOG_TX=<GP>` も。
  Serial2、921600）でビルドすると、要求ごとと run の手順ごとに 1 行出る。
- (EN) **Breaking: the 2026-10-06 wire (oep-spec 59dd028, the v1 simplification).** A host, a fake or a broker of
  the wire before this does not talk to this probe, and saved settings must be set again. Implements oep-spec
  b69ec26..59dd028 (no `v0.x` tag yet):
  - one 10-byte request header with session_id (0 = no session; role 0x81 gone): a lock-free op with 0 skips the
    session check, one with an id goes through it; an op that needs the lock with 0 is session_required; open carries
    its id in the header (0 = malformed) and answers lease_ms boot_id. A request shorter than 10 bytes is dropped.
  - every TLV is tag(u8) len(u16) value (the long form gone); the ignored TLV keeps 19 bytes of room, more than 16
    ignored: the first 15 and 0x00, fewer that fit: the last 0x00.
  - sequences without element lengths: list, connections, scan, marks, console streams, capture segments, probe.config
    state (slot_state with reset_at_ns before the tid) and bind items (3 bytes a stream).
  - closed fixed forms: a request TLV or a probe.config item longer than its form is never an extension - critical:
    unsupported with the tag as received; else ignored (an item: not applied, its key neither replaced nor created).
    probe.config items have one form each: idle 6 bytes with drive_kind 2 = default (an input idle carries kind 2 value
    0), slot with boot_reset after attach and the lock last, bind streams of 3 bytes. gpio set's drive takes kind 2.
  - the `ops` describe tag (0x09, base + bitmap) first in every fn's describe, written by the endpoint from
    `Interface::offers` (which now defaults to none: a sketch's own interface overrides it); features no longer declare
    ops (riscv-dm none, i2c-target stretch, logic / analog query / force, capture-group force). fn 0's port_speed tag
    0x4E gone. `TargetRiscvDm::offerOptional(false)` and `ProbeConfig::setStorage(false)` leave the optional ops out.
  - no resume (D1): end, a lapse and force all release everything the session created (its plan, its shares of
    connections and console streams - a stream it was the last user of closes with mark closed 2, session_ended,
    before the connection's share - its subscriptions, its capture-group bind); any id while the lock is free is
    no_session; a resent end is answered from the resend table, which a successful open drops; the owner comes from the
    open that takes the lock. `Interface::sessionLapsed` is now `sessionOver` (called at every session end) with
    `sessionOverFirst`. Console streams stay the probe's per place and mechanism; slot connections stay; an attach on a
    live combination returns the existing connection.
  - oep.link (D3): the link test (source len(u16) data, at most max_frame - 26; sink count(u16) data) and port_speed
    move from fn 0 to `oep::Link`, which the firmware adds as its last fn (the fns before it keep their numbers);
    port_speed is in its ops when the endpoint has a handler (classic ESP32).
  - 59dd028: an attach joining a live connection keeps the settings it does not carry - only a carried idle_clock
    changes how the line rests (a slot's low rest no longer went high under a host's attach without the TLV); a scan
    never changes a live connection's settings.
  - the saved settings: NVS key "items5" / EEPROM magic "OEP5"; a blob of the form before ("items4" / "OEP4") reads as
    unreadable reason 1 until the host sets and saves again.
  - registry synced (the constants added by hand for f0c68bf now generated); `tools/sync_registry.sh` also copies the
    spec's test vectors to `tests/vectors/` (`OEP_SPEC_REF` picks the commit); `tests/host/test_vectors.cpp` runs every
    one of them (headers, cobs, checks, confirm, discovery, sessions, refusals, ops, probe_config_hash) against the
    endpoint byte for byte. `cobsEncode` / `cobsDecode` and `DmConsole::crc8` public; `Endpoint::setMaxOpMs`;
    `ResourceNumbers::reset` for host tests.
  - after 59dd028: oep-spec a193272 (a failed attach does not add the session to the connection's users) is what attach
    already does; ecd1ab9 / 3c96daf (fn 0 restart): the restart entry above.
- (JA) **破壊的: 2026-10-06 の wire（oep-spec 59dd028、v1 の単純化）。** これより前の wire の host、fake、ブローカーはこの
  probe と話せず、保存した設定は入れ直す。oep-spec b69ec26..59dd028 を実装する（`v0.x` の tag はまだ無い）:
  - session_id を持つ 10 byte の要求の見出し一つ（0 = セッションなし。role 0x81 は無くなった）: ロックなしの op は 0 なら
    セッションを確かめず、id があれば確かめる。ロックの要る op の 0 は session_required。open は id を見出しに持ち（0 は
    malformed）、lease_ms boot_id で答える。10 byte より短い要求は捨てる。
  - TLV はすべて tag(u8) len(u16) value（長い形は無くなった）。ignored の場所は 19 byte、16 を超えたら最初の 15 と 0x00、
    入らなければ最後を 0x00。
  - 要素の長さの無い並び: list、connections、scan、marks、console の streams、capture の segments、probe.config の state
    （slot_state の reset_at_ns は tid の前）、bind の項目（ストリーム 1 つ 3 byte）。
  - 閉じた固定の形: 形より長い要求の TLV と probe.config の項目は伸ばしたものではない - critical なら受け取ったままの tag で
    unsupported、そうでなければ ignored（項目は適用せず、キーを置き換えも作りもしない）。probe.config の項目は tag ごとに形が
    一つ: idle は 6 byte で drive_kind 2 = 既定（入力の idle は kind 2 value 0）、slot は boot_reset を attach の後に置き錠で
    終わる、bind のストリームは 3 byte。gpio set の drive も kind 2 を受ける。
  - describe の `ops` の tag（0x09、base + bitmap）を、すべての fn の describe の最初に endpoint が `Interface::offers` から書く
    （offers の既定は「何も無い」に変わった: スケッチ自身のインターフェースは上書きする）。features は op を宣言しない
    （riscv-dm は features 無し、i2c-target の stretch、logic / analog の query / force、capture-group の force）。fn 0 の
    port_speed の tag 0x4E は無くなった。`TargetRiscvDm::offerOptional(false)` と `ProbeConfig::setStorage(false)` で任意の op を外せる。
  - 再開なし（D1）: end、リースの期限切れ、force はどれも、セッションが作ったものをすべて解放する（plan、接続とコンソールの
    ストリームの分 - そのセッションが最後に使っていたストリームは、接続の分より先に、mark closed 2（session_ended）で閉じる -、
    購読、capture-group の bind）。ロックが空いている間はどの id も no_session。送り直した end には送り直しの表から答え、表は
    成功した open で捨てる。owner はロックを取る open から取る。`Interface::sessionLapsed` は `sessionOver` になり（セッションの
    終わりのたびに呼ぶ）、`sessionOverFirst` を足した。コンソールのストリームは場所と mechanism ごとの probe のもの、スロットの
    接続は残る、生きている組への attach は今の接続を返す。
  - oep.link（D3）: 線の試験（source は len(u16) data で多くても max_frame - 26、sink は count(u16) data）と port_speed は fn 0 から
    `oep::Link` へ移った。firmware はこれを最後の fn として足す（前の fn の番号は変わらない）。port_speed は endpoint に handler が
    あるとき（classic ESP32）ops に入る。
  - 59dd028: 生きている接続に加わる attach は、運ばない設定を変えない - 休み方を変えるのは運んだ idle_clock だけ（スロットの
    low の休み方が、TLV の無い host の attach で high に戻っていた）。scan は生きている接続の設定を変えない。
  - 保存した設定: NVS のキー "items5" / EEPROM の magic "OEP5"。前の形の blob（"items4" / "OEP4"）は、host が入れ直して保存する
    まで unreadable reason 1 と読む。
  - registry を同期した（f0c68bf のために手で足した定数も生成物になった）。`tools/sync_registry.sh` は仕様の test vector も
    `tests/vectors/` に写す（`OEP_SPEC_REF` で commit を選ぶ）。`tests/host/test_vectors.cpp` がそのすべて（headers、cobs、checks、
    confirm、discovery、sessions、refusals、ops、probe_config_hash）を endpoint に byte ごとに当てる。`cobsEncode` / `cobsDecode` と
    `DmConsole::crc8` を公開し、`Endpoint::setMaxOpMs` と host の試験のための `ResourceNumbers::reset` を足した。
  - 59dd028 の後: oep-spec a193272（失敗した attach はセッションを接続の users に加えない）は attach がすでにそうしている。
    ecd1ab9 / 3c96daf（fn 0 の restart）: 上の restart の項目。
- (EN) Console (DMDATA / dmseq / SDI reading): a havereset in the console's DMSTATUS look counts only once ackHaveReset's own read confirms it. One bad read with bit 18 set unsynced dmseq, which dropped the input chunk on its way - up to 2 bytes of a command, which the target then never answered (a READ command left unanswered for 3 s on the X035 behind the P4 among many answered ones, gpio_matrix: a likely way) - and took the next repeat of a frame for a new one (its bytes twice, a restart marked). A DMSTATUS of all ones brings the bus back in step (the wire's configuration sequence, no debug-module register written): a link that dropped (a CH32L103 at its own restarts) read all ones under back-to-back polls with no idle time for the PHY's revive, until wire_lost_ms closed the stream. Host test: the answer carrying "RE" of "READ 13" lost, then one bad havereset read: the target gets the whole line, no restart; all ones, then the console goes on.
- (JA) console（DMDATA / dmseq / SDI の読み）: console が DMSTATUS を見たときの havereset は、ackHaveReset 自身の読みがそれを確かめたときだけ数える。bit 18 の立った悪い読み 1 回で dmseq を未同期に戻し、送っている途中の入力のかたまり（コマンドの 2 byte まで）を捨てていて、target はそのコマンドに答えなかった（P4 の後ろの X035 の gpio_matrix で、多くの答えの中で READ が 1 回 3 s 答えなかった: ありうる道筋）。また次に出し直されたフレームを新しいものと取っていた（そのバイトが 2 回、restart の mark）。DMSTATUS が全 1 なら線の設定の手順で bus を合わせ直す（debug module のレジスタは書かない）: 落ちた link（自分で再起動した CH32L103）は、間を空けない poll では PHY の revive が働かず、wire_lost_ms でストリームが閉じるまで全 1 を読んでいた。host test: 「READ 13」の「RE」を運ぶ答えが失われ、havereset の悪い読みが 1 回: target は行をすべて受け、restart は無い。全 1 の後も console は続く。
- (EN) riscv-dm read_block, write_block, run and step put abstractauto back as they found it (oep-if-debug §4's table: the value read in the op before touching it): read at the op's start and cleared before DATA0 is first touched, written back last, after DATA0 (a read that does not answer counts as 0). They forced it to 0 at the end, and the probe's own view of it (auto_on_) followed only its own writes: after a host's raw ABSTRACTAUTO = 1 the op's first DATA0 access - keeping the target's mailbox, moving a register through DATA0 - ran the last abstract command again. Host test: a host's raw ABSTRACTAUTO = 1, then write_block: the words land, nothing past them is written, the mailbox and GPRs as found, abstractauto 1 again.
- (JA) riscv-dm の read_block、write_block、run、step は abstractauto を見つけたときの値に戻す（oep-if-debug §4 の表: op の中で触る前に読んだ値）: op の始めに読み、DATA0 に初めて触る前に 0 にし、最後に DATA0 の後で書き戻す（答えの無い読みは 0 とする）。これまでは終わりに 0 にしていて、probe が持つ値（auto_on_）は自分の書き込みしか追っていなかった: host が raw で ABSTRACTAUTO = 1 を書いた後、op の最初の DATA0 への接触（target の郵便受けを取っておく、レジスタを DATA0 経由で動かす）が、最後の抽象コマンドをもう一度走らせていた。host test: host の raw の ABSTRACTAUTO = 1 の後の write_block: word が届き、その先は書かれず、郵便受けと GPR は元のまま、abstractauto は 1 に戻る。
- (EN) riscv-dm on a target that drops its DMI link at every change of hart state (a CH32L103; oep-if-debug §4's table: a read right after the change may give the previous value): the block ops', step's and run's check that the hart is halted reads DMSTATUS once more from a freshly brought-up link when the first read is no module's (all ones) or says the hart runs, and the waits for a change of state (resume, step, run, the reset's halt) bring the link up again every 1 ms besides on a read that is no module's. After a change the probe did not make - a host's raw halt, a hart stopped by itself - checkHalted looked once: a block op answered line (all ones) or state with 0 words and the module answering (the stale "running"; the shape of ch32rv crt0_probe's "write_block stopped after 0: state" on the L103 through the RP2350), and the run's wait, which since fa87704 relinked on all ones only, went on to its timeout with the hart at its ebreak when the line kept giving the last "running" (the 1-in-N "run: timeout" of ch32rv uploads with 0.0.28+fa87704), as a stale "halted" could hide a resume. Host test: the wire's fake with the abstract commands, program buffer and autoexec modelled, its link dropping at every change of state with writes lost and reads all ones or stale - write_block / read_block, a host's raw halt then write_block, a run that stops after a few reads, resume.
- (JA) hart の状態が変わるたびに DMI の link を落とす target（CH32L103。oep-if-debug §4 の表: 変わった直後の読みは前の値になりうる）での riscv-dm: block の op、step、run が hart が止まっているかを見るとき、最初の読みがモジュールのものでない（全 1）か hart が走っていると言うなら、link を立て直してから DMSTATUS をもう一度読む。状態の変化を待つ間（resume、step、run、reset の halt）は、モジュールのものでない読みのときに加えて 1 ms ごとに link を立て直す。probe がしたのでない変化（host の raw の halt、自分で止まった hart）の後、checkHalted は一度しか見ず、block の op は line（全 1）か、モジュールが答えているのに 0 word の state で答えた（前の値の「走っている」。RP2350 経由の L103 で ch32rv の crt0_probe が得た「write_block stopped after 0: state」の形）。run の待ちは fa87704 から全 1 のときだけ link を立て直していたので、線が前の「走っている」を返し続けると hart が ebreak で止まっているのに時間切れまで待ち（0.0.28+fa87704 の ch32rv の書き込みの N 回に 1 回の「run: timeout」）、前の「止まっている」は resume を隠しえた。host test: wire の fake に抽象コマンド、program buffer、autoexec を持たせ、状態が変わるたびに link を落とし（書き込みは失われ、読みは全 1 か前の値）- write_block / read_block、host の raw の halt の後の write_block、数回の読みの後に止まる run、resume。
- (EN) riscv-dm reset's wait for a silent debug module is limits.reset_settle_ms (700 ms, oep-if-debug §4.3, oep-spec f0c68bf) from the registry, not derived from host_wait_add_ms: the host now counts it as argument time (core §4.4). Same value, no change in behaviour.
- (JA) riscv-dm の reset が答えない debug module を待つ時間は、host_wait_add_ms から導いた値ではなく registry の limits.reset_settle_ms（700 ms、oep-if-debug §4.3、oep-spec f0c68bf）にした: host はそれを引数の時間として数える（core §4.4）。値は同じで、動きは変わらない。
- (EN) Console write fills a send queue of each stream (oep-if-console §1, §2, oep-spec f0c68bf): accepted = min(count, free space), completed failed only when the queue is full (SDI, one way, takes nothing); the probe feeds the target from the queue's head at the mechanism's pace - up to 2 bytes on each answer to a dmseq frame, 3 on each answer to a DMDATA slot; the queue goes with the stream when it closes and stays across target restarts and resets; a bind's input fills the same queue. Console describe declares it: TLV 0x41 send_queue = 256 (at least limits.console_send_queue_min_bytes = 64). The write took only the mechanism's send slot (2 / 3 bytes, 0 while it held a chunk, 56aa7dc): a line cost a request every 2-3 bytes, and the bench saw a classic ESP32's console commands 3x slower, lost replies on a CH32V003 and P4 captures armed before a command miss its burst; the experiment 0.0.28+fa87704-queue-exp (the whole queue, as in 0.0.28) fixed all three. Registry: the new constants added by hand (limits.reset_settle_ms, limits.console_send_queue_min_bytes, console send_queue) until the new-codec port regenerates the file. Host test: describe, a write that fills the queue (partial, then failed), SDI, the queue dropped at close.
- (JA) console の write はストリームごとの送りの列に入る（oep-if-console §1、§2、oep-spec f0c68bf）: accepted = count と列の空きの小さい方で、completed failed は列が満ちているときだけ（一方向の SDI は何も受けない）。probe は列の先頭から mechanism の運び方で target に渡す - dmseq はフレームへの答えごとに 2 byte まで、DMDATA は枠への答えごとに 3 byte まで。列はストリームが閉じたら捨て、target の再起動と reset では保つ。bind の入力も同じ列に入る。console の describe が宣言する: TLV 0x41 send_queue = 256（limits.console_send_queue_min_bytes = 64 以上）。write は mechanism の送り枠だけを受けていた（2 / 3 byte、枠に一つあれば 0、56aa7dc）: 1 行に 2〜3 byte ごとの要求が要り、bench では classic ESP32 の console のコマンドが 3 倍遅く、CH32V003 の返事が失われ、コマンドの前に構えた P4 のキャプチャがバーストを取り逃がした。実験の 0.0.28+fa87704-queue-exp（0.0.28 と同じく列全体）で 3 つとも直った。registry: 新しい定数（limits.reset_settle_ms、limits.console_send_queue_min_bytes、console の send_queue）は、新しい codec への移行でファイルを作り直すまで手で足した。host test: describe、列を満たす write（partial、次に failed）、SDI、close で捨てる列。
- (EN) Console DMDATA by oep-if-console §3.2 (oep-spec 4621eab): a word with bit 7 set and L 0-3 or 12-63 is no target slot - it carries no bytes, gets 0 and no input on it (L 12-63 pushed 7 bytes, 0xff each from an all-ones word, and took input; L 0-3 was answered like an empty slot, with input); DATA1 is read only for a slot of 4 bytes or more. Open writes nothing to the mailbox and throws away none of the target's output: SDI and DMDATA zeroed DATA0 (also over a bit-7-clear word, the probe's own answer or the target's 0) and discarded four polls. The empty slot (L 4) is still answered on the read after the one that found it (the spec leaves the timing to the probe; a target that replaces its empty slot loses nothing). Host test: a DMDATA target - a slot waiting at open kept, a bit-7-clear word never written, input three bytes per answer, the words that are no slot, a 7-byte slot, the empty slot.
- (JA) console の DMDATA を oep-if-console §3.2 に合わせた（oep-spec 4621eab）: bit 7 が立ち L が 0〜3 か 12〜63 の word は target の枠ではない - バイトを運ばず、0 を書き、入力を載せない（L 12〜63 は 7 byte を受け取り（全 1 の word なら 0xff を 7 つ）入力を載せていた。L 0〜3 は空の枠として入力つきで答えていた）。DATA1 は 4 byte 以上の枠でだけ読む。open は郵便受けに何も書かず、target の出力を捨てない: SDI と DMDATA は DATA0 に 0 を書き（bit 7 が 0 の word、probe 自身の答えや target の 0 の上にも）、poll 4 回分を捨てていた。空の枠（L 4）は、見つけた次の読みで答えるまま（時期は probe に任されている。空の枠を置き換える target も何も失わない）。host test: DMDATA の target - open の時に待っていた枠を保つ、bit 7 が 0 の word に書かない、答えごとに 3 byte の入力、枠でない word、7 byte の枠、空の枠。
- (EN) ESP32-P4 logic: every start is a clean capture, with or without a configure before it - its receive goes to a PARLIO RX unit made for it with its soft delimiter and callbacks, as configure leaves one (immediate one-shot, one-shot through the ring, repeat, streaming). A unit that had run kept what its last transaction left: after a stop of an immediate one-shot the next start's done came after only the rest of the window, with 32-60 % of the segment never written (zeros where the line was held high) under clean-looking metadata, and after a completed run each start began with 2064 samples (258 bytes) of the previous capture's level (P4 bench, 0.0.28+1c940ca). The analog capture (a new ADC handle each start on the ESP32s; on the RP2 the ADC stopped, its FIFO drained and the DMA set up again) and the classic ESP32's sampler (a new sampling task, the buffer cleared) already started clean. Host test: a start on a unit that had a receive before is counted by the fake driver.
- (JA) ESP32-P4 の logic: start は、前に configure があっても無くても、いつもきれいな取り込みになる - その receive は、configure が残すのと同じく、その start のために作った PARLIO RX の unit（soft delimiter と callback つき）に載る（即時の one-shot、ring を通る one-shot、repeat、streaming）。一度走った unit は前の transaction の残りを持っていた: 即時の one-shot を stop した後の start は、window の残りの分だけで done になり、segment の 32〜60 % が書かれないまま（線を high に保っていたのに 0）で、metadata はきれいに見えた。完了した run の後でも、start のたびに前の取り込みの level が 2064 sample（258 byte）入って始まった（P4 の bench、0.0.28+1c940ca）。analog（ESP32 系は start ごとに新しい ADC の handle、RP2 は ADC を止め FIFO を空にして DMA を設定し直す）と classic ESP32 の sampler（新しい取り込みの task、buffer を消す）は、もとからきれいに始まっていた。host test: 前に receive のあった unit への start を fake の driver が数える。
- (EN) riscv-dm run, step and resume bring the link up again (relink) when DMSTATUS reads all zeros / all ones, as they did on a failed read (oep-if-debug §4.2, §4.4). A CH32L103 drops its DMI link at every change of hart state and the line then reads all ones: since the version check on those waits (92c13a3) the wait went on with the link down - run timed out with the hart stopped at its ebreak (ch32rv uploads through the RP2350: 3 of 6 run timeout, 1 fault, 1c940ca), and resume could miss its acknowledgement. 0.0.28 took all ones as halted (bit 9) and relinked by accident. Host test: a module whose link drops at every change of hart state.
- (JA) riscv-dm の run、step、resume は、DMSTATUS が全 0 / 全 1 で読めたときも、読みが失敗したときと同じく link を立て直す（relink）（oep-if-debug §4.2、§4.4）。CH32L103 は hart の状態が変わるたびに DMI の link を落とし、線は全 1 を読む: それらの待ちに version の確認を入れて（92c13a3）から、link が落ちたまま待ち続け、hart が ebreak で止まっているのに run が時間切れになり（RP2350 経由の ch32rv の書き込み: 6 回のうち run timeout 3、fault 1、1c940ca）、resume は確認応答を見逃しえた。0.0.28 は全 1 を halted（bit 9）と取り、たまたま relink していた。host test: hart の状態が変わるたびに link が落ちるモジュール。
- (EN) riscv-dm reset waits out a target that restarts itself on its way out of the reset (oep-if-debug §4.3, §2): after the resume (mode 0 and 1) it looks at the module once more after 1 ms and, while DMSTATUS does not answer, keeps relinking until it does, acknowledges the havereset it then shows, and only then confirms (mode 1) and answers. A CH32V003 reset right after power-on (BOOT_MODE) runs its bootloader, which answered nothing for a few hundred ms from just after the resume before its hand-over: 0.0.28+1c940ca answered the confirmed reset status line at 212 ms (flags 0x0d, the redo restarting it into the same hand-over), and a mode 0 reset ok with line for the next two requests. The wait ends kResetSettleMs (700 ms) after the op started, so the answer, with the confirmation's halt and resume, still comes within the host's wait for a request with no time argument (host_wait_add_ms, core §4.4); still silent then, flags bit0 is cleared and the answer is status line with no redo (the connection is kept: the reset's excuse). A resume whose acknowledgement the silence swallowed counts once the module is back with the hart running. The same for attach's reset TLV with method 0 (and a slot's reset attach): after the line is let go of, the attach tries until its search's deadline instead of three times (a few ms). Host test: the simulated V003's bootloader silent for 400 ms (from the resume on, or 200 us into its run), each reset mode, the next request answered at once; a module silent past the wait; attach with the reset TLV on a new and on a live connection.
- (JA) riscv-dm の reset は、reset から出る途中で自分からもう一度リセットする target を待ち切る（oep-if-debug §4.3、§2）: resume の後（mode 0 と 1）、1 ms 後にもう一度モジュールを見て、DMSTATUS が答えない間は答えるまで link を立て直し続け、そのとき見える havereset を確認応答してから、確認（mode 1）をして答える。電源投入直後（BOOT_MODE）にリセットした CH32V003 はブートローダーを走らせ、それは resume の直後から引き渡しまで数百 ms 何も答えなかった: 0.0.28+1c940ca は確認つきの reset に 212 ms で status line を答え（flags 0x0d。やり直しが同じ引き渡しへ再び始めさせていた）、mode 0 の reset は ok を答えて次の 2 つの要求が line だった。待ちは op の始まりから kResetSettleMs（700 ms）で終わるので、確認の halt と resume を足しても、時間の引数の無い要求に host が待つ時間（host_wait_add_ms、core §4.4）の中に答える。そこでまだ答えなければ flags bit0 を下ろし、やり直さずに status line で答える（接続は保つ: reset の猶予）。答えないことで確認が見えなかった resume は、モジュールが戻って hart が走っていれば済んだものとする。attach の reset TLV の method 0（とスロットの reset つきの attach）も同じ: 線を放した後、attach は 3 回（数 ms）ではなく探索の期限まで試す。host test: 模擬の V003 のブートローダーが 400 ms 答えない（resume から、または走り出して 200 us 後から）、各 reset の mode、次の要求がすぐ答えること。待ちを越えて答えないモジュール。新しい接続と生きている接続での reset TLV つきの attach。
- (EN) SWIO: a target that resets itself through a system reset keeps its connection (oep-if-debug §2: a target reset does not close it). A CH32V003 reset right after power-on (FLASH_STATR BOOT_MODE) starts in its bootloader, which hands over to the application with a system reset; that drops the SWIO configuration and the debug module (dmactive 0, havereset), and nothing answered until the configuration pair was written again - which only a fresh attach did. riscv-dm reset(confirm) answered status line with flags 0x0d, 2 attempts, and every request after it line until the connection was lost; a raw ndmreset in the same state did not lose it. A read that gets nothing back (it fails, or DMSTATUS reads no module) is now retried with the link brought back in step - the configuration pair twice, and dmactive when DMCONTROL reads it clear - within the request's wire_retry_ms, and Ch32Dm's relink does the same while the wire does not answer (SwioPhy::reinit had nothing to do; a link in step still pays nothing). The RVSWD PHY already re-synced on relink and before a read after a rest. Host test: the swio stack on a simulated V003 whose reset goes through its bootloader (reset modes 0 / 1 / 2, the hand-over before and after the op's answer; the old PHY answers line, 0x0d, 2 attempts there, as on the bench); SwioPhy has a host backend (OEP_HOST_FAKE_SWIO).
- (JA) SWIO: system reset で自分からリセットする target でも接続を保つ（oep-if-debug §2: target のリセットは接続を閉じない）。電源投入直後（FLASH_STATR の BOOT_MODE）にリセットした CH32V003 はブートローダーから始まり、それが system reset でアプリケーションに渡す。それで SWIO の設定とデバッグモジュール（dmactive 0、havereset）が落ち、設定の対を書き直すまで何も答えなかった - それをしていたのは新しい attach だけだった。riscv-dm の reset(confirm) は flags 0x0d、attempts 2 の status line を答え、その後の要求はどれも接続が失われるまで line だった。同じ状態での生の ndmreset では失われなかった。何も返らない読み出し（失敗した、または DMSTATUS がモジュール無しと読めた）は、線を合わせ直してから - 設定の対を 2 回と、DMCONTROL が dmactive を 0 と読めたら dmactive - 要求の wire_retry_ms の中でやり直すようにし、Ch32Dm の relink も wire が答えない間は同じことをする（SwioPhy::reinit は何もしていなかった。合っている線は今までどおり何も払わない）。RVSWD の PHY は relink と休んだ後の読み出しの前にすでに合わせ直していた。host test: リセットがブートローダーを通る V003 の模型の上の swio の一式（reset の mode 0 / 1 / 2、受け渡しが op の答えの前と後。古い PHY はそこでベンチと同じく line、0x0d、attempts 2 を答える）。SwioPhy に host の backend（OEP_HOST_FAKE_SWIO）を付けた。
- (EN) Console write latency back to 0.0.28's (oep-if-console §2): the send slot is free while nothing waits in it, also while the dmseq payload before it is on its way unacknowledged, and the write op looks at a waiting target frame before it judges the slot. The slot freed only at the target's acknowledgement, which the probe answered at once with nothing - so every 2 bytes waited two of the target's polls: a 5-byte PING took 50-113 ms on the X035 (13-26 ms in 0.0.28), and captures armed before a console command with a window under about 50 ms missed the burst. Now one chunk goes on each answer, as when the write queued the whole line. Still 2 bytes (dmseq) / 3 bytes (DMDATA) per write, accepted 0 while the slot holds one. Host test: a dmseq target polling every 1 / 5 / 10 ms gets a PING in at most three polls (it took five).
- (JA) console の write の遅れを 0.0.28 のものに戻した（oep-if-console §2）: 送り枠は、その中に何も待っていなければ、前の dmseq の payload がまだ ack されずに送られている途中でも空いている。write の op は、枠を判断する前に待っている target のフレームを見る。枠は target の ack で初めて空いていて、probe はその ack に何も載せずにすぐ答えていた - そのため 2 byte ごとに target の poll を 2 回待った: X035 で 5 byte の PING に 50-113 ms かかり（0.0.28 は 13-26 ms）、console のコマンドの前に構えた窓が約 50 ms 未満のキャプチャはバーストを取り逃がした。今は答えのたびにひとかたまりが出ていき、write が行全体を待ち行列に入れたときと同じになる。write 一回は今までどおり 2 byte（dmseq）/ 3 byte（DMDATA）で、枠に一つ入っている間は accepted 0。host test: 1 / 5 / 10 ms ごとに poll する dmseq の target に PING が poll 3 回以内に届く（5 回かかっていた）。
- (EN) riscv-dm run sets ebreaks and ebreaku in dcsr for the run again, with ebreakm and prv = M, and puts ebreaks / ebreaku back as they were once the hart has stopped (oep-if-debug §4.4: the run leaves ebreakm and prv changed only). The earlier change set ebreakm alone, and a loader the hart still ran in U mode - a CH32L103 stopped in its sketch - trapped at its ebreak instead of halting: ch32rv uploads through the RP2350 failed now and then with "run: timeout", and crt0_probe's write_block after it with state (0.0.28, which set all three, passed). Host test (the dcsr writes of a run).
- (JA) riscv-dm の run は、ebreakm と prv = M に加えて、run の間 dcsr の ebreaks と ebreaku をまた立て、hart が止まったら ebreaks / ebreaku を元に戻す（oep-if-debug §4.4: run が変えたまま残すのは ebreakm と prv だけ）。前の変更は ebreakm だけを立てていて、hart がまだ U モードで動かしたローダー - スケッチの中で止めた CH32L103 - は ebreak で止まらずに trap した: RP2350 越しの ch32rv の書き込みがときどき "run: timeout" で失敗し、その後の crt0_probe の write_block が state になった（3 つとも立てる 0.0.28 は通っていた）。host test（run の dcsr の書き込み）。
- (EN) ESP32-P4 logic: stop of an immediate one-shot while capturing (state 3) goes to state 1 with the segment cut short (flags bit1) holding the DMA nodes finished before the stop, readable, with its segment event (capture §3.2; it went to state 1 with done 0, write_pos 0 - the earlier change covered the triggered one-shot only); stopped before a node finished, state 1 with nothing; one that filled meanwhile completes (state 4). The unfinished transaction is erased (the unit disabled and enabled with its queue reset), so the next start mounts its own: the driver kept the old one and went on filling it. The node count comes from the driver's per-node callback (on_partial_receive, now registered for the immediate one-shot). Analog and the classic ESP32 sampler were checked: analog cuts both immediate and triggered short, the sampler's window cannot be cut and completes (state 4). Host test.
- (JA) ESP32-P4 の logic: 取り込み中（state 3）の即時の one-shot の stop は state 1 になり、stop の前に終わった DMA の node を持つ、短く切れた（flags bit1）読める segment を、その segment の event とともに残す（capture §3.2。done 0、write_pos 0 の state 1 になっていた - 前の変更はトリガ付きの one-shot だけだった）。node が一つも終わる前の stop は何も無い state 1。その間に満ちたものは完了する（state 4）。終わっていない transaction は消す（unit を disable し、queue を reset して enable する）ので、次の start は自分のものを載せる: driver は古いものを持ち続け、それを満たし続けていた。node の数は driver の node ごとの callback（on_partial_receive。即時の one-shot でも登録するようにした）から取る。analog と classic ESP32 の sampler は確かめた: analog は即時もトリガ付きも短く切り、sampler の窓は切れないので完了する（state 4）。host test。
- (EN) ESP32-P4 logic: one large immediate one-shot configure no longer breaks every later triggered (and repeat / streaming) configure until a reboot. The immediate segment took an internal block of its own and, finding none (523264 samples on 1 line: 65408 bytes), freed the 128 KiB DMA ring for it; once the rest of the firmware had taken a piece of the freed space, no ring could be taken again and every triggered configure answered completed failed with an empty payload, state 6 error 1. The ring is now taken with the plan and never freed, and an immediate one-shot's segment is the ring itself (its own internal block, then PSRAM, only without one). A configure that cannot get its memory - the ring, the trigger's segment, the repeat / streaming store, fewer than two streaming segments - is refused unavailable cause 3 (core §4.3 order 7) instead of failed with an empty payload; the ring is checked before the earlier configuration is touched. Analog: the RP2's ring is taken before the buffer is resized (its failure after a resize left the old samples over a smaller buffer) and both refuse cause 3; the classic ESP32 sampler's buffer (taken once, kept) refuses cause 3 too. Host tests.
- (JA) ESP32-P4 の logic: 大きな即時の one-shot の configure 一つで、その後のトリガ付き（と repeat / streaming）の configure が再起動まで全部壊れることはもうない。即時の segment は内部 RAM に自分のブロックを取り、それが無いと（1 本で 523264 sample: 65408 byte）128 KiB の DMA の ring を解放してそこに入っていた。解放した場所の一部をファームウェアのほかの部分が取ると ring は二度と取れず、トリガ付きの configure はどれも空の payload の completed failed、state 6 error 1 を答えていた。ring は plan で取って解放しないようにし、即時の one-shot の segment は ring そのものにした（ring が無いときだけ自分の内部 RAM のブロック、次に PSRAM）。必要なメモリ - ring、トリガの segment、repeat / streaming の store、streaming の 2 未満の segment - が取れない configure は、空の payload の failed ではなく unavailable cause 3 で断る（core §4.3 の順 7）。ring は前の構成に触れる前に確かめる。analog: RP2 の ring は buffer の大きさを変える前に取り（大きさを変えた後の失敗で、古い samples が小さい buffer の上に残っていた）、どちらも cause 3 で断る。classic ESP32 の sampler の buffer（一度取って保つ）も cause 3 で断る。host test。
- (EN) Console open with another mechanism (or at another place) makes a new stream and erases the old one (oep-if-console §2): the new stream started with the old one's bytes and marks (a new stream number showed up to 16 marks of earlier streams). Its positions and mark serials go on (oep-if-common §1.1); the same mechanism at the same place still reopens the stream with its marks. This probe keeps one stream per console, so an open at another place also replaces the closed one. The fixture UART's stream disappears when its plan is released (oep-if-fixture §2; the next plan's stream showed the old bytes and marks). PositionStream::erase. Host test.
- (JA) 別の mechanism での（または別の場所での）console の open は新しい stream を作り、古いものを消す（oep-if-console §2）: 新しい stream が古い stream の byte と mark を持って始まっていた（新しい stream 番号に、前の stream の mark が最大 16 個見えていた）。位置と mark の serial は続く（oep-if-common §1.1）。同じ場所での同じ mechanism はこれまでどおり mark ごと stream を開き直す。この probe は console ごとに stream を一つ持つので、別の場所での open は閉じた stream も置き換える。fixture の UART の stream は plan の解放で消える（oep-if-fixture §2。次の plan の stream に古い byte と mark が見えていた）。PositionStream::erase。host test。
- (EN) detach with its force TLV (oep-if-debug §2's table): the console streams on the connection it closes get mark detach, then closed 4 (they got closed 4 alone); a plain detach that closes the connection still marks closed 4 alone. DebugPort::detached, releaseConnection's detached argument. Host test.
- (JA) force の TLV 付きの detach（oep-if-debug §2 の表）: それが閉じる接続の console の stream には detach、続いて closed 4 の mark を付ける（closed 4 だけだった）。接続を閉じる普通の detach はこれまでどおり closed 4 だけ。DebugPort::detached、releaseConnection の detached 引数。host test。
- (EN) Positioned streams mark the ring's overflow (oep-if-common §1.1, §1.3): the first byte that pushes an older one out attaches mark lost with detail 1, once per overflow episode - the ring then stays full and every later byte pushes one out, so the next mark comes only after a clear has emptied it (nothing was marked; a read still reports the gap). Console and fixture UART alike. Host test.
- (JA) 位置付きストリームは ring のあふれに mark を付ける（oep-if-common §1.1、§1.3）: 古い byte を押し出す最初の byte で detail 1 の lost の mark を、あふれの一続きにつき一度付ける - その後 ring は満ちたままで、後の byte はどれも一つ押し出すので、次の mark は clear が ring を空にした後にだけ付く（何も付けていなかった。read はこれまでどおり gap を報せる）。console でも fixture の UART でも同じ。host test。
- (EN) dmseq console: a frame accepted with TO set (the target gave up waiting and discarded what it wrote until an answer) attaches mark lost with detail 4, the target's TO, right after its payload (oep-if-common §1.3; nothing was marked). A duplicate of it adds nothing. DmConsole::setSink takes a mark sink. Host test.
- (JA) dmseq の console: TO の立ったフレームを受け入れたら（target が待つのを諦め、答えが来るまでに書いたものを捨てた）、その payload の直後に detail 4（target の TO）の lost の mark を付ける（oep-if-common §1.3。何も付けていなかった）。その重複には何も足さない。DmConsole::setSink が mark の受け口を取る。host test。
- (EN) The console's mark attach carries detail 0 (oep-if-common §1.3: attach has no detail; it carried the mechanism number). Host test.
- (JA) console の attach の mark の detail は 0（oep-if-common §1.3: attach に detail は無い。mechanism の番号を入れていた）。host test。
- (EN) The console keeps reading while the hart runs, judged from DMSTATUS read at least every 20 ms (oep-if-console §3, registry console_dmstatus_poll_ms): it stopped whenever the probe's own view said halted, so a hart the probe halted and the host then resumed through raw DMI went unread until the next high-level op. A DMSTATUS that says running also clears Ch32Dm's halted view; a halt by the probe still stops the reading at once. Host test.
- (JA) console は hart が走っている間は読み続け、それを少なくとも 20 ms ごとに読む DMSTATUS で判断する（oep-if-console §3、registry の console_dmstatus_poll_ms）: probe 自身の見方が halted のときはいつも止まっていたので、probe が止めた後に host が raw DMI で走らせた hart は、次の高水準の op まで読まれなかった。running を示す DMSTATUS は Ch32Dm の halted の見方も下ろす。probe による halt はこれまでどおりすぐに読みを止める。host test。
- (EN) Console write takes only what the mechanism's send slot carries in one go (oep-if-console §2, oep-if-common §1.4): 2 bytes for dmseq, 3 for DMDATA, and accepted 0 (completed failed) while the slot is not free - bytes still queued, or a dmseq payload not yet acknowledged (it queued up to 255 bytes). A host writes the rest after the progress it reads. A bind's serial input still queues what the driver's queue takes (DmConsole::room) and goes out a slot at a time. Host test.
- (JA) console の write は、mechanism の送り枠が一度に運ぶ分だけを受け取る（oep-if-console §2、oep-if-common §1.4）: dmseq は 2 byte、DMDATA は 3 byte。枠が空いていない間 - まだ queue にある byte、まだ ack されていない dmseq の payload - は accepted 0（completed failed）（255 byte まで queue に入れていた）。host は読みで進みを見て残りを書く。bind のシリアルの入力は、これまでどおり driver の queue が取るだけ入れ（DmConsole::room）、枠ずつ送る。host test。
- (EN) read from 3 (the last mark of kind arg) with arg over 0xFF is refused malformed (oep-if-common §1.2: a mark kind is u8; it looked up kind arg & 0xFF), in the console and in the fixture UART alike, before from 4+ is refused unsupported (PositionStream::checkRead). Host test.
- (JA) from 3（種類 arg の最後の mark から）で arg が 0xFF を超える read は malformed で断る（oep-if-common §1.2: mark の種類は u8。arg & 0xFF の種類を探していた）。console でも fixture の UART でも同じで、from 4 以上を unsupported で断るより先（PositionStream::checkRead）。host test。
- (EN) Console open on a live connection the console does not ride on - an arm-adi (swd) connection - is refused unavailable cause 6 (oep-if-console §1, §3; it answered no_connection); a number no live connection has stays no_connection. On a probe with one console per wire, a connection of the other wire gets the same answer. Host test.
- (JA) console が乗らない生きている接続 - arm-adi（swd）の接続 - への console の open は unavailable cause 6 で断る（oep-if-console §1、§3。no_connection を答えていた）。どの生きている接続も持たない番号は no_connection のまま。wire ごとに console を持つ probe では、もう一方の wire の接続にも同じ答えになる。host test。
- (EN) RVSWD's wake / configuration sequence always at T >= 500 ns and >= 1 / (2 x max_speed) (oep-if-debug §3.1): a re-sync of a link tuned faster - Ch32Dm's relink after a change of hart state, the revive after a rest of 300 us and its wakes - sent the configuration pair (and the wake) at the link's own faster period; the sequence now goes out at the slowest period and the link returns to its period after it (probeOnce likewise for its dmactive). Changes timing on hardware (each re-sync is slower). Host test.
- (JA) RVSWD の wake / 設定の手順はいつも T >= 500 ns かつ >= 1 / (2 x max_speed)（oep-if-debug §3.1）: 速く合わせた link の同期し直し - hart の状態が変わった後の Ch32Dm の relink、300 us 休んだ後の revive とその wake - は、設定の対（と wake）を link 自身の速い周期で送っていた。手順をいちばん遅い周期で送り、その後 link を自分の周期に戻す（probeOnce の dmactive も同じ）。ハードウェアでの時間が変わる（同期し直しのたびに遅くなる）。host test。
- (EN) rvswd / swio attach without pins, no live connection, on a wire whose pins the host chooses (oep-if-debug §1): exactly one candidate (allowed, not held by anything else, no channel disabled or with an idle item) is used as if named (it was refused unavailable); when the declaration allows only one combination and it is not a candidate, the refusal names why (cause 5 with holder_kind 6 / 7, or cause 1 with the holder). With one channel set for both roles only a one-wire link can have a single candidate (swd's pairs come mirrored, so its rule never applies). Host test.
- (JA) host が pin を選ぶ wire での、pins の無い、生きている接続の無い rvswd / swio の attach（oep-if-debug §1）: 候補（許され、ほかの何にも保持されず、disable や idle の項目のある channel を含まない組）がちょうど一つなら、名指されたものとして使う（unavailable で断っていた）。宣言が一つの組しか許さず、それが候補でないときは、その理由を言って断る（cause 5 と holder_kind 6 / 7、または cause 1 と保持者）。両方の role に一つの channel の集合を使うので、候補が一つになり得るのは 1 線の link だけ（swd の組は鏡の対になるので、この規則は当てはまらない）。host test。
- (EN) fixture uart on the RP2040 / RP2350 (fixture §2): a full receive queue is marked lost detail 1 (overflow) and a received break lost detail 2 (framing), read from arduino-pico's SerialUART (overflow(), getBreakReceived()) after the bytes kept before them (nothing was marked on the RP2). arduino-pico 6.1.1 drops a byte with a framing or parity error without reporting it (SerialUART::_handleIRQ), so such a byte without a break still leaves no mark.
- (JA) RP2040 / RP2350 の fixture uart（fixture §2）: 受信の待ち行列が溢れたら lost detail 1（overflow）、break を受けたら lost detail 2（framing）の mark を付ける。arduino-pico の SerialUART（overflow()、getBreakReceived()）から、それより前に保った bytes の後で読む（RP2 では何も mark していなかった）。arduino-pico 6.1.1 は framing や parity の誤りのあるバイトを知らせずに捨てる（SerialUART::_handleIRQ）ので、break の無いそういうバイトにはこれからも mark が付かない。
- (EN) Classic ESP32 GPIO34 / 35 / 36 / 39 (inputs only, no internal pulls): an idle item with mode 1 / 2 there is refused unsupported with the item's tag (probe.config §1; it was taken and enabled no pull), and they are left out of the role_channels of the roles that drive a line - UART TX, I2C SDA / SCL, SPI MISO - and of attach's reset line, a plan_apply naming them there refused unsupported (they were offered). PinTable::setNoPull / canPull / outputMask. gpio still declares modes 1 / 2 / 7 for every channel (a per-channel declaration needs a decision). Host tests.
- (JA) classic ESP32 の GPIO34 / 35 / 36 / 39（入力だけ、内蔵のプルが無い）: そこへの mode 1 / 2 の idle の項目は項目の tag で unsupported で断る（probe.config §1。受けて、プルを入れていなかった）。線を駆動する role（UART の TX、I2C の SDA / SCL、SPI の MISO）と attach の reset の線の role_channels から外し、そこにそれらを名指す plan_apply は unsupported で断る（提供していた）。PinTable::setNoPull / canPull / outputMask。gpio はこれまでどおりすべての channel に mode 1 / 2 / 7 を宣言する（channel ごとの宣言は決定が要る）。host test。
- (EN) swd attach joining a live connection faster than its max_speed slows that connection to max_speed or below and returns it (oep-if-debug §1; it refused the max_speed TLV unsupported, or ignored it when not critical), with search_retries in the answer. Host test.
- (JA) swd の attach が、その max_speed より速い生きている接続に加わるときは、その接続を max_speed 以下に下げて返す（oep-if-debug §1。max_speed の TLV を unsupported で断るか、critical でなければ無視していた）。答えに search_retries を付ける。host test。
- (EN) rvswd / swio attach joining a live connection acknowledges a pending havereset first and says so with flags bit0 (oep-if-debug §3, §4.6; only a new connection or a revived link did) - the console marks restart 1 for it; a slot's own attach joining a connection acknowledges it too. Host test.
- (JA) rvswd / swio の attach が生きている接続に加わるときも、保留中の havereset をまず acknowledge し、flags bit0 でそう言う（oep-if-debug §3、§4.6。新しい接続と立て直した link だけがしていた）- console はそれに restart 1 の mark を付ける。slot 自身の attach が接続に加わるときも acknowledge する。host test。
- (EN) riscv-dm run sets only ebreakm and prv = M in dcsr (oep-if-debug §4.4): (dcsr | 0x8003); it also set ebreaks and ebreaku (0xb003), which the hart kept after the run. Changes what the target is left with. Host test.
- (JA) riscv-dm の run は dcsr の ebreakm と prv = M だけを立てる（oep-if-debug §4.4）: (dcsr | 0x8003)。ebreaks と ebreaku も立てていて（0xb003）、run の後も hart に残っていた。target に残すものが変わる。host test。
- (EN) riscv-dm dmi within max_op_ms at run time (oep-if-debug §4.1): the probe tracks the request's elapsed time, and a request that reaches max_op_ms ends at the step running then (or the next to start) with status timeout and done = that step's index - a poll that read adds its last value, a wait is cut where the limit falls (a poll of max_reads, or waits plus the time of the reads between them, ran past it). Host test.
- (JA) riscv-dm の dmi を実行中も max_op_ms に収める（oep-if-debug §4.1）: probe は request の経過時間を数え、max_op_ms に達した request は、そのとき走っている step（または次に始める step）で status timeout、done = その step の index で終える - 読んだ poll は最後の値を足し、wait は上限のところで切る（max_reads の poll や、wait とその間の読みの時間で上限を越えて走っていた）。host test。
- (EN) riscv-dm's high-level ops on hart 0 (oep-if-debug §4): before every op other than dmi (halt with its already-halted path, resume, reset, step, read_block, write_block, run) and before an attach joins a live connection, DMCONTROL is read and, when the host's dmi left a hartsel (or hasel) other than 0, written back with hartsel 0 - every op then returns with it 0 (block reads / writes, run and step ran their abstract commands on whatever hart the host had selected). One more DMI read per op. Host test.
- (JA) riscv-dm の高水準の op は hart 0 に対して（oep-if-debug §4）: dmi 以外のどの op（すでに止まっているときの道も含む halt、resume、reset、step、read_block、write_block、run）の前と、attach が生きている接続に加わる前に、DMCONTROL を読み、host の dmi が 0 以外の hartsel（または hasel）を残していれば hartsel 0 で書き戻す - どの op もそれを 0 にして戻る（block の読み書き、run、step は host が選んだままの hart で abstract command を走らせていた）。op ごとに DMI の読みが 1 回増える。host test。
- (EN) riscv-dm reset (oep-if-debug §4.3): the reset procedure is redone at most once (registry reset_retries = 1; it ran up to 3 times, attempts 3), the wait for the hart to halt after ndmreset is released is 100 ms of time per procedure (it was 400 polls), and a failure caused by a cmderr of the reset's own abstract commands answers status fault (it was timeout). Changes timing on hardware. Host tests.
- (JA) riscv-dm の reset（oep-if-debug §4.3）: reset の手順のやり直しは 1 回まで（registry の reset_retries = 1。3 回まで回し、attempts 3 になっていた）。ndmreset を放してから hart が止まるのを待つのは手順ごとに 100 ms の時間（poll 400 回だった）。reset 自身の abstract command の cmderr による失敗は status fault を答える（timeout だった）。ハードウェアでの時間が変わる。host test。
- (EN) i2c-target on the ESP32-P4 and the classic ESP32 (fixture §3): the internal pull-ups the target enables on SDA / SCL while configured are declared - describe features bit2 and pullup_ohms (tag 0x42) 45000, the typical R_PU of both chips' datasheets (ESP32-P4 Table 5-4, ESP32 Table 5-3, DC characteristics) (they were enabled undeclared). Host test.
- (JA) ESP32-P4 と classic ESP32 の i2c-target（fixture §3）: configure の間 SDA / SCL に入れる内蔵のプルアップを宣言する。describe の features の bit2 と pullup_ohms（tag 0x42）45000、両チップのデータシートの R_PU の標準値（ESP32-P4 は Table 5-4、ESP32 は Table 5-3、DC characteristics）（宣言せずに入れていた）。host test。
- (EN) riscv-dm step (oep-if-debug §4.2): the hart is given dm_wait_ms (100 ms) to come back to debug mode by itself (it was 50 ms); if it does not, haltreq and up to 100 ms more. Stopped by that haltreq: dcsr.step cleared, DATA1 / DATA0 restored, status state with dpc_after valid (it answered ok). Still running: haltreq cleared and status state with answer TLV 0x01 step_left (length 0) - dcsr.step may still be set (it answered timeout, or line). Changes timing and the answer on hardware. Host tests.
- (JA) riscv-dm の step（oep-if-debug §4.2）: hart が自分で debug mode に戻るのを dm_wait_ms（100 ms）待つ（50 ms だった）。戻らなければ haltreq を出して、さらに 100 ms まで待つ。その haltreq で止まったら: dcsr.step を下ろし、DATA1 / DATA0 を戻し、dpc_after を有効にして status state（ok を答えていた）。まだ走っていたら: haltreq を下ろし、答えの TLV 0x01 step_left（長さ 0）を付けて status state - dcsr.step が立ったままかもしれない（timeout か line を答えていた）。ハードウェアでの時間と答えが変わる。host test。
- (EN) riscv-dm halt and resume wait dm_wait_ms (100 ms) of time (oep-if-debug §4, §4.2): halt re-issues haltreq for 100 ms (and no longer than the attach budget inside an attach) and, not halted by then, clears haltreq before it answers status timeout (it went by 8 rounds of polls and left haltreq set); resume looks for allresumeack (or running and not halted) for 100 ms before status state (it gave up after 25 reads, or after 3 that read halted). Changes timing on hardware. Host tests.
- (JA) riscv-dm の halt と resume は dm_wait_ms（100 ms）の時間だけ待つ（oep-if-debug §4、§4.2）: halt は 100 ms（attach の中では attach の予算を超えない）haltreq を出し直し、それまでに止まらなければ haltreq を下ろしてから status timeout を答える（poll の 8 周で決め、haltreq を立てたままにしていた）。resume は allresumeack（または running で halted でない）を 100 ms 探してから status state を答える（25 回の読み、または halted を 3 回読んだところで諦めていた）。ハードウェアでの時間が変わる。host test。
- (EN) Binds after a session (probe.config §1.2, common §1.3): a slot's console stream resumes from the position of the reset mark of the session's last host reset, not from where the stream had got to when the binds noticed the reset (the target's first bytes after the reset - its boot banner - could be skipped). A stream with no mark for that reset (opened by that attach) resumes from its end at the reset, and a fixture UART from its reception position then, as before. BindSource::hostResetMark gives the mark's position; the target console records it. Host test.
- (JA) session の後の bind（probe.config §1.2、common §1.3）: slot の console のストリームは、その session の最後のホストの reset の reset mark の位置から再開する。bind がその reset に気付いたときのストリームの位置からではない（reset の直後のターゲットの最初のバイト、起動のバナーを飛ばすことがあった）。その reset の mark の無いストリーム（その attach で開いたもの）は reset の時点の終わりから、fixture UART はその時点の受信位置から、これまでどおり再開する。BindSource::hostResetMark が mark の位置を返し、target の console がそれを記録する。host test。
- (EN) An unknown connection or stream is the last refusal (core §4.3 order 8): riscv-dm, arm-adi and the console's stream ops (read, marks, close, clear, mark, write) check the request's form and values first - malformed and unsupported come before no_connection (they answered no_connection before looking at the rest). New host test test_console.cpp (tests/host/run.sh); host tests.
- (JA) 知らない接続や stream は最後の拒否（core §4.3 order 8）: riscv-dm、arm-adi、console の stream の op（read、marks、close、clear、mark、write）は、まず request の形と値を検査する - malformed と unsupported は no_connection より先（残りを見る前に no_connection を答えていた）。新しい host test の test_console.cpp（tests/host/run.sh）。host test。
- (EN) gpio set (fixture §1.1, core §4.3 "Contradictions and undefined values"): a drive TLV pointing at an element whose mode is undefined (8 or more) is no longer malformed; the request is refused unsupported for that mode (payload 0x00 with the channel and index TLVs), a critical drive included. A drive on a defined mode other than 3 / 4 stays malformed. Host test.
- (JA) gpio の set（fixture §1.1、core §4.3「Contradictions and undefined values」）: mode が未定義（8 以上）の要素を指す drive の TLV はもう malformed にしない。request はその mode について unsupported で断る（payload 0x00 と channel と index の TLV）。critical な drive でも同じ。3 / 4 以外の定義済みの mode への drive はこれまでどおり malformed。host test。
- (EN) core §4.3's order in the wires' attach and scan and riscv-dm's reset: every format check of the request comes before anything refused unsupported. attach checks max_speed (absent, its length, 0), idle_clock, the pins and reset TLVs' lengths (and swd's targetsel) before the method, an unknown critical tag (swd's reset TLV among them), max_speed under min_clock_hz or a pair the declaration does not allow (the method was refused unsupported before max_speed was looked at); scan checks skip, max_speed and idle_clock before an unknown critical tag; reset checks the method TLV's length before mode 3+. Host tests.
- (JA) wire の attach と scan、riscv-dm の reset で core §4.3 の順: request の形の検査をすべて、unsupported で断るものより先にする。attach は max_speed（無い、長さ、0）、idle_clock、pins と reset の TLV の長さ（swd は targetsel も）を、method、知らない critical な tag（swd の reset TLV もその一つ）、min_clock_hz 未満の max_speed、宣言が許さない組より先に見る（max_speed を見る前に method を unsupported で断っていた）。scan は skip、max_speed、idle_clock を知らない critical な tag より先に見る。reset は method の TLV の長さを mode 3 以上より先に見る。host test。
- (EN) Line names (probe.config §1.3 step (c)): with at most one slot item, when no settings label names the line (`nrst` for the retry with reset), a firmware label of fn 0's describe (0x46) with that text is the line (they were not searched). findLine takes the firmware's describe TLVs; Endpoint::probeDescription returns what setProbeDescription was given. Host test.
- (JA) 線の名前（probe.config §1.3 の手順 (c)）: slot の項目が一つ以下で、どの設定の label もその線を名指さないとき（reset 付きの再試行の `nrst`）、fn 0 の describe のファームウェアの label（0x46）でその text のものが線になる（探していなかった）。findLine はファームウェアの describe の TLV を受け取る。Endpoint::probeDescription は setProbeDescription に渡したものを返す。host test。
- (EN) Saved settings at boot (probe.config §2): a saved bind whose port is not a serial port of this firmware makes the save unreadable with reason 2 (an interface pointed to does not exist / a bind's port is not a serial port), checked before any item is applied (it was reason 3, applying refused). Host test.
- (JA) 起動時の保存した設定（probe.config §2）: port がこのファームウェアのシリアルポートでない保存した bind は、reason 2（指す interface が無い / bind の port がシリアルポートでない）で保存を読めないものにする。どの項目を適用するよりも前に見る（reason 3、適用の拒否だった）。host test。
- (EN) Scan's listed combinations (oep-if-debug §1, core §4.3; rvswd, swio, swd): a combination the declaration does not allow is refused unsupported with tag 0x00 followed by TLV 0x40 index (u8, its position in the request) - the payload was 0x00 alone - and every listed combination is checked against the declaration before any of them is looked at for a held, disabled or idle channel or the seat (an allowed combination listed before one that is not answered unavailable). Oep.h: unsupportedIndex. Host tests.
- (JA) scan が挙げた組（oep-if-debug §1、core §4.3。rvswd、swio、swd）: 宣言が許さない組は、tag 0x00 の後に TLV 0x40 index（u8、request の中の位置）を付けて unsupported で断る（payload は 0x00 だけだった）。また、挙げた組をすべて宣言と照らしてから、どれかの channel が保持・disable・idle されているか、席があるかを見る（許されない組の前に許される組があると unavailable を答えていた）。Oep.h: unsupportedIndex。host test。
- (EN) Capture-group (oep-if-capture §4.1, §4.3): start checks every track's prerequisites before it starts any - its state, and a streaming track's subscription - and refuses unavailable cause 6 with TLV fn (0x05) naming the track (a streaming track without a subscription failed its own start after the others had started; GroupTrack::trackCanStart). A track failing after the group's start (the trigger track starting once the followers hold their pretrigger, or a track's start inside the group's) puts the group in state 6 with stopped reason 3, the other tracks stopped (the trigger track's failure stopped them silently and the group read state 1). bind over a budget names the track whose load goes over it in TLV fn. describe sends start_skew for every track, a typical 0 included (tracks with 0 were left out). status says 2 only while the trigger has not fired and a track is 2 or 3 (a track paused in state 5 made it 2). Host tests.
- (JA) capture-group（oep-if-capture §4.1、§4.3）: start は、どれかを始める前にすべての track の前提 - その状態と、streaming の track の subscription - を確かめ、unavailable cause 6 と、その track を名指す TLV fn（0x05）で断る（subscription の無い streaming の track は、ほかが始まった後で自分の start に失敗していた。GroupTrack::trackCanStart）。group の start の後で失敗した track（follower が pretrigger を持った後に始める trigger の track、group の start の中の track の start）は、group を状態 6、stopped reason 3 にし、ほかの track を止める（trigger の track の失敗は黙って止め、group は状態 1 を読んでいた）。budget を超える bind は、その負荷で超えた track を TLV fn で名指す。describe はすべての track に start_skew を送る、typical 0 も（0 の track は省いていた）。status が 2 を言うのは、trigger がまだで、track が 2 か 3 のときだけ（状態 5 で止まった track で 2 になっていた）。host test。
- (EN) ESP32-P4 logic started on its own after following a group's trigger (oep-if-capture §3.3, §2.2): a track configured immediate keeps the ring the following opened, and its own start is immediate again - no triggered event, trigger_index all ones in its segment (it sent triggered and gave the first sample as the trigger). Host test.
- (JA) group の trigger に従った後に単独で start した ESP32-P4 の logic（oep-if-capture §3.3、§2.2）: 即時と設定した track は following が開いた ring を保ち、単独の start は再び即時 - triggered の event を送らず、segment の trigger_index はすべて 1（triggered を送り、最初の sample を trigger としていた）。host test。
- (EN) ESP32-P4 logic as a capture-group follower (oep-if-capture §4.1, found while checking the immediate start after following): its start through the ring answered into a 4-byte buffer since the start answer carries the generation, so it failed, and a group with a logic track following another track's trigger answered start failed. Host test.
- (JA) capture-group の follower としての ESP32-P4 の logic（oep-if-capture §4.1。following の後の即時の start を確かめていて見つけた）: start の答えが generation を運ぶようになってから、ring を通した start が 4 バイトの buffer に答えていたので失敗し、別の track の trigger に従う logic の track を持つ group は start に failed を答えていた。host test。
- (EN) ESP32-P4 logic positions (oep-if-capture §2.2, §3.2): a repeat's segment positions, read positions and status write_pos are one space that counts the bytes discarded while it was paused (state 5) - the segments were numbered as if nothing had been discarded while write_pos counted it; read maps a position through the segments, so a position released or inside discarded bytes moves on to the next segment with the gap flag, and a segment cut short by a ring overrun is read at its own length. Streaming's new data discarded because every segment was still unsent now also shows as status flags bit0 (§2.1 rule 1; only the position jump showed it). Host tests.
- (JA) ESP32-P4 の logic の位置（oep-if-capture §2.2、§3.2）: repeat の segment の位置、read の位置、status の write_pos を、停止中（状態 5）に捨てたバイトを数える一つの空間にした - segment は何も捨てなかったかのように番号を振り、write_pos はそれを数えていた。read は位置を segment を通して引くので、解放した位置や捨てたバイトの中の位置は gap の flag を付けて次の segment に進み、ring の追い越しで短くなった segment はその長さで読む。すべての segment がまだ送られていないために捨てた streaming の新しいデータを、status の flags bit0 にも示す（§2.1 rule 1。位置の飛びでしか示していなかった）。host test。
- (EN) Classic ESP32 logic capture status in state 6 (oep-if-capture §3.2): the answer carries TLV 0x01 error (1, DMA / peripheral: the sampling task would not start), as the ESP32-P4's logic and the analog capture do (it carried none).
- (JA) classic ESP32 の logic capture の状態 6 の status（oep-if-capture §3.2）: ESP32-P4 の logic と analog capture と同じく、答えに TLV 0x01 error（1、DMA / 周辺回路: 取り込みの task が始まらなかった）を付ける（付けていなかった）。
- (EN) stop while capturing (oep-if-capture §3.2: state 3 -> 1, stopped 1, a short segment with flags bit1): the analog capture (immediate and triggered) and the ESP32-P4 logic's one-shot with a trigger now end in state 1 with the segment cut short - what came in, readable, counted by status and segments, sent as a segment event before stopped (the analog's immediate one went to state 4, its triggered one and the logic's dropped what they had); stop while waiting for the trigger ends in state 1 with nothing; a capture that completed before the stop was looked at stays complete (state 4). The classic ESP32's sampler cannot cut its window short (interrupts are off on its core): a stop while capturing lets it complete (state 4, stopped 0; it was state 4 with stopped 1). Not yet: the ESP32-P4 logic's one-shot without a trigger still drops its data on stop (the PARLIO receive tells no count before the end). Host tests.
- (JA) 取り込み中の stop（oep-if-capture §3.2: 状態 3 -> 1、stopped 1、flags bit1 の短い segment）: analog capture（即時と trigger 付き）と ESP32-P4 の logic の trigger 付き one-shot は、状態 1 で、segment を短く切って終わる - 入ったものを、読め、status と segments が数え、stopped の前に segment の event で送る（analog の即時は状態 4 になり、trigger 付きと logic は持っていたものを捨てていた）。trigger 待ちの stop は何も無しで状態 1。stop を見る前に終わった capture は完了のまま（状態 4）。classic ESP32 の sampler は window を途中で切れない（その core では割り込みを止めている）: 取り込み中の stop では完了させる（状態 4、stopped 0。状態 4 で stopped 1 だった）。未対応: ESP32-P4 の logic の trigger 無しの one-shot は、stop でまだデータを捨てる（PARLIO の受信は終わるまで数を言わない）。host test。
- (EN) Plan released or replaced (oep-if-capture §3.2): the logic capture of the ESP32-P4 and the analog capture go back to state 0 with their configuration, data and segments gone - read answers empty, status and segments count none (the ESP32-P4's logic kept its segment counts and mode, and a repeat's read then copied from the freed store; the analog kept its frame count, and a read after a plan with more channels ran past its buffer). Host tests (the logic capture's harvest now runs on the host fakes: the PARLIO's callbacks and receive buffer, a FreeRTOS task the test drives).
- (JA) plan の解放または置き換え（oep-if-capture §3.2）: ESP32-P4 の logic capture と analog capture は状態 0 に戻り、設定、データ、segment は無くなる - read は空、status と segments は何も数えない（ESP32-P4 の logic は segment の数と mode を保ち、repeat の read は解放した store から写していた。analog は frame の数を保ち、channel の多い plan の後の read は buffer の先まで読んでいた）。host test（logic capture の harvest を host の fake で動かす: PARLIO の callback と受信 buffer、test が動かす FreeRTOS の task）。
- (EN) Analog plan on a channel whose idle is an output (mode 3 / 4; core §8, oep-if-capture §1.2): refused unavailable cause 5 with the channel and holder_kind 7 (settings idle), nothing changed (it was taken, and the ADC took the pad off the idle's drive at start). Interface::planRefusalDetail gives the channel and holder_kind of an interface's own unavailable refusal to the endpoint. Host test.
- (JA) idle が出力（mode 3 / 4）の channel への analog の plan（core §8、oep-if-capture §1.2）: unavailable cause 5、その channel と holder_kind 7（設定の idle）で断り、何も変えない（受けてしまい、start で ADC が idle の駆動から pad を外していた）。Interface::planRefusalDetail が interface 自身の unavailable の拒否の channel と holder_kind を endpoint に渡す。host test。
- (EN) A track bound into a capture-group (oep-if-capture §4.1): plan_apply and plan_release of its fn are refused unavailable cause 4, holder_fn the group's fn, before anything changes (plan_release with n = 0 included; they went through and left the group bound to a track without its plan). Interface::boundTo() names the group (the logic captures and the analog capture say it); the endpoint checks it in plan_apply, plan_release and a settings plan replacement. Host tests (tests/host/test_capture_group.cpp).
- (JA) capture-group に束ねられた track（oep-if-capture §4.1）: その fn の plan_apply と plan_release は、何も変える前に unavailable cause 4、holder_fn は group の fn で断る（n = 0 の plan_release も。通ってしまい、plan の無い track を group が束ねたままになっていた）。Interface::boundTo() が group を言う（logic capture と analog capture が言う）。endpoint は plan_apply、plan_release、設定による plan の置き換えでそれを見る。host test（tests/host/test_capture_group.cpp）。
- (EN) ESP32-P4 logic configure / query (oep-if-capture §3.3, §3.5): a one-shot rate above the declared rate_limit for the channel count - 100 MHz up to 8 lines, 48 MHz up to 16 - is set to that limit, and actual_rate says so (the rate asked was taken up to 160 MHz for any count). Host test.
- (JA) ESP32-P4 の logic の configure / query（oep-if-capture §3.3、§3.5）: channel 数に対して宣言した rate_limit（8 本まで 100 MHz、16 本まで 48 MHz）を超える one-shot の rate は、その上限にして actual_rate で示す（どの本数でも 160 MHz まで求められたまま受けていた）。host test。
- (EN) Analog configure / query (oep-if-capture §3.3): a rate outside the declared rate_range is refused unsupported, tag 0x42 as received (it was clamped to the range silently); inside it, more channels than the rate_limit allows at that rate still get the nearest rate (the actual_rate says which). query answers what configure would: on the RP2 the layout's order and the skews follow the inputs' order of the round robin (query answered role order). Host tests.
- (JA) analog の configure / query（oep-if-capture §3.3）: 宣言した rate_range の外の rate は unsupported、受け取ったままの tag 0x42 で断る（黙って範囲に収めていた）。範囲の中なら、その rate で rate_limit が許すより多い channel でも最も近い rate にする（actual_rate がどれかを言う）。query は configure が答えるのと同じを答える: RP2 では layout の order と skew が round robin の入力の順に従う（query は role の順を答えていた）。host test。
- (EN) Capture configure / query (oep-if-capture §3.3, core §2.3, §4.3; the logic captures of the ESP32-P4 and the classic ESP32, the analog capture): mode, rate, trigger, pretrigger and frontend are critical whether or not bit 7 is set - a value the probe cannot honour, or one longer than it knows, refuses the request unsupported with the tag as received (Tail::refuseCritical), never ignored nor listed in ignored (sent without bit 7 they were ignored); samples and segments longer than known go by their bit. The checks follow core §4.3's order over the whole request: the form first (a known TLV shorter than its definition, a rate of 0, two analog frontends for one role: malformed), then an unknown critical tag and the values (unsupported; a trigger's or a frontend's role against the plan, or against the roles the capture has when there is none), then the plan (cause 6), the group (cause 4) and the state (cause 6); a bound track's configure, start, stop and force are refused cause 4 only after their request's form. An analog frontend for a role not planned is unsupported, tag as received (it was malformed). Host tests (the analog capture's RP2 build runs on host fakes of its ADC and DMA).
- (JA) capture の configure / query（oep-if-capture §3.3、core §2.3、§4.3。ESP32-P4 と classic ESP32 の logic capture、analog capture）: mode、rate、trigger、pretrigger、frontend は bit 7 に関わらず critical - probe が従えない値や、知っているより長い値は、受け取ったままの tag で unsupported として断り、無視も ignored への記載もしない（Tail::refuseCritical。bit 7 無しで送られると無視していた）。知っているより長い samples と segments はその bit に従う。確認は core §4.3 の順で request 全体にかける: まず形（定義より短い既知の TLV、0 の rate、一つの role への二つの analog frontend: malformed）、次に知らない critical な tag と値（unsupported。trigger や frontend の role は plan に対して、plan が無ければ capture の持つ role に対して）、次に plan（cause 6）、group（cause 4）、状態（cause 6）。束ねられた track の configure、start、stop、force を cause 4 で断るのは request の形を見た後。plan に無い role への analog の frontend は unsupported、受け取ったままの tag（malformed だった）。host test（analog capture の RP2 版を ADC と DMA の host の fake で動かす）。
- (EN) probe.config on a probe with slots_max 0 (no wire place added; probe.config §4: 0 does not handle slots): describe's items leave slot out, and a slot item in set / unset is refused unsupported with the item's tag as received (it listed slot and refused the item malformed). Host test.
- (JA) slots_max 0 の probe の probe.config（wire の place を足していない。probe.config §4: 0 は slot を扱わない）: describe の items は slot を入れず、set / unset の slot の項目は受け取ったままの項目の tag で unsupported で断る（slot を挙げ、項目を malformed で断っていた）。host test。
- (EN) probe.config idle item (core §4.3 "Contradictions and undefined values"): a mode of 5 or more that carries a drive is refused unsupported with the item's tag, for the mode (it was malformed, for a drive on a mode other than 3 / 4); a value of 4 or 5 bytes stays malformed first. Host test.
- (JA) probe.config の idle の項目（core §4.3「Contradictions and undefined values」）: drive を持つ 5 以上の mode は、mode について項目の tag で unsupported で断る（3 / 4 以外の mode の drive として malformed だった）。4 か 5 バイトの値はこれまでどおり先に malformed。host test。
- (EN) probe.config label item (probe.config §1): a text of 0 or more than 32 bytes (registry label_max_bytes), or one that is not valid text (C0 controls, 0x7F, invalid UTF-8), is refused malformed, and a channel at or beyond channels or reserved unsupported with the item's tag (both were taken). A probe without a pin table no longer declares label (as idle and disable: it has no channels to name). Host test.
- (JA) probe.config の label の項目（probe.config §1）: 0 バイトか 32 バイト（registry label_max_bytes）を超える text、正しい text でない（C0 制御文字、0x7F、不正な UTF-8）text は malformed、channels 以上か reserved の channel は項目の tag で unsupported で断る（どちらも受けていた）。pin の表を持たない probe はもう label を宣言しない（idle と disable と同じ。名付ける channel が無い）。host test。
- (EN) Names and tokens (core §13 rule 1, §7.5): Endpoint::add refuses an interface whose name is outside the form (1 to 64 bytes, two or more labels of a-z 0-9 - not starting or ending with -); describeCore fails a model or a unit_id outside a-z 0-9 -, 1 to 32 bytes (it checked the unit_id's length only). Every name and model the library and its examples use passes. Host test.
- (JA) 名前と token（core §13 規則 1、§7.5）: Endpoint::add は形から外れた名前（1 から 64 バイト、- で始まらず終わらない a-z 0-9 - の label が二つ以上）の interface を断る。describeCore は a-z 0-9 -、1 から 32 バイトから外れた model と unit_id で失敗する（unit_id の長さしか見ていなかった）。ライブラリと例が使う名前と model はすべて通る。host test。
- (EN) list's instance (core §7.2): the endpoint numbers the interfaces with the same (name, revision) from 0 in ascending fn itself (Endpoint::instanceOf), in list, the list hash, probe.config's identities and a bind's line labels - the instance an interface was built with is no longer trusted (a sketch that gave two the same number listed them so). Host test.
- (JA) list の instance（core §7.2）: 同じ（name、revision）の interface を fn の昇順に 0 から、endpoint が自分で数える（Endpoint::instanceOf）。list、list の hash、probe.config の identity、bind の行の label で使う。interface を作ったときの instance はもう信じない（二つに同じ番号を与えたスケッチはそのまま列挙していた）。host test。
- (EN) Core audit, limits: max_inflight in confirm is at most the resend table's entries (8; core §5.2 asks the table to remember at least max_inflight requests) - the endpoint lowers a larger value, and LogicCapture declares 8 (it declared 16). A closed resource number is not taken again while it is among the last 1024 closed (registry resource_reuse_distance, core §9; it kept 64). Pending pushes in a transport are held to max_frame x 2 (core §11.4): setPushQueue(0) and larger values give that bound (0 meant no limit; the default 1024 stays where max_frame is 512 or more). Host tests.
- (JA) core の監査、上限: confirm の max_inflight は resend の表の数（8）以下（core §5.2 は表が少なくとも max_inflight 個の request を覚えることを求める）。endpoint はそれより大きい値を下げ、LogicCapture は 8 を宣言する（16 を宣言していた）。閉じた resource の番号は、最後に閉じた 1024 個の中にある間は再び取らない（registry の resource_reuse_distance、core §9。64 個だった）。transport に待つ push は max_frame x 2 まで（core §11.4）: setPushQueue(0) とそれより大きい値はその上限になる（0 は上限なしだった。既定の 1024 は max_frame が 512 以上なら変わらない）。host test。
- (EN) Core audit against core §1.2 / §7.5 / §12 and the conformance checklist (after the confirm finding). The header refusals (core §4.3 order 1: an fn the probe does not have, an op it does not offer) come before the resend table: a 0x81 request of the last session refused so is no longer kept nor advances the newest corr, and no longer restarts the lease (core §5.2, §6.1). A known request TLV that appears twice is malformed, critical or not (core §2.3; Tail::parse - role_assignment, gpio set's drive and analog configure's frontend are the tags that repeat). list: flags bits 1 to 7 are refused unsupported tag 0x00, and a prefix that is not valid text malformed (core §7.2, §2.1). open: force other than 0 / 1, and an owner of 0 or over 32 bytes or that is not valid text (C0 controls, 0x7F, invalid UTF-8), are malformed (core §2.1, §6.4; an over-long owner was ignored). port_speed: a step of 3 or more is unsupported tag 0x00 (it was malformed; core §2.5, §3.5), a try with verify_ms 0 malformed (it was taken). Length-prefixed frames: a length over max_frame discards that frame and the input up to the next 200 ms pause (core §3.1; the reader hunted for a new length byte by byte inside the discarded input); on TCP a pause inside a frame no longer restarts the read (core §3.2). HID (the ESP32-P4 firmware, MultipleTransports): a report whose count is over what the report carries is discarded whole (core §3.1; it was cut to the report). Host tests.
- (JA) core §1.2 / §7.5 / §12 と conformance の一覧に対する core の監査（confirm の件の後）。header の拒否（core §4.3 order 1: probe に無い fn、提供しない op）を resend の表より前にした: そう断られた最後の session の 0x81 の request は、もう表に残らず、newest corr を進めず、lease も再開しない（core §5.2、§6.1）。知っている request の TLV が二度出たら critical かに関わらず malformed（core §2.3。Tail::parse。繰り返す tag は role_assignment、gpio set の drive、analog configure の frontend）。list: flags の bit 1 から 7 は unsupported tag 0x00、正しい text でない prefix は malformed（core §7.2、§2.1）。open: 0 / 1 以外の force、0 バイトか 32 バイトを超えるか正しい text でない（C0 制御文字、0x7F、不正な UTF-8）owner は malformed（core §2.1、§6.4。長すぎる owner は無視していた）。port_speed: 3 以上の step は unsupported tag 0x00（malformed だった。core §2.5、§3.5）、verify_ms 0 の try は malformed（受けていた）。長さ前置きの frame: max_frame を超える長さは、その frame と次の 200 ms の休止までの入力を捨てる（core §3.1。捨てる入力の中で新しい長さを 1 バイトずつ探していた）。TCP では frame の途中の休止で読み直さない（core §3.2）。HID（ESP32-P4 のファームウェア、MultipleTransports）: report が運べるより大きい count の report は丸ごと捨てる（core §3.1。report の長さに切っていた）。host test。
- (EN) confirm's transport TLV (core §7.1; found by the release-candidate hardware test, every board): the answer always carries TLV 0x01 transport (u8), the index in fn 0's describe of the transport the confirm came on - UART bridge, USB CDC, built-in USB serial, vendor bulk, HID and TCP alike (it carried none). A range with min_rev > max_rev is refused malformed (it was unsupported), and a range without revision 1 unsupported with payload tag 0x00 followed by TLV 0x01 supported (min 1, max 1) (the TLV was missing). describe of fn 0 always carries discoverable (core §7.5): 0 on a probe that does not enumerate with the project's VID:PID (it was left out). Host test (tests/host/test_core_conformance.cpp: every transport kind, the confirm vectors of oep-spec tests/vectors/confirm.json byte for byte).
- (JA) confirm の transport の TLV（core §7.1。リリース候補のハードウェア試験で見つかった。すべてのボード）: 答えにいつも TLV 0x01 transport（u8）、confirm が来た transport の fn 0 の describe での index を付ける。UART bridge、USB CDC、内蔵 USB シリアル、vendor bulk、HID、TCP のどれでも（付けていなかった）。min_rev > max_rev の範囲は malformed で断る（unsupported だった）。revision 1 を含まない範囲は unsupported、payload は tag 0x00 の後に TLV 0x01 supported（min 1、max 1）（TLV が無かった）。fn 0 の describe はいつも discoverable を入れる（core §7.5）: プロジェクトの VID:PID で列挙しない probe では 0（入れていなかった）。host test（tests/host/test_core_conformance.cpp: すべての transport の種類、oep-spec の tests/vectors/confirm.json の confirm を 1 バイトずつ）。
- (EN) USB VID:PID 1209:4F45 (breaking): the project's own VID:PID (registry synced from oep-spec d50a84e: usb project_vid / project_pid). The ESP32-P4 firmware's HS device (vendor bulk, HID, CDC and the in-app DFU interface) and the RP2040 / RP2350 firmware enumerate with it, serial number = unit_id; MultipleTransports and LogicCapture likewise. describe discoverable is 1 on the RP2 always, on the ESP32-P4 once its HS port has enumerated (a board with only USB-Serial/JTAG wired keeps 0), never on the classic ESP32 (a UART bridge). iProduct is only a name for people: the iProduct `OEP` clue and the temporary board-default ID (303a:0002, arduino-pico's) are gone, with no compatibility path. PID-USE (EN / JA): the VID:PID is obtained, the temporary-ID section and the iProduct condition are removed, the whole USB device (its in-app DFU included) is covered. After flashing, a host system grants access anew for 1209:4F45: on Linux the udev rule in oep-client-python's udev/70-oep-probe.rules (administrator rights), on Windows / WSL a new usbipd bind (administrator rights).
- (JA) USB の VID:PID 1209:4F45（破壊的変更）: プロジェクト自身の VID:PID（registry を oep-spec d50a84e に合わせた: usb の project_vid / project_pid）。ESP32-P4 の firmware の HS の device（vendor bulk、HID、CDC、アプリの中の DFU のインターフェース）と RP2040 / RP2350 の firmware はこれで列挙し、serial number は unit_id。MultipleTransports と LogicCapture も同じ。describe の discoverable は RP2 ではいつも 1、ESP32-P4 では HS の口が列挙した後に 1（USB-Serial/JTAG だけを配線した基板では 0 のまま）、classic ESP32（UART bridge）では 0。iProduct は人のための名前だけ: iProduct の `OEP` の手がかりと、ボードの既定の仮の ID（303a:0002、arduino-pico のもの）は無くなり、互換の経路は無い。PID-USE（EN / JA）: VID:PID は取得済み、仮の ID の節と iProduct の条件を消し、USB の device 全体（アプリの中の DFU も）が範囲。焼いた後は、host の OS で 1209:4F45 への権限をあらためて与える: Linux では oep-client-python's udev/70-oep-probe.rules の udev の規則（管理者の権限）、Windows / WSL では usbipd の bind をやり直す（管理者の権限）。
- (EN) Pins from boot (core §8, △12): every example now puts each channel it does not reserve in its free state (Hi-Z, no pull) before it answers anything. The ESP32-P4 firmware parks its channels after reading the saved settings (their disable items' channels stay untouched, as on the RP2 and the classic ESP32; it left the pins as the chip boots them); CustomInterface, MultipleTransports and LogicCapture park theirs. The debug-only examples (RvswdDebugProbe, SwioDebugProbe, SwdDebugProbe) declare the channels up to their wire's pins, the wire's pins as channels and the ones below reserved (they declared 64 / 40 / 30 channels with only the pair reserved and parked none, or only the pair), and park the wire's pins. MinimalProbe has no channels.
- (JA) 起動からの pin（core §8、△12）: どの例も、予約しない channel をすべて、何かに答える前に空きの状態（Hi-Z、pull 無し）にする。ESP32-P4 のファームウェアは保存した設定を読んだ後に channel を空きにする（その disable の項目の channel には触らない。RP2 と classic ESP32 と同じ。これまではチップの起動のままにしていた）。CustomInterface、MultipleTransports、LogicCapture も空きにする。debug だけの例（RvswdDebugProbe、SwioDebugProbe、SwdDebugProbe）は、wire の pin までを channel と宣言し、wire の pin を channel、それより下を予約とする（64 / 40 / 30 の channel を宣言して組だけを予約し、何も、または組だけを空きにしていた）。そして wire の pin を空きにする。MinimalProbe には channel が無い。
- (EN) Position streams' marks (core §2.6, C-40): the ring's slots and the count of marks kept no longer derive from the u32 serial, so they stay right past its wrap (they did only below 2^32 marks). Comment: attach's reset TLV on a live connection is no longer called the reset op's NRST (○2; the reset op runs ndmreset only and drives no line, as before).
- (JA) 位置付きストリームの mark（core §2.6、C-40）: ring の場所と保っている mark の数を u32 の serial から求めないので、serial が一周しても正しい（2^32 個の mark までしか正しくなかった）。コメント: 生きている接続への attach の reset TLV を、もう reset op の NRST と呼ばない（○2。reset op はこれまでどおり ndmreset だけを行い、線を動かさない）。
- (EN) describe's transport interface (core §7.5, C-41): the examples on a USB CDC transport name its communication interface - MinimalProbe, FixtureProbe, CustomInterface and RvswdDebugProbe on the RP2, SwdDebugProbe and ProbeConfig send 0 (they sent 0xFF); a UART bridge stays 0xFF, and the P4's USB-Serial/JTAG 0xFF (the probe cannot know its number). The endpoint's constructor documents the rule.
- (JA) describe の transport の interface（core §7.5、C-41）: USB CDC の transport の例は、その communication interface を名指す。RP2 の MinimalProbe、FixtureProbe、CustomInterface、RvswdDebugProbe と、SwdDebugProbe、ProbeConfig は 0 を送る（0xFF を送っていた）。UART bridge は 0xFF のまま、P4 の USB-Serial/JTAG も 0xFF（番号を probe は知り得ない）。endpoint のコンストラクタにその規則を書いた。
- (EN) Required and optional ops (core §1.2, oep-spec 2e70f40): an op an interface does not offer - one its document does not define, or an optional one it does not declare - is answered unknown_operation before the session is looked at (core §4.3 order 1; an op needing the lock answered session_required or locked first). Interface::offers(op) says which ops an interface offers; every library interface declares its op table (capture query / force and capture-group force are declared by their features, i2c-target stretch only where clock stretching is, probe.config save / erase with its storage, the riscv-dm optional ops by features, wire scan on every wire, arm-adi all). An interface of a sketch's own that does not override it leaves the answer to handle(). Host test.
- (JA) 必須と任意の op（core §1.2、oep-spec 2e70f40）: interface が提供しない op（文書が定めない op、宣言していない任意の op）には、session を見る前に unknown_operation を答える（core §4.3 order 1。lock の要る op には session_required や locked を先に答えていた）。Interface::offers(op) がどの op を提供するかを言う。ライブラリのすべての interface が自分の op の表を宣言する（capture の query / force と capture-group の force は features で宣言、i2c-target の stretch は clock stretching のある所だけ、probe.config の save / erase は storage とともに、riscv-dm の任意の op は features で、wire の scan はすべての wire に、arm-adi はすべて）。上書きしないスケッチ自身の interface は handle() に任せる。host test。
- (EN) i2c-target configure (fixture §3, △5): an address in 0x00 - 0x07 or 0x78 - 0x7F (reserved by the I2C specification) is refused unsupported, payload 0x00. Host test.
- (JA) i2c-target の configure（fixture §3、△5）: 0x00 - 0x07 と 0x78 - 0x7F のアドレス（I2C の仕様が予約）は unsupported、payload 0x00 で断る。host test。
- (EN) oep.wire.swio scan (oep-if-debug §3): a listed combination whose swclk is not 0xFFFF is one the declaration does not allow - unsupported, payload 0x00, as in attach (it was malformed). Host test.
- (JA) oep.wire.swio の scan（oep-if-debug §3）: swclk が 0xFFFF でない組を挙げたものは、宣言が許さない組 - attach と同じく unsupported、payload 0x00（malformed だった）。host test。
- (EN) core §4.3's order over a whole request (C-21): plan_apply checks the form of every role_assignment first (another length, and fn 0 - core §8's table says malformed; it answered unknown_function), then the fns named inside it (unknown_function, the end of order 5), then an unknown critical tag (unsupported), then the count against plan_roles. probe.config set refuses malformed anywhere first, then an fn named inside an item (unknown_function; a slot's wire fn now before its undefined attach, a bind's uart fn before an undefined stream kind), then the first item it cannot take (unsupported, that item's tag), then a count (unavailable cause 2; a bind's stream count now after the rest); an earlier item no longer wins over a later item's malformed. unset checks every key's form before an undeclared tag. A request's TLV tail is checked whole before an unknown critical tag is refused (Tail::parse). Host tests.
- (JA) core §4.3 の順を request 全体にかける（C-21）: plan_apply は、まずすべての role_assignment の形（違う長さ、fn 0 - core §8 の表は malformed。unknown_function を答えていた）、次に中で名指す fn（unknown_function、order 5 の終わり）、次に知らない critical な tag（unsupported）、最後に plan_roles に対する数。probe.config の set は、どこかの malformed を最初に、次に項目の中で名指す fn（unknown_function。slot の wire fn を未定義の attach より前に、bind の uart の fn を未定義の stream の kind より前に）、次に受けられない最初の項目（unsupported、その項目の tag）、次に数（unavailable cause 2。bind の stream の数はほかより後に）で断る。前の項目が後の項目の malformed に勝つことはもう無い。unset は未宣言の tag より前にすべての key の形を見る。request の TLV の尾は、知らない critical な tag を断る前に全体の形を確かめる（Tail::parse）。host test。
- (EN) plan_apply / plan_release on a probe none of whose interfaces has plan roles (core §1.2, §12: the debug-only examples - RvswdDebugProbe, SwioDebugProbe, SwdDebugProbe - and MinimalProbe answered them): unknown_operation, before the session is looked at, and its describe leaves plan_roles out. Interface::planRoles() says whether an interface has plan roles (the fixtures, the captures, the i2c / spi targets and the CustomInterface example say yes). Host test.
- (JA) どの interface にも plan の role が無い probe への plan_apply / plan_release（core §1.2、§12。debug だけの例 RvswdDebugProbe、SwioDebugProbe、SwdDebugProbe と MinimalProbe が答えていた）: unknown_operation を、session を見る前に答え、describe は plan_roles を入れない。Interface::planRoles() が interface に plan の role があるかを言う（fixture、capture、i2c / spi target、CustomInterface の例は yes）。host test。
- (EN) confirm's values within core §7.1's bounds (C-20): the endpoint raises a sketch's Limits to max_frame 64 or more, window max_frame or more, max_inflight 1 or more (a host treats a transport answering outside them as not usable). The interface list is fixed for a boot (core §7.2, C-39): Endpoint::add after the first poll() is refused. max_op_ms (core §7.5, C-47): a static_assert keeps the declared value within 1 to limits.max_op_ms_max (600000). Host tests.
- (JA) confirm の値を core §7.1 の範囲に収める（C-20）: endpoint はスケッチの Limits を max_frame 64 以上、window max_frame 以上、max_inflight 1 以上に引き上げる（範囲外を答える transport は host が使えないものとする）。interface の一覧は boot の間変わらない（core §7.2、C-39）: 最初の poll() の後の Endpoint::add は断る。max_op_ms（core §7.5、C-47）: 宣言する値が 1 から limits.max_op_ms_max（600000）に収まることを static_assert で確かめる。host test。
- (EN) boot_id (core §6.5, C-19): the endpoint picks it itself when the first message arrives unless the sketch sets one - a hardware random source where the platform has one (ESP32 esp_random, RP2 hwrand32), elsewhere the microsecond timer's count at that external event, mixed (a sketch that never called setBootId sent 0 at every boot, and platformRandom32 on such a platform read the timer at a fixed point of setup()). The examples no longer call setBootId; setBootId stays for a sketch with a better source (a boot counter in non-volatile storage). The clock (core §2.6a, C-31): on a platform whose micros() is 32 bits (not ESP32 / RP2) nowNs extends it with a count of its wraps, read on every poll(), so it no longer wraps at 71.6 minutes. Host tests.
- (JA) boot_id（core §6.5、C-19）: スケッチが決めなければ、最初のメッセージが届いたときに endpoint が自分で選ぶ。プラットフォームにハードウェアの乱数源があればそれ（ESP32 の esp_random、RP2 の hwrand32）、無ければその外からの出来事の時点のマイクロ秒タイマーの値を混ぜたもの（setBootId を呼ばないスケッチは毎回 0 を送り、そういうプラットフォームの platformRandom32 は setup() の決まった時点でタイマーを読んでいた）。例はもう setBootId を呼ばない。setBootId は、もっと良い源（不揮発の記憶に保つ起動回数）を持つスケッチのために残す。時計（core §2.6a、C-31）: micros() が 32 bit のプラットフォーム（ESP32 / RP2 以外）では、nowNs が一周の回数で延ばす。poll() のたびに読むので、71.6 分で一周しなくなった。host test。
- (EN) Comments and guides point to the renumbered oep-spec guides: host guide §4 (finding a USB probe; was §1.7), probe guide §3 / §5 / §7 / §8 (were §2.5 / §3.5 / §3.7 / §3.8), and oep-if-capture §3.3 for the logic capture's configure TLVs (was a section of the design record).
- (JA) コメントとガイドが、番号の振り直された oep-spec のガイドを指すようにした: host ガイド §4（USB の probe を探す。旧 §1.7）、probe ガイド §3 / §5 / §7 / §8（旧 §2.5 / §3.5 / §3.7 / §3.8）、logic capture の configure の TLV は oep-if-capture §3.3（設計の記録の節を指していた）。
- (EN) Analog capture, the answer's scale (capture §1.2 rule 4, §3.3 tag 0x55): zero and scale_nv come from the frontend's declared range (§3.5 tag 0x46), value 0 at range_min_mv and full scale at range_max_mv. The classic ESP32's ranges start above 0 V (100 mV at 0 / 2.5 dB, 150 mV at 6 / 12 dB) and zero was sent as 0 with scale_nv = range_max / 4095, so every voltage read up to 150 mV low (WireSkein): now 12 dB answers zero -267, scale_nv 561661 (0 / 2.5 / 6 dB: -482 / -356 / -384). The ESP32-P4's and the RP2's ranges start at 0 V and answer as before. Host test (tests/host/test_analog_scale.cpp).
- (JA) analog capture の答えの scale（capture §1.2 規則 4、§3.3 tag 0x55）: zero と scale_nv を frontend の宣言した範囲（§3.5 tag 0x46）から求める。値 0 が range_min_mv、満量程が range_max_mv。classic ESP32 の範囲は 0 V より上から始まる（0 / 2.5 dB で 100 mV、6 / 12 dB で 150 mV）のに、zero を 0、scale_nv を range_max / 4095 で送っていたので、どの電圧も最大 150 mV 低く読めた（WireSkein）。いまは 12 dB で zero -267、scale_nv 561661（0 / 2.5 / 6 dB: -482 / -356 / -384）。ESP32-P4 と RP2 の範囲は 0 V から始まり、答えは変わらない。host test（tests/host/test_analog_scale.cpp）。
- (EN) Registry synced from oep-spec 6280db2 (registry hash 3925694ca2f8bb1e): limits.max_op_ms_max and the other new limits, the jitter_kind / rate_accuracy_how / reference_how enums; mark_detail_reset 2 (nrst) and the rvswd / swio scan_kind arm_adi are gone (○2, △10). tools/sync_registry.sh leaves out the generated header's own SPDX line (it came out twice).
- (JA) registry を oep-spec 6280db2 に合わせた（registry hash 3925694ca2f8bb1e）: limits.max_op_ms_max ほかの新しい limit、jitter_kind / rate_accuracy_how / reference_how の enum。mark_detail_reset 2（nrst）と rvswd / swio の scan_kind arm_adi は無くなった（○2、△10）。tools/sync_registry.sh は生成された header 自身の SPDX 行を除く（二度出ていた）。
- (EN) P4 logic capture, streaming (capture §2.1 / §3.3): with no internal RAM left for two zero-copy stages beside the DMA ring (taken by an earlier trigger's ring and the rest of the firmware), configure in mode 3 answered failed - it now streams the copied way, the segments in the store and pushed by the endpoint like any notification. One-shot with a trigger and samples above the limit: checked to round down to the 64 KiB segment (actual_samples), with and without PSRAM, at w = 1 / 2 / 16, for samples up to 0xFFFFFFFF, configure and query. New host test of the capture's configure on fakes of the PARLIO RX driver and the heap (tests/host/test_capture.cpp). Which of the two the bench's failure at 627451 Hz was is for the bench to confirm.
- (JA) P4 の logic capture、streaming（capture §2.1 / §3.3）: DMA の ring の横に zero-copy の stage 二つ分の内部 RAM が残っていない（先の trigger の ring とファームウェアの残りが取っている）とき、mode 3 の configure は failed を答えていた。いまはコピーする道で流す。segment は store に入り、endpoint がほかの通知と同じく push する。trigger 付きで上限を超える samples の one-shot: 64 KiB の segment まで切り下げる（actual_samples）ことを、PSRAM の有り無し、w = 1 / 2 / 16、0xFFFFFFFF までの samples、configure と query で確かめた。PARLIO RX ドライバとヒープの fake の上で capture の configure を見る host test を足した（tests/host/test_capture.cpp）。bench の 627451 Hz での失敗がどちらだったかは bench で確かめる。
- (EN) rvswd / swio attach to a live connection whose module does not answer (a DMSTATUS of all zeros / all ones, or nothing back): the same connection, its link brought up afresh under max_speed within the attach budget (a fresh speed search, a pending havereset acknowledged, search_retries in the answer), then the method as usual. 0.0.28+68d9694 on the bench (ESP32-P4, SWIO, CH32V003): after the target's power came back, every attach answered timeout - the stale link read all ones, which looked halted, and the halt never landed - until a forced detach. The connection is kept rather than closed: an attach returns the live connection (oep-if-debug §1) and it closes only on wire loss (§2); a bring-up that fails answers line and the connection closes once the wire is lost. Checked the same elsewhere: RVSWD's PHY already woke a sleeping link inside the read (its parity fails), swd's attach to a live connection already brought the port back through the DPIDR read's wire retries (line reset, dormant wake), and a slot's automatic attach takes a live connection only when its DMSTATUS answers. Host tests (a stale SWIO-like link, the RVSWD simulation, the SWD simulation).
- (JA) module が答えない（DMSTATUS が all zeros / all ones、または何も返らない）生きている connection への rvswd / swio の attach: 同じ connection のまま、link を max_speed の下で attach budget の内に新しく立ち上げ直し（速度探索をやり直し、保留中の havereset を ack し、答えに search_retries）、その後 method の通りに進む。bench での 0.0.28+68d9694（ESP32-P4、SWIO、CH32V003）: target の電源が戻った後、force detach するまで attach は毎回 timeout を答えていた。古い link が all ones を読み、それが halt 済みに見え、halt が決して通らなかった。connection は閉じずに保つ: attach は生きている connection を返し（oep-if-debug §1）、閉じるのは wire loss の時だけ（§2）。立ち上げに失敗すれば line を答え、wire が lost になった時点で閉じる。他の場所も確認: RVSWD の PHY は眠った link を read の中ですでに起こしていた（parity が合わない）。swd の生きている connection への attach は DPIDR の read の wire retry（line reset、dormant の wake）ですでに port を戻していた。slot の自動 attach は DMSTATUS が答える時だけ生きている connection を取る。host test（SWIO 風の古い link、RVSWD の simulation、SWD の simulation）。
- (EN) Wire loss looked at on every request on a connection, whatever the op answered (oep-if-debug §2; 0.0.28+68d9694 on the bench: with the target's power floating, DMSTATUS read all ones, a host's dmi steps answered ok and the connection never closed). riscv-dm: after any op, a wire-loss clock that has run out turns the answer into status line (its form kept) and the connection closes after it; a dmi read or poll step of DMSTATUS that reads all zeros / all ones fails with line and no value - the same reads the clock counts as no answer (a poll for allhalted was met by all ones), and the step at which the clock runs out fails with line; resume no longer takes all ones as allresumeack; step / read_block / write_block / run on a hart whose DMSTATUS does not answer say line, not state. rvswd / swio: attach to a live connection answers line and closes it whenever the clock has run out (not only when the attach itself failed line); scan through the live connection closes it once lost. swd: scan through the live connection runs the clock too and closes it once lost; arm-adi already looked at it after every op (an SWD ACK OK is always an answer). Host tests.
- (JA) wire loss を connection 上のすべての request で、op の答えに関わらず確認する（oep-if-debug §2。bench での 0.0.28+68d9694: target の電源が浮いて DMSTATUS が all ones を読み、host の dmi の step は ok を答え、connection は閉じなかった）。riscv-dm: どの op の後でも wire-loss の clock が尽きていれば答えを status line にし（形はそのまま）、その後に connection を閉じる。DMSTATUS を読む dmi の read / poll の step が all zeros / all ones を読んだら、値なしで line で失敗する（clock が答えなしと数えるのと同じ読み。allhalted を待つ poll が all ones で満たされていた）。clock が尽きた step も line で失敗する。resume は all ones を allresumeack と取らない。DMSTATUS が答えない hart への step / read_block / write_block / run は state ではなく line。rvswd / swio: 生きている connection への attach は、clock が尽きていれば常に line を答えて閉じる（attach 自体が line で失敗した時だけではなく）。生きている connection を通した scan も lost なら閉じる。swd: 生きている connection を通した scan も clock を進め、lost なら閉じる。arm-adi はすでに毎 op の後に見ていた（SWD の ACK OK は常に答え）。host test。
- (EN) SPI target (classic ESP32, fixture §4): describe cs_setup_ns is 15000 ns, from the bench (a CH32V003 master, the console of an attached debug wire running): the first bit right from 2.0 us after CS fell at 1 MHz and 3.1 us at 3 MHz, and MISO still driven up to 11.9 us after CS rose - one value covers both, with margin (it was 10000, an estimate). Modes 1 and 3 are no longer offered: there the slave drives 0 on MISO from CS falling to the first SCK edge whatever the first bit, which the gate cannot fix; configure with mode 1 / 3 is refused unsupported, payload tag 0x00 (core §4.3 order 6, before a missing plan's unavailable). max_clock_hz stays 3 MHz (the verified rate). The CS gate itself is unchanged. The ESP32-P4 is unchanged (no gate, no cs_setup_ns, modes 0-3). Host tests.
- (JA) SPI target（classic ESP32、fixture §4）: describe の cs_setup_ns を 15000 ns にした。bench（CH32V003 の master、attach した debug wire の console が動作中）の値: CS が下がってから最初の bit が正しくなるのは 1 MHz で 2.0 us から、3 MHz で 3.1 us から。CS が上がってから MISO はまだ 11.9 us 駆動されていた。一つの値で両方を余裕を持って覆う（これまでは見積もりの 10000）。mode 1 / 3 は提供しない: そこでは slave が CS の立ち下がりから最初の SCK の edge まで、最初の bit に関わらず MISO に 0 を出し、gate では直せない。mode 1 / 3 の configure は unsupported（payload の tag 0x00、core §4.3 の順 6。plan が無い時の unavailable より先）。max_clock_hz は検証済みの 3 MHz のまま。CS の gate 自体は変えていない。ESP32-P4 は変わらない（gate も cs_setup_ns も無く、mode 0-3）。host test。
- (EN) P4 logic capture, repeat (capture §3.3: a samples above the limit is rounded down, the answer authoritative): the segment size is worked out in 64 bits - samples x width overflowed u32 from 2^28 samples at w = 16, the segment came out 0 bytes and configure failed. The other configure paths were checked and round down already: the P4 one-shot with or without a trigger (to the 64 KiB segment), streaming (samples is a hint), the classic sampler (65408), the analog (its buffer, and on the RP2 with a trigger half the DMA ring).
- (JA) P4 の logic capture、repeat（capture §3.3: 上限を超える samples は上限に切り下げ、答えの値が正）: segment の大きさを 64 bit で求める。w = 16 で 2^28 samples から samples x width が u32 を溢れ、segment が 0 byte になって configure が failed になっていた。他の configure の道は確認済みで、すでに切り下げている: P4 の one-shot（trigger の有無を問わず 64 KiB の segment に）、streaming（samples は目安）、classic の sampler（65408）、analog（その buffer、RP2 で trigger 付きなら DMA の ring の半分）。
- (EN) RVSWD attach, search_retries (oep-if-debug §1): each wake of the wake / configuration sequence that got no answer before the one that did counts 1 (they were not counted). SWIO has no wake pattern; SWD's dormant wake after the JTAG-to-SWD switch is part of its sequence, and each wake / configuration attempt that failed already counts.
- (JA) RVSWD の attach、search_retries（oep-if-debug §1）: wake / 構成の sequence の wake のうち、答えたものの前に答えの無かったものをそれぞれ 1 と数える（数えていなかった）。SWIO には wake の pattern が無い。SWD の JTAG-to-SWD の後の dormant の wake はその sequence の一部で、失敗した wake / 構成の試みはすでに数えている。
- (EN) The attach budget (oep-if-debug §1, limits.attach_budget_ms) is a hard bound on one attach answer of oep.wire.rvswd / swio, a reset's hold_ms aside: a read check or write check running when the search's deadline passes ends there (it ran to its end); the reserve after the search is wire_retry_ms + 100 ms (250 did not cover a request's wire retries and the running step); attachUnderReset's halt wait stops at the search deadline and retune's first period honours it; Ch32Dm::halt's rounds and an abstract command's wait stop at the budget's end (pastBudget). RVSWD describes min_clock_hz = 50000: a max_speed under it is refused unsupported with the tag as received, since one attach's minimum checks do not fit the budget there (a clean target took 2.2 s at 2 kHz and 4.4 s at 1 kHz in a host simulation). Checked the same for the other wires: SWIO has one fixed speed (min_clock_hz 888888) and SWD's slowest, 10 kHz, attaches or gives up in about 230 ms. Host test of the whole rvswd stack (attach / detach, a slot-shared connection, the worst cases against the budget).
- (JA) attach の予算（oep-if-debug §1、limits.attach_budget_ms）を、oep.wire.rvswd / swio の attach の答え一つの固い上限にした（reset の hold_ms は別）: 探索の期限を過ぎたとき走っている読みの確認と書きの確認はそこで終わる（最後まで走っていた）。探索の後に残す分は wire_retry_ms + 100 ms（250 では要求の wire の再試行と走っている段を覆わなかった）。attachUnderReset の停止の待ちは探索の期限で止まり、retune の最初の周期もそれを守る。Ch32Dm::halt の回と abstract command の待ちは予算の終わり（pastBudget）で止まる。RVSWD は min_clock_hz = 50000 を describe する: それより下の max_speed は受けた tag を付けて unsupported で拒む。そこでは attach 一つの最小の確認が予算に入らない（host の模擬で、きれいな target が 2 kHz で 2.2 s、1 kHz で 4.4 s）。他の wire も確認: SWIO は速さが一つに決まっている（min_clock_hz 888888）、SWD の最も遅い 10 kHz では約 230 ms で attach するか諦める。rvswd の全体の host test（attach / detach、slot と共有する接続、予算に対する最悪の場合）。
- (EN) Examples on the esp32 profile - FixtureProbe, MinimalProbe, CustomInterface, SwioDebugProbe: ESP-IDF's log is off (esp_log_level_set("*", ESP_LOG_NONE), probe guide §2.5, under ARDUINO_ARCH_ESP32 where the sketch builds for the RP2040 too), as in Firmware/OepProbe - UART0 is their OEP transport. The other examples were checked: OepProbe, MultipleTransports, RvswdDebugProbe and LogicCapture already turn it off; SwdDebugProbe, SwdPinSurvey and ProbeConfig build for the RP2 only.
- (JA) esp32 の profile の例、FixtureProbe、MinimalProbe、CustomInterface、SwioDebugProbe: ESP-IDF のログを止めた（esp_log_level_set("*", ESP_LOG_NONE)、probe ガイド §2.5。RP2040 にも build する sketch では ARDUINO_ARCH_ESP32 の下）。Firmware/OepProbe と同じ。UART0 がそれらの OEP の transport。他の例は確認済み: OepProbe、MultipleTransports、RvswdDebugProbe、LogicCapture はすでに止めている。SwdDebugProbe、SwdPinSurvey、ProbeConfig は RP2 だけ。
- (EN) open (core §6.3 / §6.4): an open sent with role 0x81 (with a session_id in the header) or with session_id 0 is rejected malformed, before the lock is looked at (both were taken).
- (JA) open（core §6.3 / §6.4）: role 0x81（header に session_id 付き）で送った open と、session_id 0 の open は、lock を見る前に malformed で拒む（どちらも受けていた）。
- (EN) The lines while the wire does not answer (oep-if-debug §2, oep-spec 975d88c; §3.2 598bb26; §5 8d91db0), oep.wire.rvswd / swio / swd: on a connection, from an exchange that got no answer (the failures that count toward wire loss, a DMSTATUS of all zeros / ones included) until one answers, the lines rest free between exchanges and are driven only during one (each retry, frame, wake pattern and SWD sequence is one); an answer restores the connection's rest state before the next exchange. RVSWD: released, SWDIO on its pull-up, SWCLK pulled toward its idle_clock level (down for idle_clock low, never up); attach's and retune's speed searches are outside it. SWIO: released to the pull-up and never driven high between frames after a failure; a read cell whose line never came back is no longer driven high against it. SWD: released with SWDIO's pull-up off (its rest level is low) after the exchange's 8 idle cycles. They were driven throughout.
- (JA) wire が答えないときの線（oep-if-debug §2、oep-spec 975d88c。§3.2 598bb26、§5 8d91db0）、oep.wire.rvswd / swio / swd: 接続の上で、答えの無いやり取り（wire 喪失に数える失敗。すべて 0 / 1 の DMSTATUS を含む）から答えるやり取りまで、線はやり取りの間を空きの状態で休み、やり取りの間だけ駆動する（再試行、frame、wake の pattern、SWD の各 sequence がそれぞれ一つ）。答えがあれば次のやり取りの前に接続の休みの状態に戻す。RVSWD: 放し、SWDIO は pull-up、SWCLK は idle_clock の level の向きに pull（idle_clock low なら下向き、上向きにはしない）。attach と retune の速さの探索はその外。SWIO: 失敗の後は frame の間で pull-up に放し、High に駆動しない。線が戻らなかった read の cell では、もうそれに逆らって High を駆動しない。SWD: やり取りの 8 idle cycle の後に放し、SWDIO の pull-up を外す（休みの level は Low）。これまでは駆動したままだった。
- (EN) oep.wire.swd idle cycles (oep-if-debug §5, 8d91db0): a read whose data parity failed now clocks the 8 idle cycles that end every packet (it returned without them), and closing a connection or freeing the pins clocks 8 idle cycles before the lines are let go while they are driven.
- (JA) oep.wire.swd の idle cycle（oep-if-debug §5、8d91db0）: データの parity が違った read も、すべての packet を締める 8 idle cycle を刻む（刻まずに戻っていた）。接続を閉じるときと pin を空けるときは、線を駆動していれば放す前に 8 idle cycle を刻む。
- (EN) oep.wire.rvswd / oep.wire.swio, attach's reset TLV (oep-if-debug §1 / §3, oep-spec 975d88c): a reset channel whose settings idle is an output (mode 3 / 4) is refused unavailable, cause 5, that channel, holder_kind 7, before anything runs (it was pulled low and held for hold_ms). A slot's retry with reset (probe.config §3.1) leaves such an nrst line alone as well. oep.wire.swd has no reset TLV (refused unsupported, as before).
- (JA) oep.wire.rvswd / oep.wire.swio、attach の reset TLV（oep-if-debug §1 / §3、oep-spec 975d88c）: 設定の idle が出力（mode 3 / 4）の reset channel は、何もしないうちに unavailable、cause 5、その channel、holder_kind 7 で拒む（これまでは Low に引いて hold_ms 保っていた）。slot の reset 付きの再試行（probe.config §3.1）もそういう nrst 線には触れない。oep.wire.swd には reset TLV が無い（これまでどおり unsupported）。
- (EN) Wire loss, the good exchange (oep-if-debug §2): a DMI read that comes back all zeros or all ones is no answer on DMSTATUS - the probe answers status line for it - and so no longer stops the wire-loss clock (it did: the read stopped the clock and failure() / checkConnection() then started it again, so a line held low or rising through its pull-up with no module behind it never closed); on any other register such a value leaves the clock as it is. The console's reads see the loss on those reads too. Checked the same mechanism: the riscv-dm DMSTATUS waits (halt's rounds, run, step, attach under reset, resume) took bit 9 of an all-ones read as halted and ackHaveReset acknowledged an all-ones havereset; they now need a module's DMSTATUS (version 2+, not 15). SWD: an ACK of OK / WAIT / FAULT is the good exchange, and a floating or stuck line gives no ACK or a parity error (no change).
- (JA) wire の喪失、良いやり取り（oep-if-debug §2）: すべて 0 かすべて 1 で返る DMI の読みは、DMSTATUS では答え無し（probe はそれに status line を返す）なので、もう wire 喪失の時計を止めない（これまでは止め、failure() / checkConnection() がまた始めていたので、Low に張り付いた線や、module 無しで pull-up で上がる線はいつまでも閉じなかった）。他のレジスタではその値は時計をそのままにする。コンソールの読みもその読みで喪失を見る。同じ仕組みを確認: riscv-dm の DMSTATUS の待ち（halt の回、run、step、reset 中の attach、resume）はすべて 1 の読みの bit 9 を停止と取り、ackHaveReset はすべて 1 の havereset を確認していた。いまは module の DMSTATUS（version 2 以上で 15 でない）を求める。SWD: OK / WAIT / FAULT の ACK が良いやり取りで、浮いた線や張り付いた線は ACK 無しか parity 違いになる（変更なし）。
- (EN) Unavailable refusals carry what they met (core §4.3: cause, channel, holder_fn, holder_kind). plan_apply (and probe.config set's plans) refused because a channel is held now - by a wire's live connection, a resource - answered an empty payload: it now says cause 1, the channel and holder_kind (2 for a connection; the pin table records who claimed a channel); another fn's plan says holder_fn and holder_kind 5 when the settings put that plan in (it said 1). Over plan_roles is cause 2; an interface's own refusal gives its cause (analog capturing 6, the capture roles' count 2, otherwise 6). The rvswd / swio / swd attach and scan of a held pair add the channel and holder_kind (scan had the channel only, attach nothing); the reset line held adds holder_kind; the one seat taken by another pair is cause 2 (a count limit) instead of cause 1; an attach without pins that must name a pair is cause 6 on rvswd / swio, as on swd. CustomInterface's "not planned" is unavailable cause 6.
- (JA) unavailable の拒否に、何にぶつかったかを載せる（core §4.3: cause、channel、holder_fn、holder_kind）。いま channel が握られているために拒んだ plan_apply（と probe.config の set の plan）は、wire の live な接続や資源が握っているとき空の payload を返していた。いまは cause 1、その channel、holder_kind を返す（接続なら 2。pin 表が channel を取った者の種類を覚える）。別の fn の plan なら holder_fn と、その plan を設定が入れたときは holder_kind 5（これまでは 1）。plan_roles 超過は cause 2、interface 自身の拒否はその cause（analog の取り込み中は 6、capture の role の数は 2、それ以外 6）。rvswd / swio / swd の attach と scan で握られた組を拒むときは channel と holder_kind を足した（scan は channel だけ、attach は何も無かった）。reset 線が握られているときは holder_kind を足した。席が別の組で埋まっているときは cause 1 ではなく cause 2（数の上限）。組を名指す必要がある pins 無しの attach は、rvswd / swio でも swd と同じく cause 6。CustomInterface の「plan 無し」は unavailable cause 6。
- (EN) oep.wire.rvswd / oep.wire.swio, host-chosen pins (oep-if-debug §1, core §8.1): an attach whose pins name the pair the link was last on is refused unavailable when a plan has taken one of its channels since (the pin table's owner is checked before the "same pair" shortcut; it was accepted and drove the plan's pin, while a scan of the same pair was refused). A slot's automatic attach (usePair) takes the same check. oep.wire.swd checked: its attach and scan test the owners before moving the link (no change).
- (JA) oep.wire.rvswd / oep.wire.swio、host の選ぶ pin（oep-if-debug §1、core §8.1）: link が最後にいた組を pins に名指す attach は、その後 plan がその channel を取っていれば unavailable で拒む（「同じ組」の近道より前に pin 表の持ち主を見る。これまでは受けて plan の pin を駆動していた。同じ組の scan は拒んでいた）。slot の自動 attach（usePair）も同じ確認をする。oep.wire.swd は確認済み: attach と scan は link を動かす前に持ち主を見る（変更なし）。
- (EN) Registry synced from oep-spec 975d88c (registry hash 41388af90cf14a68): the SPI target's describe cs_setup_ns uses reg::fixture_spi_target::kTlvDescribeCsSetupNs (0x43, oep-spec 73a0c37) in place of the local constant.
- (JA) registry を oep-spec 975d88c に合わせた（registry hash 41388af90cf14a68）。SPI target の describe の cs_setup_ns は、ローカルの定数ではなく reg::fixture_spi_target::kTlvDescribeCsSetupNs（0x43、oep-spec 73a0c37）を使う。
- (EN) OepProbe, classic ESP32: ESP-IDF's log is off (`esp_log_level_set("*", ESP_LOG_NONE)`, probe guide §2.5), as on the ESP32-P4 - UART0 is the OEP transport, and a driver's error log went out on it between frames.
- (JA) OepProbe、classic ESP32: ESP-IDF のログを止めた（`esp_log_level_set("*", ESP_LOG_NONE)`、probe ガイド §2.5）。ESP32-P4 と同じ。UART0 は OEP の transport で、ドライバのエラーログが frame の間にそこへ出ていた。
- (EN) probe.config set (C-02, oep-if-probe-config §1): values a later revision may define are refused unsupported with the item's tag as received, not malformed - a slot's attach policy 2+ and idle_clock 2+, a lock_scheme not in the table (a defined scheme these wires do not read, targetsel, was already unsupported), a bind stream kind other than 1 / 2. The form is checked first and stays malformed: retry_ms on a host slot, a lock whose n is not its scheme's length (scheme 2's length now from the registry), a cut bind element, boot_reset 2+ (malformed as §1.1 states it now). The other item fields were already so (idle mode / drive_kind, bind mode, mechanism, uart format). Host test (tests/host/test_config.cpp; ProbeConfig builds on the host with OEP_HOST_FAKE_CONFIG, the blob in RAM).
- (JA) probe.config の set（C-02、oep-if-probe-config §1）: 後の revision が定めうる値は malformed ではなく、受けた項目の tag を付けて unsupported で拒む: slot の attach の方針 2 以上と idle_clock 2 以上、表に無い lock_scheme（定義済みでこの wire が読まない targetsel はもとから unsupported）、bind の stream の kind 1 / 2 以外。形はその前に確かめ、malformed のまま: host の slot の retry_ms、n が scheme の長さでない lock（scheme 2 の長さも registry から）、切れた bind の要素、boot_reset 2 以上（いまの §1.1 の書き方どおり malformed）。他の項目の欄はもとからそうなっていた（idle の mode / drive_kind、bind の mode、mechanism、uart の format）。host テスト（tests/host/test_config.cpp。ProbeConfig は OEP_HOST_FAKE_CONFIG で host でも組め、blob は RAM に置く）。
- (EN) Fixture SPI target, classic ESP32: the MISO gate's CS handler runs on core 0 (the GPIO ISR service is installed there once per boot from a task pinned to core 0, the handler added through esp_ipc; installing the service from inside the IPC call, as a first cut did, never returned - the install makes an IPC call of its own - and the first configure hung), not on loop()'s core 1, whose interrupts the SWIO wire masks for up to about 45 µs per DMI frame - a CS edge then waited a whole frame. The first look at CS and the removal run on that core inside a critical section, so no handler falls between a look and the output enable it sets; MISO's output enable is cleared before it is taken from the slave (it could drive for a moment as the gate began). The gate only switches the output enable: the pad's value is the slave's MISO signal throughout. describe declares the delay from CS falling to MISO driven as cs_setup_ns (tag 0x43, u32, a local constant until the registry has it): 10000 ns, an estimate from the code (one level-3 GPIO interrupt, about 1.5 µs, plus core 0's own critical sections), to be measured on the bench; a logic capture of the core-0 sampler at the same time is not covered. A level-4 / level-5 handler was not used: level 5 is ESP-IDF's own vector and level 4 the Bluetooth controller's dispatcher, neither near 0.3 µs. Host test.
- (JA) フィクスチャの SPI target、classic ESP32: MISO の門の CS ハンドラは core 0 で動く（GPIO の ISR サービスは core 0 に固定したタスクから boot ごとに一度だけそこに入れ、ハンドラは esp_ipc で足す。最初の版のように IPC の呼び出しの内でサービスを入れると、入れる処理が自分でも IPC を呼ぶので戻らず、最初の configure が止まった）。loop() の core 1 ではない。core 1 の割り込みは SWIO の wire が DMI の frame ごとに最大約 45 µs 止めていて、CS の端がそのあいだ frame 一つ分待たされていた。CS の最初の確認と取り外しはその core で critical section の内に行い、確認とそれが決める出力の許可の間に別のハンドラが入らない。MISO の出力の許可は slave から取り上げる前に下ろす（門を始めるとき一瞬駆動しえた）。門が切り替えるのは出力の許可だけで、パッドの値はずっと slave の MISO の信号。CS が下がってから MISO を駆動するまでの遅れを describe の cs_setup_ns（tag 0x43、u32。registry に入るまではローカルの定数）で宣言する: 10000 ns。コードからの見積もり（level 3 の GPIO 割り込み一つで約 1.5 µs と、core 0 自身の critical section）で、ベンチで測る予定。core 0 の sampler のロジックキャプチャが同時に動く場合は含まない。level 4 / 5 のハンドラは使わなかった: level 5 は ESP-IDF 自身のベクタ、level 4 は Bluetooth コントローラの振り分けで、どちらも 0.3 µs には届かない。host テスト。
- (EN) oep.wire.swd / oep.target.arm-adi (RP2), wire retries as oep-if-debug §2 / §5 require (there were none): a transfer that gets nothing back - no ACK, or a DP read's data parity - is tried again after the line reset (JTAG-to-SWD, then the dormant wake when that brings no answer, TARGETSEL on a multidrop connection, and the DPIDR read a DP needs after a line reset) while one more round still ends inside the request's 200 ms (wire_retry_ms); an AP read with a bad data parity is not repeated (the target took it: TAR moved and the posted value changed). The same for the DPIDR look of an attach joining a live connection. attach tries the wake again the same way, within wire_retry_ms (well inside the attach budget), and answers search_retries (TLV 0x12): the wakes that failed before the one that answered. scan still tries a pair once. WAIT keeps its own 100 retries. swd::transfer reports a bad data parity. Host tests on the simulated SWD target.
- (JA) oep.wire.swd / oep.target.arm-adi（RP2）、oep-if-debug §2 / §5 の求める wire の再試行（これまで無かった）: 何も返らない transfer（ACK 無し、または DP の読みのデータの parity 違い）は、line reset（JTAG-to-SWD、それで答えなければ dormant からの wake、multidrop の接続では TARGETSEL、line reset の後に DP が要る DPIDR の読み）の後にもう一度試す。もう一回りが要求の 200 ms（wire_retry_ms）の内に終わる間だけ。データの parity の違う AP の読みは繰り返さない（target はそれを受けた: TAR が進み、posted の値が変わった）。live な接続に加わる attach の DPIDR の確認も同じ。attach は wake を同じく wire_retry_ms の内（attach の予算の十分内）で試し直し、search_retries（TLV 0x12）を答える: 答えた wake の前に失敗した wake の数。scan は今も組を一度だけ試す。WAIT は自分の 100 回の再試行のまま。swd::transfer はデータの parity 違いを知らせる。host テストは SWD target の模擬で。
- (EN) RVSWD PHY (ESP32-P4, RP2040 / RP2350): the way back after a rest of 300 µs or more counts inside the request's wire retries (oep-if-debug §2, wire_retry_ms 200): after the first DMSTATUS read that does not answer, the re-sync and each of the up to 12 wakes are charged to the allowance the read's own retries use, and no retry starts that would end past it (each is let start only when its cost - the last one measured - still fits). At a 10 kHz max_speed one failed request took 691 ms (twelve wakes of about 37 ms, then 200 ms of read retries); it now takes 210 ms. The SWIO PHY's read retries follow the same rule. Host test.
- (JA) RVSWD の PHY（ESP32-P4、RP2040 / RP2350）: 300 µs 以上休んだ後の立て直しを、要求の wire の再試行（oep-if-debug §2、wire_retry_ms 200）の内に数える: 答えない最初の DMSTATUS の読みの後、再同期と最大 12 回の wake は、読みの再試行と同じ持ち分から差し引き、持ち分を越えて終わる再試行は始めない（直前に測ったその手間がまだ収まるときだけ始める）。max_speed 10 kHz では失敗する要求一つが 691 ms かかっていた（約 37 ms の wake 12 回の後に 200 ms の読みの再試行）。いまは 210 ms。SWIO の PHY の読みの再試行も同じ決まりに従う。host テスト。
- (EN) Idle items and the debug wires (oep-if-debug §1, core §4.3 / §8.1, P2-★1), oep.wire.rvswd / swio / swd: scan with count = 0 leaves out every channel that has an idle item in the settings (any mode; a fixed pair with one is not in the list, a live connection's pair still is), and an attach without pins on a fixed pair with one has no candidate left (unavailable cause 5, the channel, holder_kind 7 settings_idle). A scan listing combinations or the pins of attach naming a channel whose idle is an output (mode 3 / 4) is refused unavailable cause 5, the channel, holder_kind 7; an input idle (mode 0-2) named is accepted. A slot's automatic attach does not drive a pair with an output idle either. Host tests, the SWD ones on a new simulated SWD target (tests/host/shim/fake_swd_io.h, OEP_HOST_FAKE_SWD); SwdPort's `io` gets a default initialiser (no missing-initializer warning).
- (JA) idle の項目と debug wire（oep-if-debug §1、core §4.3 / §8.1、P2-★1）、oep.wire.rvswd / swio / swd: count = 0 の scan は、設定に idle の項目がある channel をすべて除く（mode を問わない。それのある固定の組は列に入らない。live な接続の組は入る）。それのある固定の組で pins 無しの attach は候補が残らない（unavailable cause 5、その channel、holder_kind 7 settings_idle）。組を並べた scan や attach の pins が idle の出力（mode 3 / 4）の channel を名指しすると unavailable cause 5、その channel、holder_kind 7 で拒む。入力の idle（mode 0〜2）なら名指しを受ける。slot の自動の attach も出力の idle のある組は駆動しない。host テスト。SWD のものは新しい SWD target の模擬（tests/host/shim/fake_swd_io.h、OEP_HOST_FAKE_SWD）で。SwdPort の `io` に既定の初期化子を付けた（missing-initializer の警告が出ない）。
- (EN) Wire loss as oep-if-debug §2 states it (limits.wire_lost_ms): a connection is lost only after 1000 ms of real time in which its exchanges got nothing back with no good one between. A riscv-dm request (and an attach joining a live connection) that fails inside its 200 ms of retries answers status line and keeps the connection; it closed after one such request (three DMSTATUS reads). One clock per connection, in the PHY (DmiPhy::read runs it; WireLossClock, src/OepWireLoss.h), shared by the host's requests, the console's reads (which kept a clock of their own) and an at-boot slot's liveness check (which closed after three failed reads; a failed check now keeps the connection until the clock runs out, then the retries start after retry_ms). A reset line's hold (attach's reset TLV, a slot's retry with reset) and riscv-dm reset, with the 1000 ms after them, are not counted. oep.wire.swd / oep.target.arm-adi the same (a transfer list, a block op or an attach joining the connection closed it on its first status line). Host tests.
- (JA) wire の喪失を oep-if-debug §2 の書き方（limits.wire_lost_ms）に合わせた: 接続の失われるのは、やり取りが何も返さず、その間に成功が一度も無いまま実時間で 1000 ms たったときだけ。200 ms の再試行の内で失敗した riscv-dm の要求（と live な接続に加わる attach）は status line を答え、接続は残す（これまではそのような要求一つ（DMSTATUS の 3 回の読み）で閉じていた）。時計は接続ごとに一つで PHY に置き（DmiPhy::read が進める。WireLossClock、src/OepWireLoss.h）、host の要求、コンソールの読み（これまでは自分の時計を持っていた）、at boot の slot の生存確認（これまでは 3 回の読みの失敗で閉じていた。いまは確認が一度失敗しても時計が尽きるまで接続を残し、その後 retry_ms ごとの再試行が始まる）で共有する。reset 線を保つ間（attach の reset TLV、slot の reset 付き再試行）と riscv-dm の reset、およびその後の 1000 ms は数えない。oep.wire.swd / oep.target.arm-adi も同じ（transfer の列、block の op、接続に加わる attach が最初の status line で閉じていた）。host テスト。
- (EN) Refusals as oep-spec fb44490 (C-02) states them - values a later revision may define are unsupported, not malformed: attach method 2+ (rvswd / swio; swd anything but 0) and riscv-dm reset mode 3+ (payload 0x00); read `from` 4+ (console, fixture uart; 0x00); gpio set mode 8+ (as an undeclared mode, with the channel and index); i2c-target configure mode 0 / 4+ (0x00; address over 0x7F stays malformed); uart configure format with an undefined value or reserved bit (the tag as received; a wrong length stays malformed). A gpio drive TLV of an undefined kind (2+) is ignored and listed, unsupported when critical. probe.config: an idle's undefined drive_kind and an idle mode of 5+, and a uart item's undefined format, are unsupported with the item's tag (the mode and the format were malformed before fb44490 too).
- (JA) 拒否を oep-spec fb44490（C-02）の書き方に合わせた。後の revision が定めうる値は malformed ではなく unsupported: attach の method 2 以上（rvswd / swio。swd は 0 以外）と riscv-dm の reset の mode 3 以上（payload 0x00）、read の `from` 4 以上（コンソール、fixture uart。0x00）、gpio の set の mode 8 以上（宣言していない mode と同じく channel と index 付き）、i2c-target の configure の mode 0 / 4 以上（0x00。0x7F を超える address は malformed のまま）、uart の configure の format の未定義の値や予約 bit（受け取ったままの tag。長さ違いは malformed のまま）。未定義の kind（2 以上）の gpio の drive TLV は無視して ignored に載せ、critical なら unsupported。probe.config: idle の未定義の drive_kind と 5 以上の idle の mode、uart の項目の未定義の format は、項目の tag 付きで unsupported（mode と format は fb44490 の前から unsupported の決まりだった）。
- (EN) Console dmseq host rule 1 as target-console-dmseq now states it (DS-5): the count of invalid words restarts when a session starts (start, and the unsynchronising a havereset does) as well as at a valid frame and after the rule-1 answer, and it stops at 3 while not synced (it ran on and wrapped; no answer either way). Checked: the rule-1 answer goes only while synced, a word with bit 7 = 0 leaves the count alone, and there is no 0x7f7f7f7f safeguard (DS-8).
- (JA) コンソールの dmseq の host 規則 1 を、いまの target-console-dmseq の書き方（DS-5）に合わせた: 無効な語の数は、有効な frame と規則 1 の答えの後に加えて、セッションの始まり（start と、havereset で同期を外すとき）にも 0 に戻り、同期していない間は 3 で止まる（数え続けて一周していた。どちらでも答えはしない）。確かめたこと: 規則 1 の答えは同期している間だけ、bit 7 = 0 の語は数を変えない、0x7f7f7f7f の安全策は無い（DS-8）。
- (EN) oep.wire.swd (RP2): the scan budget is the registry's limits.scan_budget_ms (a local 500 before; the same value). The SwdDebugProbe example leaves its fixed pair free at boot (Hi-Z, no pull: the RP2's pads come out of reset pulled down), as after a detach. Checked against debug §5: attach and scan write only the JTAG-to-SWD switch, the dormant wake and TARGETSEL before DPIDR (swd names no scratch, so there is no write check), and SWD has no line retries to bound.
- (JA) oep.wire.swd（RP2）: scan の予算は registry の limits.scan_budget_ms（これまではローカルの 500。値は同じ）。SwdDebugProbe の例は固定の組を boot で空き（Hi-Z、pull 無し: RP2 のパッドはリセット後 pull-down）にする。detach の後と同じ。debug §5 と照らした: attach と scan が DPIDR の前に書くのは JTAG から SWD への切り替え、dormant からの wake、TARGETSEL だけ（swd は scratch を名指さないので書き込みの確認は無い）。SWD には抑えるべき線のやり直しが無い。
- (EN) SWIO PHY, classic ESP32 and ESP32-P4 (oep-if-debug §1 / §3, P2-★8): dmactive is written only when DMCONTROL does not read it set (it was written twice every attach, clearing a haltreq). attach now checks the write path as §1 requires - PROGBUF0 and ABSTRACTAUTO = 0 only, 16 round trips at the wire's one speed with up to 3 retries (SWIO has no parity, so a wrong bit shows only in a comparison), PROGBUF0 read first and written back - where it wrote ABSTRACTAUTO and ABSTRACTCS through Ch32Dm before anything was checked. `bringUp()` is scan's look without the check. Read retries stay within the request's wire_retry_ms. begin() leaves the pin free (no pull-up), as a release does; attach still looks at the line through its pull-up for 2 ms first. Built only; not tried on hardware yet.
- (JA) SWIO の PHY、classic ESP32 と ESP32-P4（oep-if-debug §1 / §3、P2-★8）: dmactive は DMCONTROL が立っていないときだけ書く（attach のたびに 2 回書いて haltreq を消していた）。attach は §1 の求めどおり書き込みの道を確かめる。PROGBUF0 と ABSTRACTAUTO = 0 だけを、線の一つの速さで 16 往復、3 回までやり直して（SWIO には parity が無く、違う bit は比べて初めて分かる）。PROGBUF0 は先に読み、後で書き戻す。これまでは何も確かめないうちに Ch32Dm から ABSTRACTAUTO と ABSTRACTCS を書いていた。`bringUp()` は確認無しの scan の見方。読みのやり直しは要求の wire_retry_ms に収める。begin() はピンを空き（pull-up 無し）にする。解放と同じ。attach は今も、まず 2 ms pull-up で線を見る。ビルドのみ。実機ではまだ試していない。
- (EN) oep.wire.rvswd / oep.wire.swio (oep-if-debug §1 / §2): scan looks at a pair through the PHY's bring-up - the wake / configuration and dmactive only, no write check and no scratch, nothing written as the try ends (it ran a full attach with its write check, and the detach after it wrote ABSTRACTAUTO and DMCONTROL, clearing a haltreq). An attach with method 1 answers the DMSTATUS read after the halt (it answered the one before: 191 of 192 bench attaches said running while halted). The attach answer carries TLV 0x12 search_retries (u16, 0xFFFF and up) when a speed search ran: the failed tries inside the PHY plus each attach tried again. One attach answer keeps to limits.attach_budget_ms (1000 ms, a reset's hold_ms aside): the PHY's search stops 250 ms short of it and no attach is tried again past it (a probe's own attaches - a slot's, the console's re-attach - too). No scan pair starts later than limits.scan_budget_ms (500 ms) after the request, and one try is bounded by the attach budget; the budget comes from the registry (it was a local 500). Every request and every console read starts with the full wire_retry_ms (200 ms) of retries. Host tests in `test_wire.cpp`.
- (JA) oep.wire.rvswd / oep.wire.swio（oep-if-debug §1 / §2）: scan は組を PHY の立ち上げで見る。wake / 構成と dmactive だけで、書き込みの確認も scratch も無く、試し終わりにも何も書かない（これまでは書き込みの確認込みの attach をして、その後の detach が ABSTRACTAUTO と DMCONTROL を書き、haltreq を消していた）。method 1 の attach は止めた後に読んだ DMSTATUS を返す（止める前のものを返していた: ベンチの 192 回中 191 回、止まっているのに running と答えた）。速さを探したとき、attach の応答に TLV 0x12 search_retries（u16、0xFFFF 以上は 0xFFFF）を付ける: PHY の中で失敗した試しと、attach をやり直した回数。一つの attach の応答は limits.attach_budget_ms（1000 ms、reset の hold_ms は別）に収める: PHY の探索はその 250 ms 手前で止まり、それを過ぎて attach をやり直さない（probe 自身の attach、スロットのものやコンソールのつなぎ直しも）。scan の組は要求から limits.scan_budget_ms（500 ms）を過ぎて始めない。一つの試しは attach の予算で抑える。予算は registry の値（ローカルの 500 だった）。要求とコンソールの読みはどれも wire_retry_ms（200 ms）のやり直しを丸ごと持って始まる。ホストテストは `test_wire.cpp`。
- (EN) RVSWD PHY attach (ESP32-P4 and RP2040 / RP2350, oep-if-debug §1 / §3, P2-★8): before the speed is verified only the wake, the configuration pair and dmactive go out, all at the slowest period - the re-sync in front of a faster period's checks now sends its configuration pair at the slowest period and then switches (it went out at the unverified period). The write check writes only PROGBUF0 and ABSTRACTAUTO = 0: PROGBUF0 is read first and written back after (it was left 0), at the checked period, or at the slowest one when the check failed. The read check ignores DMSTATUS bits 8-19 (the harts' state: a hart stopping or a self-reset made a good period fail). When only the slowest period is left (max_speed 1 MHz, or every faster one failed) its read check is not run a second time (79 ms less: the code's attach at 1 MHz is about 121 ms of wire time, 200 before), and its checks take up to 3 retried reads or round trips (an isolated parity or turnaround error failed the attach: 4 % on the bench, L103 behind an RP2350); faster periods take none. A check is also cut to 100 / 60 ms at a slow max_speed (at least 64 reads, 2 rounds), so the attach fits its budget. attach() stops at the attach budget's deadline, counts search_retries, and `bringUp()` is a scan's look: the wake / configuration and dmactive (only when DMCONTROL does not read it set; probeOnce too), DMSTATUS, no write check. A failed read is retried only while the request's wire_retry_ms (200 ms) lasts. begin() and usePins() leave the pins free (no pull-up on SWDIO, not driven), as a release does; an attach puts the pull-up back. Host test on a simulated target (`test_rvswd_phy.cpp`); not tried on hardware yet.
- (JA) RVSWD の PHY の attach（ESP32-P4 と RP2040 / RP2350、oep-if-debug §1 / §3、P2-★8）: 速さを確かめるまでは、wake、構成の組、dmactive だけを、すべて最も遅い周期で送る。速い周期を確かめる前の再同期は、構成の組を最も遅い周期で送ってから切り替える（確かめていない周期で送っていた）。書き込みの確認は PROGBUF0 と ABSTRACTAUTO = 0 だけを書く: PROGBUF0 を先に読み、後で書き戻す（0 のままにしていた）。書き戻しは確かめた周期で、確認が失敗したときは最も遅い周期で。読みの確認は DMSTATUS の bit 8-19（hart の状態: hart が止まる途中や自分でのリセットで、よい周期が落ちていた）を比べない。最も遅い周期しか残っていないとき（max_speed 1 MHz、または速い周期がすべて失敗）、その読みの確認を二度は行わず（79 ms 減る: コードから見た 1 MHz の attach は線の時間で約 121 ms、前は 200）、確認は読みか往復を 3 回までやり直す（単発の parity や turnaround の誤りで attach が失敗していた: ベンチで 4 %、RP2350 の先の L103）。速い周期はやり直さない。遅い max_speed では確認を 100 / 60 ms で打ち切る（読み 64 回、2 巡は必ず）ので、attach が予算に収まる。attach() は attach の予算の期限で止まり、search_retries を数える。`bringUp()` は scan の見方: wake / 構成と dmactive（DMCONTROL が立っていないときだけ。probeOnce も）、DMSTATUS で、書き込みの確認はしない。失敗した読みは、要求の wire_retry_ms（200 ms）が残っている間だけやり直す。begin() と usePins() はピンを空き（SWDIO に pull-up 無し、駆動しない）にする。解放と同じ。attach が pull-up を戻す。模擬の target でのホストテスト（`test_rvswd_phy.cpp`）。実機ではまだ試していない。
- (EN) dmVersionKnown follows oep-if-debug §1 (P2-○1): a module is found when DMSTATUS.version is 2 or more and not 15 (it took 2 and 3 only); the riscv-dm ops treat them all alike. RvswdPhy::probeOnce and the read check's reference test the version bits 3:0 (they tested bits 8-11, the harts' state).
- (JA) dmVersionKnown を oep-if-debug §1（P2-○1）に合わせた: DMSTATUS.version が 2 以上で 15 でなければ見つかった（2 と 3 だけだった）。riscv-dm の op はどれも同じに扱う。RvswdPhy::probeOnce と読みの確認の基準は version の bit 3:0 を見る（hart の状態の bit 8-11 を見ていた）。
- (EN) `src/OepRegistry.h` synced from oep-spec 0bb10e8, then 536fc99 (probe_config line names): role_assignment is 0x10 (sent critical as 0x90; the plan refusal still carries 0x90), the target_id schemes and lengths are common (`reg::common::kTargetIdScheme*`), max_op_ms's reference value is `kReferenceMaxOpMs` ([reference]), and the new limits (attach / scan budgets, wire_retry_ms, wire_lost_ms), search_retries 0x12, attach_writes_unbounded and holder_kind 7 are there.
- (JA) `src/OepRegistry.h` を oep-spec 0bb10e8、続いて 536fc99（probe_config の線の名前）から写した: role_assignment は 0x10（critical を付けて 0x90 で送る。plan の拒否は今も 0x90 を付ける）、target_id の scheme と長さは common（`reg::common::kTargetIdScheme*`）、max_op_ms の参照値は `kReferenceMaxOpMs`（[reference]）。新しい limits（attach / scan の予算、wire_retry_ms、wire_lost_ms）、search_retries 0x12、attach_writes_unbounded、holder_kind 7 が入った。
- (EN) unit_id is always in describe (core §7.5): on a platform the library has no chip number for, the build stops unless the sketch defines OEP_UNIT_ID (1 to 16 of a-z 0-9 -, the build constant core §7.5 allows); describeCore fails rather than leave it out (it was silently omitted). ESP32 (MAC) and RP2 (flash id) are unchanged.
- (JA) unit_id は describe に必ず入る（core §7.5）: ライブラリがチップの番号を知らない platform では、スケッチが OEP_UNIT_ID（a-z 0-9 - で 1〜16 文字、core §7.5 が許すビルド定数）を定義しないとビルドが止まる。describeCore は省くかわりに失敗する（これまでは黙って省いていた）。ESP32（MAC）と RP2（flash の id）は変わらない。
- (EN) riscv-dm: a version-3 (debug spec 1.0) debug module is worked with like version 2 (oep-if-debug §1: found = DMSTATUS.version 2 or 3). The ops, the console and the RVSWD link revive required version 2: a 1.0 module was found by scan but never treated as halted.
- (JA) riscv-dm: version 3（debug spec 1.0）の debug module も version 2 と同じに扱う（oep-if-debug §1: 見つかった = DMSTATUS.version 2 か 3）。op、コンソール、RVSWD のリンクの立て直しが version 2 を要求していて、1.0 の module は scan で見つかっても停止中と扱われなかった。
- (EN) Debug wires (RVSWD, SWIO, SWD): pins no connection holds go to their free state - Hi-Z with no pull, or the idle the settings give them (core §8, debug §1): after a connection closes (with idle_clock low SWCLK stayed driven low; a fixed pair kept the PHY's state and SWDIO its pull-up), after each scan try (the tried pins stayed Hi-Z, not their idle), after a failed attach of a host or a slot. New DmiPhy::free; attach puts the PHY's pull-up back.
- (JA) debug wire（RVSWD、SWIO、SWD）: 接続が持っていないピンは空きの状態（pull 無しの Hi-Z か、設定の idle）に戻る（core §8、debug §1）: 接続が閉じた後（idle_clock low では SWCLK を Low に駆動したままだった。固定の組は PHY の状態のままで、SWDIO は pull-up のままだった）、scan で試すたび（試したピンが idle でなく Hi-Z のままだった）、host やスロットの attach が失敗した後。DmiPhy::free を追加し、attach が PHY の pull-up を戻す。
- (EN) Logic capture (ESP32-P4 parallel IO, classic sampler): taking the plan changes no pin (core §8, capture §1.2: a logic capture only listens). It set the pins to input, which stopped an output idle or another fn's output, and never restored them. The classic only switches the pad's input buffer on, at the take and at each start.
- (JA) ロジックキャプチャ（ESP32-P4 の parallel IO、classic の sampler）: plan を取ってもピンを変えない（core §8、capture §1.2: ロジックキャプチャは聞くだけ）。これまではピンを入力にして、出力の idle や他の fn の出力を止め、戻さなかった。classic は pad の入力バッファを入れるだけ（取ったときと、start のたび）。
- (EN) Fixture SPI target, classic ESP32: MISO is undriven while CS is high (the slave drove it low from configure to release, fighting any other device on the line; the ESP32-P4 never did). The pad's output enable is taken from the slave and follows CS through a CS edge interrupt (IRAM, level 3). Built only: the bench re-runs its 1 MHz test on this build.
- (JA) フィクスチャの SPI target、classic ESP32: CS が High の間は MISO を駆動しない（slave が configure から release まで Low に駆動していて、同じ線の他のデバイスとぶつかっていた。ESP32-P4 は元から駆動しない）。pad の出力の許可を slave から取り、CS のエッジ割り込み（IRAM、level 3）で CS に従わせる。ビルドのみ: ベンチがこのビルドで 1 MHz の試験をやり直す。
- (EN) Reject reasons and payload tags (core §4.3, §8): plan_apply of a channel outside role_channels, or of a role the interface lacks, is unsupported (spi-target and i2c-target said unavailable, logic / analog capture too for a role or an ADC-less pin) and carries tag 0x90 (the endpoint sent no payload). probe.config set: what an item cannot take (idle, disable, slot, bind, uart) carries the item's tag as received, not 0x00; a plan the interface refuses carries the plan item's tag. attach (rvswd / swio / swd): pins and reset carry the tag as received (pins was always 0x83); sent without the critical bit, a value the probe cannot take is ignored and listed (core §2.3).
- (JA) 断る理由と payload の tag（core §4.3、§8）: role_channels に無い channel や、インターフェースに無い役の plan_apply は unsupported（spi-target と i2c-target は unavailable だった。ロジック / アナログのキャプチャも役や ADC の無いピンで同じ）、tag 0x90 を付ける（endpoint は payload を付けていなかった）。probe.config の set: 項目が受け付けられないもの（idle、disable、slot、bind、uart）は 0x00 でなく受け取ったままの項目の tag を付ける。インターフェースが断った plan は plan の項目の tag。attach（rvswd / swio / swd）: pins と reset は受け取ったままの tag（pins はいつも 0x83 だった）。critical のビット無しで送られた、扱えない値は無視して ignored に載せる（core §2.3）。
- (EN) ESP32-P4: oep.wire.swio gets its own oep.target.console (instance 1, the last fn) and is the second place a slot may name (probe.config §1.1): a CH32V00x on the one wire can be an at-boot slot with a console, the retry with reset included (it was refused unsupported: only the RVSWD wire was a place). Static RAM +9 KB (76 %).
- (JA) ESP32-P4: oep.wire.swio に自分の oep.target.console（instance 1、最後の fn）を付け、スロットが名指せる 2 つ目の場所にした（probe.config §1.1）: 1 本線の CH32V00x を、コンソール付きの起動時のスロットにできる（リセットでのやり直しも）。これまでは RVSWD の線だけが場所で、unsupported だった。静的 RAM +9 KB（76 %）。
- (EN) probe-config §2 (oep-spec 3cc50c8): unset of an item tag this probe does not declare (idle / disable without pins too) is rejected unsupported with the tag as received (was 0x00). The unset row's tag is taken as it is (no critical bit there): 0x81 is not the plan item, it is an undeclared tag.
- (JA) probe-config §2（oep-spec 3cc50c8）: この probe が宣言していない項目の tag の unset（ピンの無い probe の idle / disable も）は、受け取ったままの tag で rejected unsupported（0x00 だった）。unset の要素の tag はそのまま扱う（そこに critical のビットは無い）: 0x81 は plan の項目ではなく、宣言していない tag。
- (EN) Request tails and refusals as oep-spec 9ed53e7 states them (core §2.3 / §7.3 / §4.3): ignored (0x7F) is on every completed answer, a failed status too - the list lives for the request and the endpoint appends it to an answer whose handler returned without it (before, an early `failed()` - a target's status line, a capture or I2C / SPI target refusal, probe.config save / erase failing - dropped it); a tag is listed once per TLV ignored (was once per tag). describe and probe.config get take no request TLV: one there is rejected malformed (was ignored). A probe.config set of an item this probe does not declare is rejected unsupported with the item's tag as received (was 0x00).
- (JA) 要求の tail と拒否を oep-spec 9ed53e7 の書き方どおりに（core §2.3 / §7.3 / §4.3）: ignored（0x7F）は、status が failed のものも含めて、completed の応答すべてに付ける。一覧は要求ごとに持ち、handler が付けずに返した応答には endpoint が付ける（前は、早めの `failed()`、たとえば target の status line、capture や I2C / SPI target の拒否、probe.config の save / erase の失敗で落ちていた）。tag は無視した TLV ごとに一つ並べる（前は tag ごとに一つ）。describe と probe.config の get は要求に TLV を取らない。あれば rejected malformed（前は ignored）。probe.config の set で、この probe が宣言していない item は、受け取ったままの item の tag を付けて rejected unsupported（前は 0x00）。
- (EN) `oep.probe.config` slot `boot_reset` (oep-spec ee562c3, probe-config §1.1 / §1.3 / §3.1 / §3.3): an at-boot slot with boot_reset 1 whose automatic attach got no answer from the wire (status line) attaches once more with its reset line pulled for `slot_retry_reset_hold_ms` (20 ms) first, as attach's reset TLV with method 0 - only while no session has taken the lock since boot, at most once a boot per slot, and only when the slot's `nrst` line (the label `slot.nrst`, or `nrst` with at most one slot) is a reset channel of its wire that nothing holds. slot_state ends with reset_at_ns (all ones when not done). The retry's reset is not a host reset: a last-reset bind keeps its selection. boot_reset 2+, or 1 on a host slot, is malformed. `findLine` / `ProbeConfig::lineOf` give the §1.3 line names to sketches. Not tried on hardware yet.
- (JA) `oep.probe.config` の slot の `boot_reset`（oep-spec ee562c3、probe-config §1.1 / §1.3 / §3.1 / §3.3）: boot_reset 1 の at-boot の slot で、自動 attach に線から応答がなかったとき（status line）、reset の線を `slot_retry_reset_hold_ms`（20 ms）引いてから、もう一度だけ attach する。attach の reset TLV（method 0）と同じ。boot から一度もセッションが lock を取っていない間だけ、slot ごとに boot ごと一回まで、slot の `nrst` の線（label `slot.nrst`、slot が一つ以下なら `nrst`）がその線の reset の channel で、だれも持っていないときだけ。slot_state の最後に reset_at_ns（していなければ全ビット 1）。この reset は host の reset ではないので、last-reset の bind の選択は変わらない。boot_reset が 2 以上、または host の slot で 1 は malformed。`findLine` / `ProbeConfig::lineOf` で §1.3 の線の名前を sketch からも引ける。実機ではまだ試していない。
- (EN) `oep.fixture.gpio` output drive strength (oep-spec ee562c3 / dc71b1f, fixture §1.1): describe drive_levels on the classic ESP32 and the ESP32-P4 (`GPIO_DRIVE_CAP_0..3`, about 5 / 10 / 20 / 40 mA, default 2 = the IDF default) and on the RP2040 / RP2350 (2 / 4 / 8 / 12 mA, default 4 mA); other chips declare none. set's drive TLV per mode 3 / 4 element (a level, or an mA ceiling), malformed for a bad index / kind / element, ignored (0x01 listed once per TLV) for a level the probe does not have, unsupported when that one is critical (every form checked first: a malformed drive anywhere in the request wins); without drive_levels every drive TLV is an unknown tag. The strength is the set's drive, else the channel's idle drive, else the default, kept until the next set; a channel taken by a plan, or released, is at its idle state's. read answers TLV 0x01 drive (the level per channel, 0xFF when not driven in mode 3 / 4). The probe.config idle item takes drive_kind / drive_value (mode 3 / 4 only; length 4 / 5, another mode or an undefined kind malformed; a level out of range unsupported), applied with the level at boot and at every release. A pad leaving mode 3 / 4 goes back to the default strength (so does a pin a UART or an I2C / SPI target takes straight from an output idle), and the RVSWD PHY sets its weakest drive again at every attach (a host-chosen pair may have been a strong gpio output in between). Not tried on hardware yet.
- (JA) `oep.fixture.gpio` の出力の強さ（oep-spec ee562c3 / dc71b1f、fixture §1.1）: describe の drive_levels を、classic ESP32 と ESP32-P4（`GPIO_DRIVE_CAP_0..3`、約 5 / 10 / 20 / 40 mA、既定 2 = IDF の既定）と RP2040 / RP2350（2 / 4 / 8 / 12 mA、既定 4 mA）で出す。ほかのチップは出さない。set の drive TLV は mode 3 / 4 の要素ごと（段階の番号か mA の上限）。index / kind / 要素がおかしいと malformed、ない段階はその TLV を無視（ignored に TLV ごとに 0x01）、それが critical なら unsupported（形はすべて先に確かめるので、要求のどこかに malformed の drive があればそちらが勝つ）。drive_levels がなければ drive TLV はどれも知らない tag。強さは set の drive、なければその channel の idle の drive、なければ既定で、次の set まで保つ。plan が取った channel と解放した channel は idle の状態の強さ。read は TLV 0x01 drive（channel ごとの段階、mode 3 / 4 で駆動していなければ 0xFF）を返す。probe.config の idle の item は drive_kind / drive_value を取り（mode 3 / 4 だけ。長さ 4 / 5、ほかの mode、未定義の kind は malformed。範囲外の段階は unsupported）、boot と解放のたびにレベルと一緒にかける。mode 3 / 4 を離れたパッドは既定の強さに戻し（UART や I2C / SPI target が出力の idle から直接取るピンも）、RVSWD の PHY は attach のたびに最弱の強さをかけ直す（host が選ぶ組が間に強い gpio の出力だったことがありうる）。実機ではまだ試していない。
- (EN) RP2040 / RP2350: an output (fixture gpio mode 3 / 4 and open-drain low, an output idle, a fixture UART's TX parked high) puts its level into the pad's latch with the SDK's `gpio_put` before the output goes on. arduino-pico's `digitalWrite` on a pin last set INPUT_PULLUP / INPUT_PULLDOWN only switches the output on or off and leaves the latch at the 0 / 1 those modes wrote, so a pull-up pin set to output high came up driven low (and a UART TX parked from its pull-up dipped low first). Found by reading the core; not tried on hardware yet.
- (JA) RP2040 / RP2350: 出力（fixture gpio の mode 3 / 4 と open-drain low、出力の idle、fixture UART の TX を high にしておくところ）は、出力を有効にする前に SDK の `gpio_put` でパッドのラッチにレベルを入れる。arduino-pico の `digitalWrite` は、最後に INPUT_PULLUP / INPUT_PULLDOWN にしたピンでは出力の有効 / 無効を切り替えるだけで、ラッチはそのモードが書いた 0 / 1 のまま。そのため pull-up のピンを output high にすると low で駆動していた（pull-up から high にしておく UART の TX も、一度 low に落ちていた）。core を読んで見つけた。実機ではまだ試していない。
- (EN) ESP32-P4 `oep.wire.swio` (added today): the SWIO pin also runs at the weakest drive strength, as the classic ESP32's SWIO and every RVSWD PHY already did (P4 CAP_0, RP2 2 mA since 2026-09-22); the port had left it at the default. Checked on the P4 + CH32V003: attach 6/6, read_block 64 words 7.4-8.1 ms (8.1 before), power-cycle attach at dpc 0, `oep pins` finds SWIO and NRST.
- (JA) ESP32-P4 の `oep.wire.swio`（今日足した）: SWIO のピンも最弱の出力にする。classic ESP32 の SWIO と、すべての RVSWD の PHY（P4 は CAP_0、RP2 は 2 mA。2026-09-22 から）と同じ。移植で既定のまま残していた。P4 + CH32V003 で確認: attach 6/6、read_block 64 語 7.4〜8.1 ms（前は 8.1）、電源の入れ直しからの attach は dpc 0、`oep pins` が SWIO と NRST を見つける。
- (EN) Classic ESP32 `oep.wire.swio`: the SWIO pin runs at the weakest drive strength (about 5 mA, `OEP_SWIO_DRIVE_CAP` to change). At the default 20 mA its edges coupled into the fixture lines next to it: on the V003 jig a 1 MHz `oep.fixture.spi-target` lost or shifted bits whenever the console was read over SWIO (23 of 36 64-byte frames good). At the weakest drive 72 of 72, and the wire's own speed is unchanged (read_block 64 words 36 ms). Found with the bench; no wiring was changed.
- (JA) classic ESP32 の `oep.wire.swio`: SWIO のピンを最弱の出力（約 5 mA、`OEP_SWIO_DRIVE_CAP` で変えられる）で動かす。既定の 20 mA では、エッジが隣の fixture の線に乗り、V003 の治具で、SWIO でコンソールを読んでいる間、1 MHz の `oep.fixture.spi-target` が bit を落としたりずらしたりした（64 byte の 36 frame 中 23 だけ正しい）。最弱では 72 中 72 で、線そのものの速さは変わらない（read_block 64 語 36 ms）。bench と一緒に見つけた。配線は変えていない。
- (EN) Breaking: USB identification as oep-spec 2108125 (core §3.3) defines it. `src/OepRegistry.h` synced: `kUsbIproductPrefix` is gone (registry `usb.iproduct_prefix` removed; the library never used it). Hosts identify probes automatically only by the project's own VID:PID (none listed yet), open a probe named by its unit_id by the USB serial number, and treat anything else with the confirm-only probing rule. The reference firmware keeps its iProduct `OEP probe (...)`, now a temporary clue for hosts (oep-spec host guide §1.7), not an identification. describe discoverable (0x4A) is 1 only on the project's VID:PID: `Firmware/OepProbe` (RP2 and ESP32-P4) and `MultipleTransports` no longer call `setDiscoverable(true)`, so fn 0's describe carries no discoverable (0). READMEs, guides and sketch comments aligned.
- (JA) 破壊的変更: USB での見分け方を oep-spec 2108125（core §3.3）の定めに揃えた。`src/OepRegistry.h` を写し直し、`kUsbIproductPrefix` が消えた（registry の `usb.iproduct_prefix` は削除。ライブラリは使っていなかった）。host が自動で probe と見分けるのはプロジェクトの VID:PID だけ（まだ無い）で、unit_id で名指した probe は USB の serial number で開き、それ以外は confirm だけで探る規則で扱う。参照の firmware の iProduct `OEP probe (...)` はそのままで、見分けではなく host の暫定の手がかり（oep-spec host 開発ガイド §1.7）になった。describe の discoverable（0x4A）はプロジェクトの VID:PID のときだけ 1: `Firmware/OepProbe`（RP2 と ESP32-P4）と `MultipleTransports` は `setDiscoverable(true)` を呼ばなくなり、fn 0 の describe に discoverable は出ない（0）。README、ガイド、sketch のコメントを揃えた。
- (EN) The project's USB VID:PID is not settled: `src/OepRegistry.h` synced from oep-spec 2bbd7d1 drops `kUsbReferenceVid` / `kUsbReferencePid` (registry `usb.reference_vid` / `reference_pid` are gone). The firmware keeps the board's default VID:PID (303a:0002 on the ESP32-P4), a temporary USB ID that may not be used for distribution; the READMEs, the guides and the sketch comments no longer name a planned number. Hosts find probes by iProduct `OEP` and describe.
- (JA) プロジェクトの USB の VID:PID は決まっていない: oep-spec 2bbd7d1 から写した `src/OepRegistry.h` から `kUsbReferenceVid` / `kUsbReferencePid` が消えた（registry の `usb.reference_vid` / `reference_pid` は削除）。firmware はボードの既定の VID:PID（ESP32-P4 では 303a:0002）のまま。これは仮の USB の ID で、配布には使えない。README、ガイド、sketch のコメントは予定の番号を書かない。host は iProduct の `OEP` と describe で probe を見つける。
- (EN) Fix: fixture SPI target (classic ESP32, ESP32-P4) loads the next transaction in the spi_slave driver's interrupt (`post_trans_cb`, `SPI_SLAVE_NO_RETURN_RESULT`), at the CS rising edge that ended the last one, instead of from `service()` in loop(). In 0.0.28 a CS frame with no clock left the arm to be queued again by the next `service()`, which could come inside the following frame: on the classic ESP32 a load resets the slave and rewrites its buffer, so the frame restarted there (bench, V003 jig, 1 MHz, 64 bytes: the DUT's MISO wrong from byte 2 or 28, read_rx holding only bits from there on, the first of three 64-byte transfers). The discard after a transfer is loaded the same way, so an unarmed frame right after an armed one is counted with MISO 0 (it could go unseen, sending the last frame's MOSI). arm no longer restarts the driver: it takes the discard back with `spi_slave_queue_reset` and loads the armed transaction in its place.
- (JA) 修正: fixture SPI target（classic ESP32、ESP32-P4）は次の転送を、loop() の `service()` からではなく spi_slave ドライバの割り込み（`post_trans_cb`、`SPI_SLAVE_NO_RETURN_RESULT`）で、前の転送を終えた CS の立ち上がりで積む。0.0.28 では clock の無い CS の frame の後、arm を次の `service()` が積み直し、それが次の frame の途中に来ることがあった: classic ESP32 では積むと slave がリセットされバッファが書き直されるので、frame がそこからやり直しになった（ベンチ、V003 のジグ、1 MHz、64 バイト: DUT の MISO が 2 か 28 バイト目から違い、read_rx はそこからの bit だけ、3 回の 64 バイト転送の 1 回目）。転送の後の捨てる用の転送も同じく積むので、arm した転送のすぐ後の arm していない frame も MISO 0 で数える（見えないまま、前の frame の MOSI を送ることがあった）。arm はドライバを作り直さなくなった: 捨てる用の転送を `spi_slave_queue_reset` で取り戻し、その場所に arm した転送を積む。

## 0.0.28
- (EN) Fix: replacing a plan (plan_apply with a new channel set for the same fn, or a settings change of that plan) no longer puts every old channel to Hi-Z before claiming the new set. In 0.0.27 a power line kept in both gpio plans glitched off (ESP32-P4: the target lost power and stopped answering until power-cycled). `Endpoint::replaceFns` now defers the idle states (`PinTable::deferIdle` / `settleIdle`; `ProbeConfig::setPins` hands the table to the endpoint): a channel in both plans keeps its state and drive, a channel leaving goes to its idle state at the end, a new channel stays in its idle state until the first set (oep-core §8, fixture §1). The same for every fixture that releases through the pin table (uart, i2c / spi targets, analog) and for a refused replacement's undo. Host test in `test_idle.cpp`.
- (JA) 修正: plan の置き換え（同じ fn への新しい channel の組での plan_apply、設定でのその plan の変更）で、新しい組を取る前に古い channel を全部 Hi-Z にしなくなった。0.0.27 では、両方の gpio の plan にある電源の線が一瞬切れた（ESP32-P4: target の電源が落ち、電源を入れ直すまで応答しなくなった）。`Endpoint::replaceFns` は空きの状態を後回しにする（`PinTable::deferIdle` / `settleIdle`。`ProbeConfig::setPins` が表を endpoint にも渡す）: 両方の plan にある channel は状態と駆動を保ち、外れる channel は最後に空きの状態に戻り、新しい channel は最初の set まで空きの状態のまま（core §8、fixture §1）。pin の表で解放するほかの fixture（uart、i2c / spi target、analog）と、断られた置き換えの戻しも同じ。host テストは `test_idle.cpp`。
- (EN) Idle states as oep-spec 5013ffb / 7b3c319 define them (registry synced: idle_mode `output_low` 3, `output_high` 4). `oep.probe.config` idle takes modes 3 output low and 4 output high: a free channel with one is driven at that level (the level is put in the output register before the output is turned on, so a power switch sees no pulse); on a channel the probe cannot drive (the classic ESP32's GPIO34-39, `PinTable::setInputOnly`) it is rejected unsupported. Every release goes to the idle state: plan_release, a replaced plan, the lease lapse and force as before, and now also a debug wire's connection going away (only channels with an idle set are touched, the others keep the PHY's Hi-Z) and attach's reset line after the pulse. Boot order (probe.config §2): a settings change and the saved settings now apply the idle states before the plans (they used to come after the plans), then the uarts, then the at-boot slot's attach - an output idle is driving before a gpio plan takes its channel and before the target is attached. A channel taken by an `oep.fixture.gpio` plan keeps its idle state (an output idle keeps driving) until the first set (fixture §1; planApply never touched the pad, now tested). Host test `test_idle.cpp`.
- (JA) 空きの状態を oep-spec 5013ffb / 7b3c319 の定めに揃えた（registry を写し直した: idle_mode の `output_low` 3、`output_high` 4）。`oep.probe.config` の idle が mode 3 出力 low と 4 出力 high を受ける: それを置いた空きの channel はその level で駆動する（出力を有効にする前に出力レジスタへ level を入れるので、電源のスイッチにパルスが出ない）。probe が出力にできない channel（classic ESP32 の GPIO34-39、`PinTable::setInputOnly`）では unsupported で断る。解放はどの道でも空きの状態に戻る: plan_release、置き換えられた plan、lease の失効と force はこれまでどおりで、debug の wire の接続が消えたとき（idle を置いた channel だけ触る。ほかは PHY の Hi-Z のまま）と attach の reset 線のパルスの後も加えた。起動の順（probe.config §2）: 設定の変更と保存した設定の適用で、idle を plan より先に掛けるようにした（これまでは plan の後だった）。その後 uart、最後に at boot のスロットの attach。出力の idle は、gpio の plan が channel を取る前にも、target に attach する前にも駆動している。`oep.fixture.gpio` の plan が取った channel は、最初の set まで空きの状態を保つ（出力の idle は駆動し続ける。fixture §1。planApply はもともとパッドに触らない。テストを加えた）。host テスト `test_idle.cpp`。
- (EN) ESP32-P4 (`Firmware/OepProbe` Esp32P4.h): `oep.wire.swio`, the one-wire link of WCH CH32V00x, as a wire of its own next to `oep.wire.rvswd` (added after the capture group, so the fns before it keep their numbers; saved settings from before do not match the new interface list). Its pin and the reset line may be any channel but the USB-Serial/JTAG pair (24, 25). Its connections are served by the one `oep.target.riscv-dm`: `TargetRiscvDm::addPort` gives it a second wire's DebugPort and a request goes to the wire whose live connection it names (connection numbers are the probe's one space). The console stays the RVSWD wire's only. `SwioPhy` on the P4: the classic ESP32's frames and read recharge on the CPU's dedicated GPIO (one out channel with its OEN bit as the pad's output enable, one in channel; a GPIO register access takes about 260 ns there, as long as a short pulse), timed in nanoseconds against the cycle counter with each edge at a deadline from the one before: 262 / 862 ns low, 262 ns high, as the classic's coefficient 8; measured on the pad with a core-0 sampler at 250-280 / 840-890 / 250-280 ns. Any GPIO0-54; the pin is in the bundle only while attached. `RvswdPhy` on the P4 takes its pins back into its bundles at attach when something routed them away (swio on one of the RVSWD pair's pins). The classic ESP32 path is unchanged. Not yet attached to a target: the bench CH32V003 answered nothing on either candidate pin (2026-10-02).
- (JA) ESP32-P4（`Firmware/OepProbe` の Esp32P4.h）: WCH CH32V00x の単線の `oep.wire.swio` を、`oep.wire.rvswd` と並ぶ独立の wire として加えた（capture group の後に加えたので、それより前の fn の番号は変わらない。以前の保存した設定は新しいインターフェースの並びと合わない）。そのピンとリセットの線は USB-Serial/JTAG の対（24、25）以外のどの channel でもよい。その接続は 1 つの `oep.target.riscv-dm` が受け持つ: `TargetRiscvDm::addPort` が 2 本目の wire の DebugPort を渡し、要求はそれが名指す生きた接続の wire へ行く（接続番号は probe で 1 つの空間）。console は RVSWD の wire のものだけのまま。P4 の `SwioPhy`: classic ESP32 のフレームと読み出しの recharge を CPU の dedicated GPIO の上で行う（出力 1 チャネルとその OEN ビットをパッドの出力有効に、入力 1 チャネル。P4 の GPIO レジスタのアクセスは約 260 ns かかり、短いパルス 1 本と同じ長さ）。時間はサイクルカウンタに対する ns で、どの edge も前の edge からの期限で打つ: low 262 / 862 ns、high 262 ns（classic の係数 8 と同じ）。core 0 のサンプラでパッドを測って 250〜280 / 840〜890 / 250〜280 ns。GPIO0〜54 のどれでもよく、ピンは attach している間だけ bundle に入る。P4 の `RvswdPhy` は attach のとき、ほかが自分のピンを引き抜いていれば（swio が RVSWD の対のピンを使った）bundle に戻す。classic ESP32 の経路は変わらない。まだ target に attach できていない: ベンチの CH32V003 はどちらの候補のピンでも何も答えなかった（2026-10-02）。
- (EN) Breaking: fixture I2C target follows fixture §3 as oep-spec 75ca9ee defines it (registry synced: describe `max_stretch_us` 0x41). The received bytes are now this target's own: the ESP-IDF slave driver (v1, what the arduino-esp32 3.3.12 libraries are built with) only sets the peripheral up, and a handler on its shared interrupt reads every byte of a transaction and keeps the TX FIFO topped up. The v1 driver could not do the spec's write rules: its receive event carries no byte count (`i2c_slave_rx_done_event_data_t` has only the buffer; the v2 driver with a length is not compiled into the libraries), its job reads its own length from the FIFO whatever came, and its transmit ring has no slot boundaries. At STOP a write is judged by its byte count: an address-only write counts nothing; mode 1 queues a write of exactly the armed length and drops any other length with errors + 1, the wait going on (armed stays 1 after a frame; arm_rx just replaces the length, no restart); mode 2 takes the length byte and the body in one write (was two transactions), L = 0, L over max_length or a body of another count dropped with errors + 1; mode 3 ACKs a write (write-then-read with a repeated START still answers) and drops it with errors + 1. Mode 3: preload_tx keeps at most queue_depth (4) unread slots (beyond: unavailable cause 2, nothing placed) and answers the slots placed since configure (u8, wraps); every read starts at the next slot's first byte whatever length the last read took, sends 0xFF past a slot's end and with no slot, and status tx_slots counts the unread ones (one fewer per read). The ESP32-P4 filler byte after each slot is gone (the TX FIFO is reset after a read). configure clears frames, the wait, slots, rx_frames and errors (and keeps stretch); reset likewise keeps mode, address and stretch; releasing or replacing the plan goes back to describe's state (state 0, mode 0, all cleared, stretch 0). stretch (ESP32-P4): taken in any state up to max_stretch_us 100000 (declared; over it unsupported), applied to the running target at once: SCL is held at every received byte's ACK (slave_byte_ack_ctl_en) and at a read's address, and service() lets go after stretch_us instead of a busy wait in loop(). The classic ESP32's slave cannot hold SCL, so it no longer declares features bit1 (stretch is unknown_operation there; it used to be taken and do nothing). Not yet run on hardware: the two-board HIL must cover write lengths, mode 2, slots and stretch before a release. Host tests with a fake controller (`tests/host/test_i2c_target.cpp`).
- (JA) 破壊的変更: fixture I2C target を oep-spec 75ca9ee が定めた fixture §3 に揃えた（registry を写し直した: describe の `max_stretch_us` 0x41）。受信のバイトはこの target 自身が扱う: ESP-IDF の slave ドライバ（v1。arduino-esp32 3.3.12 のライブラリはこれでビルドされている）は周辺の設定だけをし、その共有の割り込みに置いた handler がトランザクションのすべての byte を読み、TX FIFO を満たし続ける。v1 のドライバでは仕様の書き込みの規則を守れなかった: 受信の event に byte 数が無い（`i2c_slave_rx_done_event_data_t` は buffer だけ。長さを持つ v2 のドライバはライブラリに入っていない）、job は何が来ても自分の長さを FIFO から読む、送信の ring に置き場の区切りが無い。STOP で書き込みを byte 数で判じる: アドレスだけの書き込みは何も数えない。mode 1 は arm した length ちょうどの書き込みを積み、違う長さは捨てて errors + 1、待ちは続く（フレームの後も armed は 1。arm_rx は length を替えるだけで作り直さない）。mode 2 は長さの byte と本文を 1 つの書き込みで受ける（2 つのトランザクションだった）。L = 0、L が max_length 超、本文の数が違うものは捨てて errors + 1。mode 3 は書き込みに ACK し（repeated START の write-then-read は引き続き答える）、捨てて errors + 1。mode 3: preload_tx は未読の置き場を queue_depth（4）個まで持ち（超えれば何も置かずに unavailable cause 2）、configure からの置いた数（u8、一周する）を答える。読み出しは、前の読み出しの長さによらず、いつも次の置き場の先頭から答え、置き場の終わりの後と置き場が無いときは 0xFF。status の tx_slots は未読の数（読み出しごとに 1 減る）。ESP32-P4 の置き場ごとの埋め草の 1 byte は無くなった（読み出しの後に TX FIFO を空にする）。configure はフレーム、待ち、置き場、rx_frames、errors を消す（stretch は保つ）。reset も同じで mode、address、stretch を保つ。plan を解く・置き換えると describe の直後の状態（state 0、mode 0、すべて消え、stretch 0）。stretch（ESP32-P4）: state によらず max_stretch_us 100000（宣言する。超えれば unsupported）まで受け、動いている target にすぐ効く: 受けた byte ごとの ACK（slave_byte_ack_ctl_en）と読み出しのアドレスで SCL を保ち、loop() での busy wait の代わりに service() が stretch_us の後に放す。classic ESP32 の slave は SCL を保てないので features の bit1 を宣言しない（stretch はそこでは unknown_operation。これまでは受けて何もしなかった）。まだ実機で動かしていない: リリースの前に二台の HIL で書き込みの長さ、mode 2、置き場、stretch を確かめること。偽の controller による host テスト（`tests/host/test_i2c_target.cpp`）。
- (EN) Fixture SPI target, the rest of fixture §4 as oep-spec 75ca9ee defines it: configure clears transactions and errors (as well as the queue and the wait); releasing or replacing the plan goes back to describe's state (state 0, mode and bit_order 0, counts cleared); read_rx in state 0 is rejected unavailable cause 6 (was an empty answer). A CS frame with no SCK edge counting nothing is now the spec's rule. Host tests add these and over length with the queue full (errors + 2).
- (JA) fixture SPI target の残りを oep-spec 75ca9ee が定めた fixture §4 に揃えた: configure は transactions と errors を消す（列と待ちに加えて）。plan を解く・置き換えると describe の直後の状態（state 0、mode と bit_order は 0、数は消える）。state 0 の read_rx は rejected unavailable cause 6（空の応答だった）。SCK の edge が無い CS の区切りを何も数えないのは仕様の規則になった。host テストにこれらと、列が満ちているときの length 超過（errors + 2）を加えた。
- (EN) Fixture SPI target follows fixture §4 for transfers nobody armed: one that ends while not armed drops MOSI (MISO 0) and counts in both `transactions` and `errors` (the probe used to see none of them: the spi_slave driver had no transaction queued, so they counted nothing). While nothing is armed a discard transaction waits in the driver; arm restarts the target in place of it (the driver cannot take a queued transaction back), so the armed one is the next transfer. A transfer that ends while armed still consumes the arm and is queued, not an error; one longer than the armed length now also counts an error (data up to length, bits as they came). Implementation choice, where the spec says nothing: a CS frame with no SCK edge (0 bits) is noise - it counts nothing and an arm keeps waiting. With SCK / CS floating an M5Stack ATOM saw phantom transfers that consumed an arm and counted no error (2026-10-02, found by oep-client-python tests/hw on 0.0.27). Host tests on a fake spi_slave driver (`tests/host/test_spi_target.cpp`).
- (JA) fixture SPI target を、arm していない転送について fixture §4 に揃えた: arm していない間に終わった転送は MOSI を捨て（MISO は 0）、`transactions` と `errors` の両方を数える（これまでは spi_slave のドライバに転送が積まれておらず、probe はそれを見ず、何も数えなかった）。arm していない間は捨てる用の転送をドライバに置いておき、arm はその代わりに target を作り直す（ドライバは積んだ転送を取り戻せない）ので、次の転送が arm したものになる。arm している間に終わった転送はこれまでどおり arm を使い切って列に積み、errors にしない。arm した length より長い転送は errors も数えるようにした（data は length まで、bits は来たまま）。仕様が何も言わないところの実装の選択: SCK の edge が 1 つも無い CS の区切り（0 bit）は雑音として何も数えず、arm は待ったまま。SCK / CS を浮かせた M5Stack ATOM で、幻の転送が arm を使い切り、errors を数えなかった（2026-10-02、0.0.27 で oep-client-python の tests/hw が見つけた）。偽の spi_slave ドライバの上の host テスト（`tests/host/test_spi_target.cpp`）。
- (EN) Breaking: fixture logic (`LogicCapture`, ESP32-P4; `SamplerCapture`, classic ESP32) declares role_channels (core §7.4) like the other fixtures: roles 0 to 15 (8 on the classic ESP32; role k = channel k, oep-if-capture) on any channel of the probe's PinTable. The constructors take that `PinTable` in place of the reserved-pin mask (`LogicCapture(endpoint, pins, instance)`, `SamplerCapture(endpoint, pins)`), and planCheck refuses a channel outside it as unsupported (was unavailable). The capture claims no channel, so it may still listen on pins another fixture holds. The PinTable also leaves out the pins the chip's package uses itself (`forbid`), which the reserved mask did not. `05.Capture/LogicCapture` makes a PinTable of its channels.
- (JA) 破壊的変更: fixture logic（ESP32-P4 の `LogicCapture`、classic ESP32 の `SamplerCapture`）が、ほかの fixture と同じく role_channels（core §7.4）を宣言する: role 0〜15（classic ESP32 は 8 本。role k = チャネル k、oep-if-capture）が probe の PinTable のどの channel でもとれる。コンストラクタは reserved のピンのマスクの代わりにその `PinTable` を受け（`LogicCapture(endpoint, pins, instance)`、`SamplerCapture(endpoint, pins)`）、planCheck はそれ以外の channel を unsupported で断る（unavailable だった）。capture は channel を claim しないので、ほかの fixture が持つピンも引き続き聞ける。PinTable はチップのパッケージ自身が使うピン（`forbid`）も外す（reserved のマスクでは外れていなかった）。`05.Capture/LogicCapture` は自分の channel の PinTable を作る。

## 0.0.27
- (EN) port_speed follows core §3.5 as rewritten (oep-spec eab0c16: the section is the handshake only; the rate choice and verification are the host guide's). A step that does not fit the port's state is rejected unavailable cause 6, the same refusal as the wrong port: try while the port is trying or committed, commit at the boot speed or when committed already (a second commit used to be taken and reset idle_ms), revert at the boot speed (used to be taken as a no-op). A step over 2 is rejected malformed (was unsupported). A committed port goes back after three broken candidates in a row with no good frame between them, whatever their spacing (was three within one second: a host that fell back to the boot speed and confirms slowly never reached the window). idle_ms is not counted while a request runs - it runs from the answer, like the lease - so a block op or verify longer than idle_ms no longer takes the port back under the host. Host tests cover each case.
- (JA) port_speed を書き直した core §3.5（oep-spec eab0c16: この節は握手だけ。速さの選び方と確かめは host ガイドへ）に揃えた。口の状態に合わない step は、口が違うときと同じ rejected unavailable cause 6: 試し・決めたの口への試す、起動時や決めた後の決める（2 度目の決めるはこれまで受けて idle_ms を取り直していた）、起動時の戻す（これまで何もせず受けていた）。step が 2 より大きければ rejected malformed（unsupported だった）。決めた口は、正常なフレームを挟まず壊れが 3 つ続いたら間隔によらず戻る（1 秒以内に 3 つだった: 起動時の速さに戻った host がゆっくり confirm を送ると窓に入らなかった）。idle_ms は要求を実行している間は数えず、lease と同じく応答から数える: idle_ms より長い block op や verify が host の下で口を戻してしまわない。それぞれ host のテストで確かめる。
- (EN) read_block / write_block (riscv-dm and arm-adi): a count whose count × 4 exceeds the declared max_length is rejected unsupported with payload `0x00` (oep-if-debug §4.5 / §6, core §7.4; was malformed); an address that is not a multiple of 4 stays malformed, count 0 stays success with done 0. max_length (describe 0x03) is derived from the endpoint's max_frame through `blockMaxLength` (max_frame − 24, a multiple of 4, and within the word buffer), so both the read_block answer and the write_block request fit a frame: 488 bytes on the ESP32 profile (max_frame 512; was 492, whose write_block request did not fit), 1000 on ESP32-P4 and RP2 (max_frame 1024; was 1004). arm-adi did not declare max_length at all and took any count that fit the frame; it now declares 1000 (RP2) and refuses over it like riscv-dm. Host tests cover the declared values and the refusal.
- (JA) read_block / write_block（riscv-dm と arm-adi）: count × 4 が宣言した max_length を超える要求は payload `0x00` の rejected unsupported（oep-if-debug §4.5 / §6、core §7.4。malformed だった）。4 の倍数でない address は malformed のまま、count 0 は success・done 0 のまま。max_length（describe の 0x03）は endpoint の max_frame から `blockMaxLength` で求める（max_frame − 24、4 の倍数、語のバッファ以内）ので、read_block の応答も write_block の要求もフレームに収まる: ESP32 の profile は 488 byte（max_frame 512。492 だったが、その write_block の要求は収まらなかった）、ESP32-P4 と RP2 は 1000（max_frame 1024。1004 だった）。arm-adi は max_length を宣言しておらず、フレームに収まる count を何でも受けていた。今は 1000（RP2）を宣言し、超えれば riscv-dm と同じく断る。宣言する値と断りは host のテストで確かめる。
- (EN) Fixture UART: the channels a role may take are declared per role (`FixtureUart::setRoleChannels(rx_mask, tx_mask)`; describe's role_channels for RX and TX differ, planCheck refuses others as unsupported). RP2 (`Firmware/OepProbe` Rp2.h): Serial1 = UART0 reaches only a few pins each way (RP2040: TX 0 / 12 / 16 / 28, RX 1 / 13 / 17 / 29; RP2350A adds 2 / 14 / 18 and 3 / 15 / 19), and arduino-pico's `setRX` / `setTX` panic() on any other pin - a plan rx 4 / tx 5 followed by configure froze the Pro Micro RP2350 for good (USB dead, 2026-10-02, found by oep-client-python tests/hw). `platformUartBegin` on RP2 also refuses such a pin instead of reaching the core. `platformUartRxMask` / `platformUartTxMask(uart_index)`.
- (JA) fixture UART: 役ごとに使える channel を宣言する（`FixtureUart::setRoleChannels(rx_mask, tx_mask)`。describe の role_channels が RX と TX で異なり、planCheck はそれ以外を unsupported で断る）。RP2（`Firmware/OepProbe` の Rp2.h）: Serial1 = UART0 が届くピンは片方向に数本だけ（RP2040: TX 0 / 12 / 16 / 28、RX 1 / 13 / 17 / 29。RP2350A は 2 / 14 / 18 と 3 / 15 / 19 が加わる）で、arduino-pico の `setRX` / `setTX` はそれ以外のピンで panic() する。plan rx 4 / tx 5 の後の configure で Pro Micro RP2350 が固まったまま戻らなかった（USB も死ぬ、2026-10-02、oep-client-python の tests/hw が見つけた）。RP2 の `platformUartBegin` も core に渡す前にそのピンを断る。`platformUartRxMask` / `platformUartTxMask(uart_index)`。

## 0.0.25
- (EN) port_speed: once committed, the probe goes back to the boot speed after at most `port_speed_idle_max_ms` (3000 ms, oep-spec 877cf01, core §3.5 items 5 and 6; registry synced) with no good frame on that port. idle_ms 0 and anything longer count as that maximum (0 used to mean never): a host that raised the speed and died no longer leaves the port at a rate the next host cannot reach, however long the lease. A host keeps the line alive with keepalives (or other requests) more often than every 3 s. Host tests: idle 0 reverts after 3000 ms, a large idle_ms is clamped.
- (JA) port_speed: 決めた後は、その口に正しいフレームが来ないまま最長 `port_speed_idle_max_ms`（3000 ms、oep-spec 877cf01、core §3.5 の 5 と 6。registry を写し直した）で起動時の速さに戻る。idle_ms の 0 とそれより長い値はその最長として扱う（0 はこれまで戻らないだった）: 速さを上げた host が落ちても、lease が長くても、次の host が届かない速さに口が残らない。host は keepalive（や他の要求）を 3 秒より短い間隔で送って線を保つ。host テスト: idle 0 は 3000 ms で戻る、大きな idle_ms は最長に切り詰める。
- (EN) Fixture UART (ESP32): the receive errors the UART driver reports - its FIFO or buffer overflowing, a framing error or break, a parity error - are marked lost (detail 1 / 2 / 3) at the stream's position, as fixture §2 says. A byte lost at 2 Mbaud on the X035 jig's P4 left no mark at all (2026-10-01, seen on 0.0.23 and 0.0.24 alike: not a regression). RP2: the core reports none.
- (JA) fixture UART（ESP32）: UART のドライバが知らせる受信の誤り（FIFO やバッファのあふれ、framing の誤りと break、parity の誤り）を、ストリームの位置に lost（detail 1 / 2 / 3）のマークとして付ける（fixture §2 のとおり）。X035 のジグの P4 で 2 Mbaud のときに byte が落ちても、マークが何も付かなかった（2026-10-01。0.0.23 でも 0.0.24 でも起き、退行ではない）。RP2 は core が知らせない。

## 0.0.24
- (EN) port_speed's try state ignores a broken candidate until a good frame has come at the new speed: the bytes in flight while both ends switch closed as one on every switch on an M5Stack ATOM's FTDI, and reverting on it threw away rates that verified cleanly (dogfooding, 2026-10-01). A rate that never carries a good frame still runs out its verify_ms.
- (JA) port_speed の試しの状態は、新しい速さで正しいフレームが来るまで、壊れた候補を数えない: 両端が切り替わる間に流れていたバイトが、M5Stack ATOM の FTDI では切り替えのたびに壊れた候補になり、それで戻すと、確かめれば通る速さまで捨てていた（ドッグフーディング、2026-10-01）。正しいフレームが一度も来ない速さは、これまでどおり verify_ms で戻る。
- (EN) `port_speed` (oep-spec 3b9311c, core §3.5, fn 0 op 0x14; registry synced): the optional raise of a UART bridge's baud for a session. `Endpoint::setPortSpeed(fn, base)` turns it on (describe port_speed 0x4E = 1, the op taken; without it the op stays unknown_operation): `fn(port, baud, apply)` says the rate the UART would run at (0: unsupported) or switches it, `base` is the boot speed every revert goes back to. The request must come in on that UART bridge (else unavailable cause 6) and hold the lock. try (step 0) answers at the old speed, then switches; a commit (step 1, the same baud, at the new speed) must come within verify_ms, and any broken candidate on that port before it reverts at once. Committed: idle_ms (the commit's; 0 off) with no good frame, or 3 broken candidates within 1 s, revert. revert (step 2) answers the boot speed at the speed now, then reverts; the session's end (end, lapse, force) reverts after its answer. `SerialReader::badCandidates()` counts the candidates closed by a 0x00 that were not frames. `Firmware/OepProbe` classic ESP32: on (UART0, `Serial.updateBaudRate`, base 115200; `-DOEP_PORT_SPEED=0` leaves it out), and max_inflight 2 / window 1024 (was 1 / 512): at a raised speed the second frame in flight nearly doubles throughput, at 115200 it changes nothing. Host tests for the state machine.
- (JA) `port_speed`（oep-spec 3b9311c、core §3.5、fn 0 の op 0x14。registry を写し直した）: セッションの間 UART bridge の速さを上げる任意の機能。`Endpoint::setPortSpeed(fn, base)` で ON（describe の port_speed 0x4E = 1 を出し、op を受ける。無ければ op は unknown_operation のまま）: `fn(port, baud, apply)` は UART がその baud で実際に掛かる速さ（0 は unsupported）を返すか切り替える。`base` は戻り先の起動時の速さ。要求はその UART bridge から来てロックを持つこと（でなければ unavailable cause 6）。試す（step 0）は今の速さで応答してから切り替え、決める（step 1、同じ baud、新しい速さで）が verify_ms のうちに来なければ戻り、その前にその口に壊れた候補が来ればすぐ戻る。決めた後は、idle_ms（決めるの要求のもの。0 は確かめない）の間正しいフレームが来ない、または 1 秒に 3 つの壊れた候補で戻る。戻す（step 2）は今の速さで起動時の速さを答えてから戻る。セッションの終わり（end、lease の期限切れ、force）は応答の後に戻る。`SerialReader::badCandidates()` は 0x00 で閉じたがフレームでなかった候補を数える。`Firmware/OepProbe` の classic ESP32: ON（UART0、`Serial.updateBaudRate`、起動時 115200。`-DOEP_PORT_SPEED=0` で外す）、max_inflight 2 / window 1024（1 / 512 から）: 上げた速さでは 2 つ目のフレームで速度がほぼ倍になり、115200 では変わらない。状態の遷移の host テスト。
- (EN) `oep.probe.config` item `disable` (0x07, oep-spec 6088438; registry synced): a channel the probe never uses or touches (not on this board, or wired to another part). Every request naming it is rejected unavailable cause 5 (held by settings) with the channel: plan_apply, a settings plan or slot, an attach's pins / reset TLV (rvswd, swio, swd), scan pairs, gpio set / read; scan's count-0 list leaves it out. A set disabling a channel in use (a plan, a connection, a slot) is cause 1; idle and disable for one channel is malformed; describe is unchanged (the items list takes 0x07 when the sketch gave `setPins`). The PinTable keeps the settings' disabled channels (`setDisabled`, `disabled`, `disabledMask`) apart from the firmware's `forbid`: a disabled channel is never claimed, set to its idle state or parked, and one enabled again goes to its idle state. Boot: `config.load()` now comes first in `Firmware/OepProbe` (classic ESP32, RP2) and `06.Settings/ProbeConfig`, `pins.setDisabled(config.savedDisabled())` and the start-up park leaves those pins alone; `applySaved()` gives them back when the saved settings are not applied. `Endpoint::setDisabled`. Following oep-spec 4842e95 (registry resynced): every cause 5 refusal for a disabled channel carries holder_kind 6 (`disabled`), and a disable item for a channel the firmware does not offer (not in the PinTable, or forbidden) is rejected unsupported, as idle.
- (JA) `oep.probe.config` の項目 `disable`（0x07、oep-spec 6088438。registry を写し直した）: probe が一切使わず触れない channel（ボードに出ていない、ほかの部品につながっている）。それを指す要求は channel 付きの rejected unavailable cause 5（設定が持つ）: plan_apply、設定の plan やスロット、attach の pins / reset TLV（rvswd、swio、swd）、scan の組、gpio の set / read。scan の count 0 の並びに入れない。使われている channel（plan、接続、スロット）を無効にする set は cause 1。同じ channel の idle と disable は malformed。describe は変わらない（スケッチが `setPins` を渡したとき items に 0x07 が入る）。PinTable は設定の無効（`setDisabled`、`disabled`、`disabledMask`）を firmware の `forbid` と分けて持つ: 無効の channel は claim されず、空きの状態にも、起動時の Hi-Z にもされない。有効に戻った channel は空きの状態になる。起動: `Firmware/OepProbe`（classic ESP32、RP2）と `06.Settings/ProbeConfig` は `config.load()` を最初に呼び、`pins.setDisabled(config.savedDisabled())` で起動時の Hi-Z からそのピンを外す。保存を適用しなかったときは `applySaved()` が戻す。`Endpoint::setDisabled`。oep-spec 4842e95 に追随（registry を写し直した）: 無効にした channel の cause 5 の断りはすべて holder_kind 6（`disabled`）を持ち、firmware が出していない channel（PinTable に無い、forbid された）の disable の項目は rejected unsupported（idle と同じ）。
- (EN) Classic ESP32: the pins the chip's own package uses are found at start-up and left out of every channel set (and declared reserved): GPIO16 / 17 on an ESP32-PICO-D4 / PICO-V3 / D2WD (their in-package flash), or with a PSRAM the build enabled (a build with PSRAM off leaves them free). The released `OepProbe-esp32` image parked them Hi-Z and an M5Stack ATOM (PICO-D4) reset at boot over and over (TG1WDT); it now runs there. `PinTable::forbid`, `platformUnusablePins()`.
- (JA) classic ESP32: チップのパッケージ自身が使うピンを起動時に調べ、どのチャンネルの集合からも外す（reserved として宣言する）: ESP32-PICO-D4 / PICO-V3 / D2WD（パッケージ内の flash）、またはビルドで有効にした PSRAM があるときの GPIO16 / 17（PSRAM を無効にしたビルドでは使える）。リリースの `OepProbe-esp32` はそれを Hi-Z にして、M5Stack ATOM（PICO-D4）が起動のたびにリセットした（TG1WDT）。今はそこで動く。`PinTable::forbid`、`platformUnusablePins()`。

## 0.0.23
- (EN) The retry table keeps answers up to a whole 1 KiB frame (8 entries; was 72 bytes × 16): a read_block answer corrupted on the link is answered from the table when the host sends the request again, instead of rejected result_lost (the V003 jig's CP2102 link dropped bytes in long frames, 2026-10-01).
- (JA) 送り直しの表は 1 KiB のフレーム丸ごとまで覚える（8 個。72 byte × 16 だった）。線の上で壊れた read_block の答えは、host が同じ要求を送り直したときに表から答え、rejected result_lost にしない（V003 ジグの CP2102 の経路が長いフレームで byte を落とした、2026-10-01）。
- (EN) CH32 riscv-dm: a block op's fixed cost is smaller (ABSTRACTAUTO is written only when it may be set; the four GPR restores share one completion check; cmderr is cleared only when one was raised): about 42 DMI round trips around a read_block instead of about 70 (X035 measured 5.1 ms of fixed cost per read_block at 777 kHz, 2026-10-01).
- (JA) CH32 riscv-dm: block op の固定のコストを減らした（ABSTRACTAUTO は立っている可能性があるときだけ書く、GPR 4 本の復元は完了の確認を 1 回にまとめる、cmderr は出たときだけ消す）。read_block 1 回あたりの DMI の往復が約 70 回から約 42 回に（X035 で 777 kHz のとき固定分 5.1 ms を測った、2026-10-01）。

## 0.0.22
- (EN) Breaking: the probe follows the rewritten OEP v1 spec of 2026-10-01 (oep-spec docs/v1-zero-base-proposal.ja.md §3 / §7; registry ec281256). Every answer that ended in raw data carries a length (read: `start flags len(u16) data`, capture read `position flags len(u32) data`, gpio read `n n x level`, dmi `done status nvals(u16) values`, arm-adi transfer `done status ack nvals values`, run `status stopped dpc elapsed_us nvals(u8) values`); data frames are `position len(u16) data [TLV]`; TLVs of 255 bytes and more use the long form (`tag 0xFF len(u16)`); confirm answers `boot_id`; subscribe's max_delay_ms is u32; a time is always ns since boot (marks `time_ns`, the heartbeat `uptime_ns`, the slot's `last_try_at_ns`).
- (JA) 破壊的変更: 2026-10-01 の書き直した OEP v1 の仕様（oep-spec docs/v1-zero-base-proposal.ja.md §3 / §7、registry ec281256）に揃えた。生のデータで終わっていた応答はすべて長さを持つ（read は `start flags len(u16) data`、capture の read は `position flags len(u32) data`、gpio の read は `n n × level`、dmi は `done status nvals(u16) values`、arm-adi の transfer は `done status ack nvals values`、run は `status stopped dpc elapsed_us nvals(u8) values`）。データフレームは `position len(u16) data [TLV]`。255 byte 以上の TLV は長い形（`tag 0xFF len(u16)`）。confirm は `boot_id` を返す。subscribe の max_delay_ms は u32。時刻は常に起動からの ns（マークの `time_ns`、ハートビートの `uptime_ns`、スロットの `last_try_at_ns`）。
- (EN) Breaking: sessions and resources. A lease that lapsed sweeps the session's resources: the next request with that session id is rejected `expired` (0x0E) and its open says `resumed` 2; an end still keeps them (resumed 1); a takeover by force sweeps them too and forgets the id (that host meets locked, then no_session). Opening the held id again keeps the subscriptions and moves them to that transport. `connections` (wire op 0x05) and console `streams` (0x08) take `first(u8)` and answer `more count entries`. Connections and streams take their numbers from one probe-wide u16 counter (1 upwards, wrapping, recently closed numbers avoided); a live number of another kind is rejected unavailable cause 6. rejected unsupported always carries `tag(u8)` (0x00 = a fixed-part value; gpio set adds channel / index TLVs); subscribing to a fn that emits nothing is unsupported, unsubscribing nothing is fine.
- (JA) 破壊的変更: セッションと資源。lease の期限切れはセッションの資源を外す: その session_id の次の要求は rejected `expired`（0x0E）、その open は `resumed` 2。end は残す（resumed 1）。force での奪取も資源を外し、その ID は忘れる（奪われた host は locked、その後 no_session）。持っている ID をもう一度 open すると購読は残り、送り先がその経路に替わる。`connections`（線の op 0x05）と console の `streams`（0x08）は `first(u8)` を受け、`more count entries` を返す。connection とストリームの番号は probe で 1 つの u16 のカウンタ（1 から、一周し、直近に閉じた番号は避ける）。別の種類の生きている番号は rejected unavailable cause 6。rejected unsupported は常に `tag(u8)` を持つ（0x00 = 固定部の値。gpio の set は channel / index の TLV を足す）。送り出さない fn の subscribe は unsupported、無い購読の unsubscribe は ok。
- (EN) Breaking: fn 0's describe has `discoverable` (0x4A, was oep_pid), `max_op_ms` (0x4D, 10000) and plan_roles as u32; its `label` TLVs are the firmware's fixed names only (the settings' labels come from probe.config get). `Endpoint::setOepPid` is `setDiscoverable`; `setProbeExtra` is gone. The P4's vendor bulk interface carries bInterfaceSubClass 0x4F / bInterfaceProtocol 0x45 and its HID usage page 0xFF4F / usage 0x45 (Esp32P4.h patches EspUsbDevice's descriptors).
- (JA) 破壊的変更: fn 0 の describe に `discoverable`（0x4A、旧 oep_pid）と `max_op_ms`（0x4D、10000）、plan_roles は u32。`label` の TLV は firmware の固定の名前だけ（設定の label は probe.config の get で読む）。`Endpoint::setOepPid` は `setDiscoverable` に。`setProbeExtra` は無くなった。P4 の vendor bulk のインターフェースは bInterfaceSubClass 0x4F / bInterfaceProtocol 0x45、HID は usage page 0xFF4F / usage 0x45（Esp32P4.h が EspUsbDevice の記述子を直す）。
- (EN) Breaking, oep.probe.config: describe is `storage(max_bytes) items slots_max bind_modes(u32)` only; the state (storage, slot_state with `last_try_at_ns`, bind_state) is the new lock-free op `state` (0x06, paged by first_slot / first_bind); `unset` (0x05) removes items by key; the new `uart` item (0x06: fn baud format) sets a fixture UART when its plan has pins (a session's configure wins until the plan is released); the slot carries `retry_ms(u32)` and `max_speed_hz(u32)`, mechanism 0xFF = no console (never bound), its lock must be 4 bytes (scheme 1), and only rvswd / swio may be its wire; the probe keeps every item's bytes as sent and hashes the canonical form (TLV, tag then key order); storage max_bytes is what always fits (384); replacing or removing a slot drops its share of the connection and its bound console (mark closed 3); an at-boot slot reads DMSTATUS every retry_ms and closes (and retries) on line loss. Saved settings from 0.0.21 and before are not readable (storage unreadable, form; NVS "items4" / EEPROM "OEP4"): set and save them again.
- (JA) 破壊的変更、oep.probe.config: describe は `storage(max_bytes) items slots_max bind_modes(u32)` だけ。状態（storage、`last_try_at_ns` 付きの slot_state、bind_state）はロック不要の新しい op `state`（0x06、first_slot / first_bind でページング）。`unset`（0x05）はキーで項目を消す。新しい `uart` 項目（0x06: fn baud format）は fixture UART の plan にピンが付いたときに掛かる（セッションの configure は plan を解くまで勝つ）。スロットは `retry_ms(u32)` と `max_speed_hz(u32)` を持ち、mechanism 0xFF = コンソールなし（bind に載せない）、錠は 4 byte（scheme 1）、線は rvswd / swio だけ。probe は項目のバイト列を送られたまま持ち、正規形（TLV、tag の次にキーの順）を hash する。storage の max_bytes は必ず入る量（384）。スロットの置き換え・削除はその接続の分と bind のコンソールを外す（mark closed 3）。at boot のスロットは retry_ms ごとに DMSTATUS を読み、線切れなら閉じてやり直す。0.0.21 までに保存した設定は読めない（storage unreadable、form。NVS "items4" / EEPROM "OEP4"）: もう一度 set して save する。
- (EN) Breaking, debug: `attach_under_reset` is gone; attach takes TLV 0x05 `reset(channel, hold_ms)` (critical; method 1 = stopped before the first instruction, method 0 = reset and left running; on a live connection = a reset of it, mark reset 3) and answers `connection id flags speed_hz [TLV 0x10 target_id, 0x11 dpc]` with the common attach_flags (havereset_acked 1, existing 2, dormant_woken 4, halted 8); max_speed is required (malformed without it, unsupported under min_clock_hz, which the wires now declare); scan takes max_speed 0x01 / skip 0x02 / idle_clock 0x04 and writes only dmactive; a failed scan / attach / detach answers `status [TLV]`. Every riscv-dm op puts back what it touched before answering (read_block / write_block: s0, s1, a0, a1, DATA1 / DATA0, abstractauto; step: dcsr.step and DATA; reset lowers haltreq afterwards; halt may keep haltreq while halted); the 0.0.20 carry-over (the raw DMI write restoring, hostRaw) is gone and the console asks DMSTATUS whether the hart runs. run's timeout_ms is 1..max_op_ms (0 malformed, over it unsupported) and the answer always has the same shape (stopped 2 = could not be halted, nvals 0); dmi's waits may not add up to more than max_op_ms. A request that finds the line gone answers status line and then closes the connection; havereset seen anywhere is acknowledged and marked restart 1. swd: attach has a method byte (1 unsupported), connections entries carry tid scheme 2 = TARGETSEL, scan takes max_speed / targetsel.
- (JA) 破壊的変更、debug: `attach_under_reset` は無くなり、attach の TLV 0x05 `reset(channel, hold_ms)`（critical。method 1 = 最初の命令の前で止める、method 0 = reset して走ったまま。生きている connection には reset、mark reset 3）になった。応答は `connection id flags speed_hz [TLV 0x10 target_id、0x11 dpc]`、flags は共通の attach_flags（havereset_acked 1、existing 2、dormant_woken 4、halted 8）。max_speed は必須（無ければ malformed、min_clock_hz 未満は unsupported。線が min_clock_hz を宣言する）。scan は max_speed 0x01 / skip 0x02 / idle_clock 0x04 を受け、dmactive しか書かない。失敗した scan / attach / detach は `status [TLV]`。riscv-dm のどの op も触ったものを応答の前に戻す（read_block / write_block: s0、s1、a0、a1、DATA1 / DATA0、abstractauto。step: dcsr.step と DATA。reset は終わったら haltreq を下ろす。halt は止まっている間 haltreq を保ってよい）。0.0.20 の持ち越し（raw DMI 書き込みでの戻し、hostRaw）は無くなり、コンソールは hart が走っているかを DMSTATUS に聞く。run の timeout_ms は 1〜max_op_ms（0 は malformed、超えれば unsupported）で応答の形はいつも同じ（stopped 2 = 止められなかった、nvals 0）。dmi の待ちの和は max_op_ms まで。線切れを見た要求は status line で答えてから connection を閉じる。havereset はどこで見ても確認応答して mark restart 1。swd: attach に method のバイト（1 は unsupported）、connections の tid は scheme 2 = TARGETSEL、scan は max_speed / targetsel を受ける。
- (EN) Breaking, console and fixture: console `streams` (0x08, lock-free); one live stream per connection, used by the host's session and the slot (close / lapse / the settings drop their share; mark closed 0x09 with details 1-4; link-lost before closed 4 when the line went); a same-place re-open gives the same stream number back; write's accepted is what fit the mechanism's slot (0 = failed). fixture uart: `status` (0x07; configured 0 default, 1 session configure, 2 settings item, 3 item whose baud could not be made, the default applied), the stream is made by the plan (115200 8N1 or the uart item until configured) and its positions never go back within a boot, format is checked (undefined bits malformed), a baud more than 5 % off is unsupported, describe formats `n n x u8`. gpio describe modes u32. i2c / spi: errors u32, describe queue_depth (0x40), undefined modes malformed, over max_length unsupported, reset / arm in state 0 unavailable cause 6.
- (JA) 破壊的変更、console と fixture: console に `streams`（0x08、ロック不要）。connection に生きているストリームは 1 つで、host のセッションとスロットが使う（close / lease 切れ / 設定は自分の分を外す。mark closed 0x09 と detail 1〜4。線が落ちたときは closed 4 の前に link-lost）。同じ場所の再 open は同じ番号を返す。write の accepted は方式の送り枠に入った分（0 = failed）。fixture uart: `status`（0x07。configured は 0 既定、1 セッションの configure、2 設定の項目、3 項目の baud を作れず既定を掛けた）。ストリームは plan が作り（configure まで 115200 8N1 か uart の項目）、位置は起動の中で戻らない。format を確かめ（未定義のビットは malformed）、5% を超えて外れる baud は unsupported、describe の formats は `n n × u8`。gpio の describe の modes は u32。i2c / spi: errors は u32、describe に queue_depth（0x40）、未定義の mode は malformed、max_length 超は unsupported、state 0 の reset / arm は unavailable cause 6。
- (EN) Breaking, capture: a generation (u32) per start - start answers `blocking_ms generation`, status `state serial_done write_pos flags generation [TLV error]`, the segment record is 37 bytes (+ generation), read is `generation position max` and release `generation serial` (another generation: unavailable cause 6), streaming data frames carry TLV 0x01 generation; trigger value is u32 (TLV of 6 bytes); segments answers `more count ...`; a repeat with no free segment pauses and resumes on release without a stopped event; write_pos counts dropped bytes; describe: mode is the maximum (not the free space), rate_list `n n x u32`, channels `max layouts(u32)`, trigger `types(u32) max_pretrigger`; a bound track's own configure / start / stop / force are unavailable cause 4 with the group's fn; capture-group events are triggered 0x03 / stopped 0x02 (reason, error), its start answers TLV 0x01 generations, describe tracks / budget carry n(u8); analog calibration factory has `raw_len(u16)`. configure during a capture is unavailable cause 6 (was busy).
- (JA) 破壊的変更、capture: start ごとの世代（u32）。start は `blocking_ms generation`、status は `state serial_done write_pos flags generation [TLV error]`、区画の情報は 37 byte（+ generation）、read は `generation position max`、release は `generation serial`（別の世代は unavailable cause 6）、ストリーミングのデータフレームは TLV 0x01 generation を持つ。trigger の value は u32（6 byte の TLV）。segments は `more count …`。空き区画の無いリピートは止まり、release で自動再開し stopped は送らない。write_pos は捨てた分を含む。describe: mode は最大（空きではない）、rate_list は `n n × u32`、channels は `max layouts(u32)`、trigger は `types(u32) max_pretrigger`。束ねられたトラック自身の configure / start / stop / force は組の fn 付きの unavailable cause 4。capture-group の出来事は triggered 0x03 / stopped 0x02（reason、error）、start の応答に TLV 0x01 generations、describe の tracks / budget に n(u8)。analog の calibration の factory に `raw_len(u16)`。取得中の configure は unavailable cause 6（以前は busy）。

## 0.0.21
- (EN) Breaking: a bind's streams carry a length each (oep-spec probe.config §1.2, freeze decision 1, missed until now): `n × (len, kind, id)`. The probe skips a longer one's tail, refuses one under 3 as malformed, and returns len 3. A bind saved by 0.0.20 is refused on load (storage unreadable: set it again).
- (JA) 破壊的変更: bind のストリームの並びの各要素の前に長さを置く（oep-spec probe.config §1.2。凍結の決定 1 で漏れていた）: `n × (len、kind、id)`。probe は長い要素の後ろを飛ばし、3 未満は malformed で断り、返すときは len 3。0.0.20 で保存した bind は読み込みで断る（storage は unreadable。設定し直す）。
- (EN) Breaking: every example numbers its interfaces' instance from 0 per name, as core §7.2 says (they started at 1; the P4's second UART is instance 1, its logic capture 0). Saved settings name interfaces by (name, instance, revision), so those saved before load as unreadable (interface): set them again.
- (JA) 破壊的変更: どの example もインターフェースの instance を名前ごとに 0 から振る（core §7.2 のとおり。1 から始めていた。P4 の 2 つめの UART は instance 1、logic capture は 0）。保存した設定は (name, instance, revision) でインターフェースを指すので、前に保存したものは unreadable（interface）になる。設定し直す。
- (EN) `firmware-<version>.json` drops `chip` (it was the same as `model`, and not the describe chip TLV); the README describes the file and the firmware string (the library's release version).
- (JA) `firmware-<version>.json` から `chip` を外した（`model` と同じ値で、describe の chip の TLV とは別のものだった）。README にファイルの形と firmware の文字列（ライブラリのリリースの版）を書いた。

## 0.0.20
- (EN) CH32 riscv-dm: a raw DMI write from the host first puts back what the probe changed during the stop (s0, s1, a0, a1 used by a block op, then DATA0 / DATA1). They were dropped, so halt -> read_block -> a raw write -> resume let the target run with the block address in s0 (mcause 5 on the X035, reported by ch32rv).
- (JA) CH32 riscv-dm: host の raw DMI 書き込みの前に、停止中に probe が変えたもの（block op が使った s0, s1, a0, a1、次に DATA0 / DATA1）を戻すようにした。捨てていたため、halt → read_block → raw 書き込み → resume で target が s0 に block のアドレスを持ったまま動いた（X035 で mcause 5、ch32rv の報告）。
- (EN) `Rp2350L103Probe` and `Rp2040ZeroProbe` are gone: the Pro Micro RP2350 bench runs `Firmware/OepProbe` (profile promicrorp2350) with its settings - the L103 found by scan on GP0 / GP1, attached, read and kept as an at-boot slot on 0.0.19.
- (JA) `Rp2350L103Probe` と `Rp2040ZeroProbe` を消した: Pro Micro RP2350 のベンチは `Firmware/OepProbe`（profile promicrorp2350）と設定で動く。0.0.19 で、L103 を GP0 / GP1 に scan で見つけ、attach して読み、at boot のスロットとして保存した。

## 0.0.19
- (EN) The pre-freeze decisions (oep-spec docs/v1-freeze-decisions.ja.md), on the probe:
  - `oep.fixture.capture` is `oep.fixture.logic` (the registry's `reg::fixture_logic`); the I2C / SPI targets are the standard `oep.fixture.i2c-target` / `spi-target`: status as separate fields (state, mode, armed, queued, counts), `read_hw` gone, stretch is op 0x07.
  - Answer lists put each element's length first (core §2.3): list, scan, connections, marks, capture segments.
  - A scan answer takes at most 500 ms (the host asks again for the rest): an RP2350 bit-banging 26 free pins did not answer for seconds (0.0.18).
  - probe.config: the slot's lock is length-prefixed (lock_len); settings are saved with the (name, instance, revision) of each interface they name and renumbered at boot - another interface added or moved leaves them in force - and the storage says why it is unreadable (its last byte). The saved form changed (NVS "items3", EEPROM "OEP3"): settings saved by 0.0.18 or before are not read; set and save them again. Label items are taken and show in oep.core's describe.
  - rejected unavailable carries core §4.3's TLVs where the endpoint refuses a plan (the channel another plan holds, a settings' plan, over plan_roles) and gpio's set / read (the channel, its position as TLV 0x40). An unknown console stream is no_connection.
  - The unit id is text, lowercase hex (the ESP32's MAC, the RP2's flash unique id), and the USB serial number is the unit id: the P4's HS port drops "-hs", the RP2 sets it (USB.setSerialNumber).
  - describe model: `esp32p4` (was `esp32-p4`), `esp32`, `rp2040`, `rp2350`; chip `<part> v<rev>` without hyphens (`esp32p4 v1.3`). The capture status flags: bit0 the probe dropped data, bit1 the time base bent.
  - firmware-<version>.json: `"schema": 1`, each file's `model` and `chip` (from the profile's build.chip / build.mcu).
  - A SparkFun Pro Micro RP2350 profile (`promicrorp2350`, released as OepProbe-promicrorp2350): every GPIO but GP19 (its PSRAM select); the Pico 2 build kept GP23-25 / 29 from the L103 bench's RVSWD on GP24 / 23.
- (JA) 凍結前の決定（oep-spec docs/v1-freeze-decisions.ja.md）を probe に入れた:
  - `oep.fixture.capture` は `oep.fixture.logic`（registry の `reg::fixture_logic`）。I2C / SPI の target は標準の `oep.fixture.i2c-target` / `spi-target`: status は分けたフィールド（state、mode、armed、queued、数）、`read_hw` は無くなり、stretch は op 0x07。
  - 応答の並びは要素の前に長さを置く（core §2.3）: list、scan、connections、marks、capture の segments。
  - scan の 1 回の応答は 500 ms まで（残りは host がもう一度聞く）。空いた 26 本を bit-bang で試す RP2350 が、数秒答えなかった（0.0.18）。
  - probe.config: スロットの錠は長さ付き（lock_len）。設定は、指すインターフェースの (name、instance、revision) と一緒に保存し、起動時に番号を読み替える（ほかのインターフェースを足しても動かしても効いたまま）。storage は読めない理由を返す（最後のバイト）。保存の形が変わった（NVS "items3"、EEPROM "OEP3"）: 0.0.18 までに保存した設定は読まない。もう一度 set して save する。label の項目を受け、oep.core の describe に出す。
  - rejected unavailable は、endpoint が plan を断るとき（ほかの plan が持つ channel、設定の plan、plan_roles を超える）と gpio の set / read（channel と、並びの位置を TLV 0x40）で、core §4.3 の TLV を付ける。知らない console の stream は no_connection。
  - unit id は text で小文字の 16 進（ESP32 は MAC、RP2 は flash の unique id）。USB の serial number は unit id: P4 の HS の口は "-hs" を外し、RP2 は設定する（USB.setSerialNumber）。
  - describe の model: `esp32p4`（以前は `esp32-p4`）、`esp32`、`rp2040`、`rp2350`。chip は `<型番> v<rev>`、ハイフンなし（`esp32p4 v1.3`）。capture の status の flags: bit0 probe の中で落とした、bit1 時間の基準が曲がった。
  - firmware-<version>.json: `"schema": 1`、各ファイルの `model` と `chip`（profile の build.chip / build.mcu から）。
  - SparkFun Pro Micro RP2350 の profile（`promicrorp2350`、OepProbe-promicrorp2350 として出す）: GP19（PSRAM の選択）以外のすべての GPIO。Pico 2 のビルドは GP23〜25 / 29 を持っていて、L103 のベンチの RVSWD（GP24 / 23）を使えなかった。

## 0.0.18
- (EN) capture-group: a second triggered run in one boot follows its own trigger. The group asked the trigger track for its trigger's time while it waited for the followers' pretriggers - before that track had started, so it still held the last run's - and the followers cut their segments at that old time at once (ti 0; the group's trigger_ns the last run's, 0.0.17, both jigs).
- (JA) capture-group: 1 回の起動の中の 2 回目以降のトリガ付きの取得も、自分のトリガに従う。組は、従う側のプリトリガを待つ間（トリガのトラックを始める前）もトリガの時刻を聞いていたため、前回のトリガの時刻を受け取り、従う側がその古い時刻ですぐ区画を切っていた（ti 0。組の trigger_ns が前回のもの、0.0.17、両方の治具）。
- (EN) Classic ESP32 analog: the segment's start_ns is 100 us later, measured against the logic's edge in a capture-group at 10, 20 and 40 kS/s (the analog came 50-200 us early, a time rather than a count of samples); start_uncertainty_ns is 100 us plus one sample (it was a whole conversion frame and 100 us).
- (JA) classic ESP32 のアナログ: 区画の start_ns を 100 µs 後ろにした。capture-group でロジックのエッジと比べ、10、20、40 kS/s で測った（アナログが 50〜200 µs 早かった。サンプル数ではなく時間で一定）。start_uncertainty_ns は 100 µs + 1 サンプル（以前は変換フレーム 1 つ + 100 µs）。

## 0.0.17
- (EN) Firmware/OepProbe: the saved settings are applied again after a reboot. The sketches loaded and applied them before adding the analog and the group, so the interface list they were checked against never matched the one they were saved with: storage "unreadable" after every reboot since 0.0.11 (found after a DFU update, X035 jig). `ProbeConfig::load()` / `applySaved()` belong after the last `endpoint.add()`.
- (JA) Firmware/OepProbe: 保存した設定が、再起動の後にまた入るようにした。スケッチはアナログと組を足す前に設定を読んで入れていたので、確かめる相手のインターフェースの一覧が、保存したときの一覧と一致しなかった。0.0.11 から、再起動のたびに storage が「unreadable」になっていた（DFU で更新した後に見つかった、X035 の治具）。`ProbeConfig::load()` / `applySaved()` は、最後の `endpoint.add()` の後に呼ぶ。
- (EN) P4 one-shot without a trigger: when no 64 KiB of internal DMA RAM is left for the segment (the ring a trigger or repeat keeps allocated), the ring is freed and the allocation tried again, then PSRAM. 5 MHz x 4 ch x 130816 samples failed configure after triggered captures (0.0.15, X035 jig).
- (JA) トリガの無い P4 のワンショット: 区画のための 64 KiB の内部の DMA の RAM が残っていなければ（トリガやリピートが確保したままのリング）、リングを返してもう一度取り、次に PSRAM を使う。トリガ付きのキャプチャの後、5 MHz × 4 ch × 130816 サンプルの configure が失敗していた（0.0.15、X035 の治具）。
- (EN) The P4 logic trigger is looked for once its pretrigger has filled (as the sampler and the analog do): a trigger 133 us after the start gave 266 samples of a pretrigger of 1000. A capture-group starts its trigger track once every follower holds its pretrigger (the analog had no values yet at a trigger right after the start); force starts it at once.
- (JA) P4 のロジックのトリガは、プリトリガが埋まってから探す（sampler とアナログと同じ）。開始から 133 µs のトリガで、プリトリガ 1000 のうち 266 サンプルしか無かった。capture-group は、従うトラックがすべて自分のプリトリガを持ってからトリガのトラックを始める（開始直後のトリガで、アナログにまだ値が無かった）。force はすぐに始める。
- (EN) An analog channel is shared with nothing (oep-if-capture §1.2): its pad goes analog while it runs, cutting its digital input and output (classic ESP32 GPIO32 as logic and analog at once read no edges). `Interface::planShares()` false makes the endpoint refuse, rejected unavailable, any plan that puts another fn (a logic capture too) on an analog channel or the analog on another fn's channel, whichever comes second; `AnalogCapture::setPins(&pins, owner)` claims its channels in the pin table against wires and settings (Firmware/OepProbe: owner 7).
- (JA) アナログのチャネルは誰とも共有しない（oep-if-capture §1.2）。取っている間 pad がアナログになり、デジタルの入出力が切れる（classic ESP32 の GPIO32 をロジックとアナログで同時に取ると、エッジが出なかった）。`Interface::planShares()` が false なら、endpoint は、アナログのチャネルにほかの fn（ロジックのキャプチャも）を載せる plan も、ほかの fn のチャネルにアナログを載せる plan も、後から来た方を rejected unavailable で断る。`AnalogCapture::setPins(&pins, owner)` は、線や設定に対してチャネルをピンの表で持つ（Firmware/OepProbe: owner 7）。

## 0.0.16
- (EN) Firmware/OepProbe on the ESP32-P4 updates over its HS port alone: a USB DFU interface (EspUsbDeviceDfu, download mode, EP0 only, interface 4 after the others) takes `dfu-util -D OepProbe-esp32p4-<version>.bin`, writes the other app partition, checks it and restarts into it; the settings (NVS) stay. The new firmware is confirmed once the HS port has enumerated (verifyRollbackLater, markValid), so one that does not get that far goes back at the next reset. Releases attach each ESP32 Firmware build's app image (`<Example>-<profile>-<version>.bin`) too; firmware-<version>.json gives each file's kind (merged / app / uf2). Unverified on hardware.
- (JA) ESP32-P4 の Firmware/OepProbe は HS の口だけで更新できる: USB DFU のインターフェース（EspUsbDeviceDfu、download モード、EP0 だけ、ほかの後のインターフェース 4）が `dfu-util -D OepProbe-esp32p4-<version>.bin` を受け、もう一方の app の領域に書き、確かめて、そこから再起動する。設定（NVS）は残る。新しい firmware は HS の口が列挙されたら確定する（verifyRollbackLater、markValid）ので、そこまで進まない firmware は次のリセットで前に戻る。Release には ESP32 の Firmware のビルドの app の image（`<Example>-<profile>-<version>.bin`）も付け、firmware-<version>.json に各ファイルの種類（merged / app / uf2）を書く。実機では未確認。

## 0.0.15
- (EN) The capture-group's trigger (oep-if-capture §4.1): bound with trigger_track, that track waits for its own trigger and the others follow it - they run into their rings from the start, and when the trigger's time is known the group hands it to them, each cutting its segment around its sample nearest to it with its own pretrigger (a pretrigger with an immediate trigger is kept for this). The group's status and triggered event carry trigger_ns and trigger_fn, its state is waiting until then, and force goes to the trigger track. P4: the PARLIO logic and the analog follow and trigger; classic ESP32: the sampler triggers (it cannot follow: its search leaves gaps), the analog follows and triggers; RP2 analog follows up to 8192 values. A follower that cannot is refused at bind. Unverified on hardware.
- (JA) capture-group のトリガ（oep-if-capture §4.1）: trigger_track を付けて束ねると、そのトラックは自分のトリガを待ち、ほかのトラックはそれに従う。従うトラックは始めからリングに取り続け、トリガの時刻が分かると組がそれを渡し、各トラックはその時刻に最も近い自分のサンプルの周りで、自分の pretrigger で区画を切る（このため、即時のトリガでも pretrigger を覚えておく）。組の status と出来事 triggered は trigger_ns と trigger_fn を返し、それまで state は waiting。force はトリガのトラックへ送る。P4: PARLIO のロジックとアナログは、従うこともトリガになることもできる。classic ESP32: sampler はトリガになれる（探し方にすき間があるので従えない）。アナログは従うこともトリガになることもできる。RP2 のアナログは 8192 値まで従える。従えないトラックは bind で断る。実機では未確認。

## 0.0.14
- (EN) Classic ESP32 analog: the values come in order. The I2S DMA gives each pair of conversions the other way round (a 1 kHz square wave read one-channel had a 0 inside its high run, 0.0.13); drain takes them pairwise swapped. The 0.0.11-0.0.13 analog segments on the classic ESP32 have neighbouring values swapped.
- (JA) classic ESP32 のアナログ: 値が順に並ぶようにした。I2S の DMA は変換を 2 つずつ逆の順で渡す（1 チャネルで読んだ 1 kHz の矩形波の high の中に 0 が挟まった、0.0.13）。drain は 2 つずつ入れ替えて読む。0.0.11〜0.0.13 の classic ESP32 のアナログの区画は、隣どうしの値が入れ替わっている。
- (EN) The Firmware workflow builds every (example, profile) in its own job, all at once, with the platform's installation cached per version, and attaches the Firmware builds in a last job; no `--clean` (a fresh runner).
- (JA) Firmware のワークフローは、(example, profile) ごとに別のジョブで一度にビルドする。platform のインストールは版ごとにキャッシュし、Firmware のビルドは最後のジョブで Release に付ける。`--clean` はしない（毎回新しい runner）。

## 0.0.13
- (EN) Analog triggers (oep.fixture.analog): the ADC value of one channel crossed up (from below to at or above) or down, with a pretrigger and force. ESP32: the segment's buffer is the ring the driver's reads fill, looked at after every read, turned to order at the end; the pretrigger leaves 129 frames for a channel running ahead (up to 16255 at one channel), a pool overflow after the segment's start marks it slipped. RP2040 / RP2350: the DMA runs round a 32 KiB write ring (allocated on the first triggered configure), poll looks at it and copies the segment out; a triggered segment is at most 8192 values (half the ring), and a DMA that came round over its start marks it slipped. A stop while waiting or filling gives no segment. Unverified on hardware.
- (JA) アナログのトリガ（oep.fixture.analog）: 1 つのチャネルの ADC の値が上向き（下から、その値以上へ）または下向きに横切る。プリトリガと force。ESP32: 区画のバッファを、ドライバの読み出しが埋めるリングにし、読むたびに探し、最後に順に並べ替える。先に進むチャネルのために、プリトリガは 129 フレームを残す（1 チャネルで 16255 まで）。区画の始まりの後に置き場があふれたら slipped。RP2040 / RP2350: DMA を 32 KiB の書き込みのリング（最初のトリガ付きの configure で確保）で回し、poll で探して区画を写す。トリガ付きの区画は 8192 値まで（リングの半分）。DMA が区画の始まりまで回ってきたら slipped。待っている間や埋めている間の stop は区画を出さない。実機では未確認。

## 0.0.12
- (EN) Capture triggers (oep-if-capture §3.3): level and edge on one channel, one-shot, with a pretrigger and force; the triggered event carries trigger_ns and the segment its trigger_index. ESP32-P4 PARLIO: the repeat's DMA ring is searched (a byte whose samples cannot trigger is skipped whole), the segment copied out of it from pretrigger samples before the trigger (up to 64 KiB of ring back: 32768 samples at 16 channels). Classic ESP32 sampler: searched while sampling, in bursts of at most 250 ms with interrupts off (the watchdog's 300 ms), about 1 ms apart; the pretrigger is kept in each burst, so the segment is always contiguous. A capture-group refuses tracks with a trigger until the group trigger is in. Unverified on hardware.
- (JA) キャプチャのトリガ（oep-if-capture §3.3）: 1 つのチャネルのレベルとエッジ、ワンショット、プリトリガと force。出来事 triggered は trigger_ns を、区画は trigger_index を持つ。ESP32-P4 の PARLIO: リピートの DMA のリングを探し（トリガの立ちえないバイトは丸ごと飛ばす）、トリガの pretrigger サンプル前から区画に写す（リングを 64 KiB まで戻れる: 16 チャネルで 32768 サンプル）。classic ESP32 の sampler: サンプルしながら探す。割り込みを止めるのは 1 回 250 ms まで（watchdog は 300 ms）、間は約 1 ms。プリトリガはその区切りの中で取るので、区画はいつもつながっている。capture-group は、組のトリガができるまで、トリガを持つトラックを断る。実機では未確認。

## 0.0.11
- (EN) `oep.fixture.analog` (`AnalogCapture`, oep-if-capture §1.2 / §3.8): up to 4 ADC channels in turn, one-shot, immediate trigger, raw 12-bit values in 16-bit slots, with per-channel frontend (attenuation), scale and skew, the reference, rate_accuracy and the factory calibration as read. ESP32-P4: ADC1 GPIO16-23, 46 kHz in all (the P4 doubles values above), the first value's time corrected by the conversion frame the driver leaves out, pool overflows marked slipped. Classic ESP32: ADC1, 20-100 kHz in all, the eFuse ADC fields. RP2040 / RP2350: GP26-28 by FIFO + DMA, 500 kS/s in all. Unverified on hardware.
- (JA) `oep.fixture.analog`（`AnalogCapture`、oep-if-capture §1.2 / §3.8）: ADC の 4 チャネルまでを順に、ワンショット、即時のトリガ、16 ビットの枠に 12 ビットの生の値。チャネルごとの frontend（減衰）、scale、skew、基準電圧、rate_accuracy、読んだままの出荷時の較正を返す。ESP32-P4: ADC1 の GPIO16〜23、合計 46 kHz（それより上は値が 2 回ずつ出る）、ドライバが捨てる変換フレームの分を補正した最初の値の時刻、置き場があふれたら slipped。classic ESP32: ADC1、合計 20〜100 kHz、eFuse の ADC のフィールド。RP2040 / RP2350: GP26〜28 を FIFO + DMA で、合計 500 kS/s。実機では未確認。
- (EN) `oep.fixture.capture-group` (`CaptureGroup`, oep-if-capture §4): binds captures that implement `GroupTrack` (the P4's PARLIO, the classic ESP32's sampler, the analog) and starts them together; each track stamps its own first sample, so its offset is in its segment's start_ns. Firmware/OepProbe has the analog on every chip and the group on the ESP32s (after config: the other fns keep their numbers).
- (JA) `oep.fixture.capture-group`（`CaptureGroup`、oep-if-capture §4）: `GroupTrack` を持つ capture（P4 の PARLIO、classic ESP32 の sampler、アナログ）を束ね、一緒に始める。各トラックが自分の最初のサンプルの時刻を刻むので、ずれは区画の start_ns に出る。Firmware/OepProbe は、どのチップにもアナログを、ESP32 に組を持つ（config の後: ほかの fn の番号は変わらない）。

## 0.0.10
- (EN) Capture segments give their time in ns on the probe's clock with an uncertainty (oep-if-capture §2: `start_ns`, `start_uncertainty_ns`, 33 bytes): +-5 us on the P4's PARLIO (measured against the GPIO write), +-2 us on the classic ESP32's sampler. The P4's one-shot start reads the same clock as the other modes (esp_timer, not micros()).
- (JA) キャプチャの区画は、probe の時計の ns と不確かさで時刻を返す（oep-if-capture §2: `start_ns`、`start_uncertainty_ns`、33 byte）。P4 の PARLIO は ±5 µs（GPIO を書いた時刻と比べた実測）、classic ESP32 の sampler は ±2 µs。P4 のワンショットの開始は、ほかのモードと同じ時計（micros() ではなく esp_timer）を読む。
- (EN) oep.core's describe carries the MCU and its revision (`chip`, core §7.5) in the Firmware and LogicCapture sketches: `describeChip(w)` after `describeCore`.
- (JA) Firmware と LogicCapture のスケッチは、oep.core の describe に MCU とそのリビジョン（`chip`、core §7.5）を出す: `describeCore` の後に `describeChip(w)`。
- (EN) Guides in `docs/guide/` (English and Japanese): getting started (flash, find, use a probe from Python - every snippet checked against the fake probe), writing a probe (the endpoint, interfaces, TLV tails, the pin table and the plan, wires, binds and settings, pushes, USB identity, testing), boards (what each chip does, the released firmware's pins, building for another board). `04.Debug/RvswdDebugProbe` also builds for the ESP32-S3 (profile esp32s3, not checked on a bench).
- (JA) `docs/guide/` に手引きを置いた（英語と日本語）: 使い始める（焼く、見つける、Python から使う。例はすべて偽の probe で確かめた）、probe を書く（endpoint、インターフェース、TLV の後ろの部分、ピンの表と plan、線、bind と設定、push、USB の名乗り、試し方）、ボード（チップごとにできること、リリースされた firmware のピン、ほかのボード向けのビルド）。`04.Debug/RvswdDebugProbe` は ESP32-S3 でもビルドできる（profile esp32s3、ベンチでは未確認）。
- (EN) More examples: `03.Transports/MultipleTransports` (one endpoint on four USB transports), `04.Debug/RvswdDebugProbe` / `SwioDebugProbe` / `SwdDebugProbe`, `05.Capture/LogicCapture` (was Esp32P4CaptureProbe; now VID:PID 303a:0002, iProduct "OEP capture (ESP32-P4)", its test interface renamed io.github.open-embedded-probe.test-signal, `host/stream_test.py` on the released oep-client-python), `06.Settings/ProbeConfig`, `Tools/SwdPinSurvey` (was PicoDebugPortSurvey). The README lists every example with its boards.
- (JA) example を増やした: `03.Transports/MultipleTransports`（1 つの endpoint を 4 つの USB の経路で）、`04.Debug/RvswdDebugProbe` / `SwioDebugProbe` / `SwdDebugProbe`、`05.Capture/LogicCapture`（旧 Esp32P4CaptureProbe。VID:PID は 303a:0002、iProduct は "OEP capture (ESP32-P4)"、試験用のインターフェースの名前は io.github.open-embedded-probe.test-signal、`host/stream_test.py` はリリースされた oep-client-python を使う）、`06.Settings/ProbeConfig`、`Tools/SwdPinSurvey`（旧 PicoDebugPortSurvey）。README にすべての example とボードを載せた。

## 0.0.9
- (EN) `Esp32P4X035Probe` and `Esp32V003Probe` are gone: their jigs run `Firmware/OepProbe` (esp32p4 / esp32) with their settings, checked on the benches with 0.0.8.
- (JA) `Esp32P4X035Probe` と `Esp32V003Probe` を消した: その治具は `Firmware/OepProbe`（esp32p4 / esp32）と設定で動き、0.0.8 でベンチを通った。
- (EN) An attach without pins joins the wire's live connection on a wire whose pins the host chooses too (oep-if-debug §1): a host that did not name the slot's pair was refused unavailable (0.0.8, the X035 jig with an at-boot slot).
- (JA) host がピンを選ぶ線でも、pins の無い attach は、その線の生きている接続に乗る（oep-if-debug §1）。スロットの組を名指さない host が unavailable で断られていた（0.0.8、at boot のスロットがある X035 の治具）。

## 0.0.8
- (EN) Examples to learn from, for any RP2040 / RP2350 or classic ESP32 board: `01.Basics/MinimalProbe` (oep.core alone), `01.Basics/FixtureProbe` (gpio + uart, pins planned by the host), `02.Interfaces/CustomInterface` (your own interface under your own name: describe, the plan, ops, TLV tails). `refused()` and `platformRandom32()` are public for sketches.
- (JA) 学ぶための example（どの RP2040 / RP2350、classic ESP32 のボードでも）: `01.Basics/MinimalProbe`（oep.core だけ）、`01.Basics/FixtureProbe`（gpio + uart、ピンは host が plan で決める）、`02.Interfaces/CustomInterface`（自分の名前で自分のインターフェース: describe、plan、op、TLV の後ろの部分）。`refused()` と `platformRandom32()` をスケッチから使えるようにした。
- (EN) Releases attach only `examples/Firmware/`'s builds; every other example is built to check it.
- (JA) Release に付けるのは `examples/Firmware/` のビルドだけ。ほかの example は確かめるためにビルドする。
- (EN) `SwioPhy` takes any GPIO0-31 at run time (`begin(pin)`, `usePins` for host-chosen pins); `SwioPhy::kPin` is gone. The pin's mask is read once per transaction, so the bit timing is the same instructions as with the compile-time pin (one more instruction between bits, about 4 ns). Unverified on hardware.
- (JA) `SwioPhy` は実行中に GPIO0-31 のどれでも取る（`begin(pin)`、host が選ぶピンには `usePins`）。`SwioPhy::kPin` は無くなった。ピンのマスクは 1 回の転送ごとに 1 度だけ読むので、ビットの時間はコンパイル時のピンのときと同じ命令の並び（ビットの間に 1 命令、約 4 ns 増えるだけ）。実機では未確認。
- (EN) `Firmware/OepProbe` builds for the classic ESP32 too (profile esp32): a UART bridge; SWIO + riscv-dm + console, gpio, uart, capture (sampler), the SPI / I2C devices and probe.config, every pin chosen by the host.
- (JA) `Firmware/OepProbe` を classic ESP32 でもビルドできるようにした（profile esp32）: UART bridge。SWIO + riscv-dm + コンソール、gpio、uart、capture（sampler）、SPI / I2C デバイス、probe.config。ピンはすべて host が選ぶ。
- (EN) `Firmware/OepProbe` builds for the ESP32-P4 too (profile esp32p4): the four transports, RVSWD + riscv-dm + console, gpio, uart x2, capture, the SPI / I2C devices and probe.config, every pin chosen by the host; iProduct "OEP probe (ESP32-P4)" (VID:PID 303a:0002 and serial "<MAC>-hs" as before). Each chip's part is in its own header (Rp2.h, Esp32P4.h). Unverified on hardware.
- (JA) `Firmware/OepProbe` を ESP32-P4 でもビルドできるようにした（profile esp32p4）: 4 つの経路、RVSWD + riscv-dm + コンソール、gpio、uart x2、capture、SPI / I2C デバイス、probe.config。ピンはすべて host が選ぶ。iProduct は "OEP probe (ESP32-P4)"（VID:PID 303a:0002 と serial "<MAC>-hs" は今までどおり）。チップごとの部分は別のヘッダ（Rp2.h、Esp32P4.h）。実機では未確認。
- (EN) oep.probe.config on RP2040 / RP2350 too: the settings are saved in the flash's last sector (arduino-pico EEPROM). `Firmware/OepProbe` has it: slots on any pair, a bind of its CDC port, idle states.
- (JA) oep.probe.config が RP2040 / RP2350 でも使える: 設定は flash の最後の領域に保存する（arduino-pico の EEPROM）。`Firmware/OepProbe` に入れた: 任意の組のスロット、CDC の口の bind、空きのときの状態。
- (EN) Wires whose pins the host chooses (oep-if-debug §1): `DebugPort` / `SwdPort` `pin_choice` (the channels SWDIO / SWCLK may take, declared as role_channels). scan and attach take any allowed free pair (`RvswdPhy::usePins` moves the link: the P4's dedicated GPIO bundles are made again, the RP2 bit-bang set up again), a live connection holds its pins against plans, count-0 scans skip held pairs and go on with the skip TLV, and with the one seat taken only the live pair is tried. probe.config slots carry their own pair (up to 4 slots, several on one wire, one at boot). Fixed-pair sketches work as before; the fixed-pair helpers are gone.
- (JA) host がピンを選ぶ wire（oep-if-debug §1）: `DebugPort` / `SwdPort` の `pin_choice`（SWDIO / SWCLK に取れる channel、role_channels で宣言）。scan と attach は、許された空いている組ならどれでも受ける（`RvswdPhy::usePins` が線を動かす: P4 は dedicated GPIO の束を作り直し、RP2 は bit-bang を設定し直す）。生きている接続はピンを plan から守り、count = 0 の scan は持たれている組を飛ばして skip の TLV で続け、席が埋まっていれば生きている組だけを試す。probe.config のスロットは自分の組を持つ（4 個まで、1 本の wire に複数、at boot は 1 つ）。固定の組のスケッチは今までどおり。固定の組の補助関数は無くなった。
- (EN) New `examples/Firmware/OepProbe`: one firmware for any RP2040 / RP2350 board (profiles rp2040 / rp2350), every pin chosen by the host - RVSWD + riscv-dm + console, SWD + arm-adi, gpio, uart - with iProduct "OEP probe (RP2040)" / "(RP2350)". Unverified on hardware.
- (JA) 新しい `examples/Firmware/OepProbe`: どの RP2040 / RP2350 のボードにも焼ける 1 本の firmware（profile rp2040 / rp2350）。ピンはすべて host が選ぶ（RVSWD + riscv-dm + コンソール、SWD + arm-adi、gpio、uart）。iProduct は "OEP probe (RP2040)" / "(RP2350)"。実機では未確認。
- (EN) The Firmware workflow builds every profile of every example (examples/**) and names the files `<Example>-<profile>-<version>.uf2` / `.merged.bin`.
- (JA) Firmware の workflow は、すべての example（examples/**）のすべての profile をビルドし、`<Example>-<profile>-<version>.uf2` / `.merged.bin` と名付ける。

## 0.0.7
- (EN) Every source file carries its license (`SPDX-License-Identifier: MIT` and the copyright line); `tools/sync_registry.sh` puts them on the copied registry header too. Stale header comments fixed (the frame formats in OepFrame.h, the RP sketches' transport).
- (JA) すべてのソースファイルにライセンスの表記（`SPDX-License-Identifier: MIT` と著作権の行）を入れた。`tools/sync_registry.sh` は写した registry のヘッダにも付ける。古くなった冒頭の説明を直した（OepFrame.h のフレームの形、RP のスケッチの経路）。
- (EN) riscv-dm puts back the GPRs its block / word ops use (s0, s1, a0, a1) before the hart runs again (oep-if-debug §4.5); the describe no longer lists clobbers. A sketch stopped over and over by halt -> read_block -> resume died when they were left changed.
- (JA) riscv-dm は、block / 語の op が使う GPR（s0、s1、a0、a1）を、hart を走らせる前に元に戻す（oep-if-debug §4.5）。describe の clobbers は無くなった。halt → read_block → resume を何度も挟まれた sketch は、それらが変わったままで死んでいた。

## 0.0.6
- (EN) riscv-dm keeps the target's DATA0 / DATA1 from the moment the hart stops and writes them back before resume / step (oep-if-debug §4.2): after another client's halt -> read_block -> resume, a dmseq console no longer goes quiet for seconds (the abstract commands had wiped the target's frame).
- (JA) riscv-dm は hart が止まったときの target の DATA0 / DATA1 を覚え、resume / step の前に書き戻す（oep-if-debug §4.2）: 別の client の halt → read_block → resume の後に、dmseq のコンソールが数秒黙らなくなった（abstract command が target のフレームを消していた）。
- (EN) No default reset line (oep-if-debug §3): attach_under_reset takes only the channel the host names - one the wire declares (describe role_channels, role reset) and no other interface holds. `DebugPort::reset_default` is gone; a sketch sets `reset_allowed` and `pins`.
- (JA) 既定の reset 線は無くなった（oep-if-debug §3）: attach_under_reset は host が名指した channel だけを受ける。wire が宣言したもの（describe の role_channels の role reset）で、ほかのインターフェースが持っていないものに限る。`DebugPort::reset_default` は無くなり、スケッチは `reset_allowed` と `pins` を設定する。
- (EN) The line's settings are the target's and come from the host: rvswd's attach / attach_under_reset take idle_clock (SWCLK low while resting), and a probe.config slot carries max_speed / idle_clock for the probe's own attach. `RvswdPhy::setMinHalfNs` is gone, and Rp2350L103Probe no longer rests the line low or floors its speed by itself. The slot item's layout changed; the settings are saved under a new NVS key, so saved settings from earlier versions are not loaded (set them again).
- (JA) 線の設定は target のもので、host から来る: rvswd の attach / attach_under_reset は idle_clock（休ませる間の SWCLK を low に）を受け、probe.config のスロットは probe が自分で attach するための max_speed / idle_clock を持つ。`RvswdPhy::setMinHalfNs` は無くなり、Rp2350L103Probe は自分で線を low で休ませたり速さに下限を付けたりしない。スロットの項目の形が変わった。設定は新しい NVS の key に保存するので、前の版で保存した設定は読まれない（設定し直す）。
- (EN) The plan holds 64 role assignments (was 16), every fn together, and says so in oep.core's describe (plan_roles 0x4B). More than that is rejected unavailable, not malformed (oep-core §8).
- (JA) plan が持てる role_assignment は 64 個（前は 16）。すべての fn の合計で、oep.core の describe で宣言する（plan_roles 0x4B）。それを超えると malformed ではなく rejected unavailable（oep-core §8）。
- (EN) A plan the settings put in (probe.config) belongs to the settings (oep-core §8): plan_release leaves it (n = 0 included) and plan_apply naming its fn is rejected unavailable.
- (JA) 設定（probe.config）が入れた plan は設定のもの（oep-core §8）: plan_release はそれを解かず（n = 0 でも）、その fn を挙げた plan_apply は rejected unavailable。

## 0.0.5
- (EN) The firmware string in oep.core's describe is the library's release version (it was a fixed `3.2.0-v1rc`). Every Release now gets each example's built firmware attached (`<Example>-<version>.merged.bin` for ESP32, flash at 0x0; `.uf2` for RP2040 / RP2350; `firmware-<version>.json` with the sha256), by a separate Firmware workflow after the shared Release workflow.
- (JA) oep.core の describe の firmware の文字列は、ライブラリのリリースの版になった（前は固定の `3.2.0-v1rc`）。リリースごとに、example ごとのビルド済みの firmware が付く（ESP32 は `<Example>-<version>.merged.bin`、0x0 に書く。RP2040 / RP2350 は `.uf2`。sha256 は `firmware-<version>.json`）。共通の Release の後に、別の Firmware の workflow が作る。

## 0.0.4
- (EN) P4 HS sketches receive vendor bulk OUT a packet at a time (`CFG_TUD_VENDOR_RX_NEED_ZLP=0`): a request ending on a 512-byte boundary (+ the host's ZLP) was held until the next OUT, 3 s without an answer.
- (JA) P4 の HS のスケッチは vendor bulk の OUT を packet ごとに受ける（`CFG_TUD_VENDOR_RX_NEED_ZLP=0`）。512 byte の境目で終わる要求（+ host の ZLP）が次の OUT まで抱えられ、3 s 答えなかった。
- (EN) The P4 loop no longer stops for seconds when a USB reader goes away: the HS CDC port reports its real FIFO room (raw bytes never wait; an open port nobody read took 200 ms per 64-byte chunk), and once a vendor bulk frame or a HID report could not go, later ones do not wait again until one goes.
- (JA) USB の読み手が居なくなったときに P4 の loop が数秒止まらないようにした: HS の CDC は FIFO の本当の空きを返す（生のバイトは待たない。開いたまま読まれない口では 64 byte ごとに 200 ms 待っていた）。vendor bulk のフレームや HID の report が一度出せなかったら、次に出せるまで待たない。
- (EN) Vendor bulk (DirectBulkStream): a result frame goes into one buffer whole or is dropped whole (`dropped()`), after waiting up to 2 s for the host to take IN - it used to drop the rest of a frame after 200 ms, which broke the host's framing (a 3 s timeout, twice in 25 writes over usbip).
- (JA) vendor bulk（DirectBulkStream）: 結果のフレームは 1 つの buffer に丸ごと入れるか、丸ごと捨てる（`dropped()`）。host が IN を取るのを最大 2 s 待つ。前は 200 ms でフレームの残りを捨てて host のフレームを壊していた（usbip 越しの 25 回の書き込みで 2 回、3 s の時間切れ）。

## 0.0.3
- (EN) The console reads again once DMSTATUS shows the hart running after raw DMI writes (a debugger resuming it through dmcontrol), instead of waiting for the probe's own resume (oep-if-console §2).
- (JA) raw の DMI の書き込みの後でも、DMSTATUS で hart が走っていればコンソールの読みを戻す（debugger が dmcontrol で走らせたとき）。probe 自身の resume を待たない（oep-if-console §2）。
- (EN) A read from the last mark of a kind that is not kept starts now, not at the oldest byte (oep-if-common §1.2): a fixture UART, which has no reset marks, gave its old output again at every open.
- (JA) その kind のマークが残っていないときの「最後のマークから」の read は、一番古い位置ではなく今から（oep-if-common §1.2）。reset のマークを持たない fixture UART では、開くたびに古い出力が繰り返されていた。

## 0.0.2
- (EN) A serial port no longer hands a lone 0x00 to its bind: the closing 0x00 of every OEP frame opened an empty candidate, and after 200 ms without input it went to the bound console as a raw byte (a stray 0x00 on the target after the first answer, a monitor reopened, a flash).
- (JA) シリアルの口が、中身の無い 0x00 を bind に渡さなくなった。OEP のフレームの閉じの 0x00 が空の候補を開き、200 ms 入力が無いと生のバイトとして bind のコンソールへ行っていた（最初の答えの後、monitor の開き直し、flash の後に target に 0x00 が届いていた）。

## 0.0.1
- (EN) First release of the OEP v1 probe library and firmware examples (oep-spec 2026-09-29): serial ports share OEP frames (0x00 <COBS> 0x00) and raw bytes, binds (last-reset / manual / mixed), oep.probe.config with slots and NVS storage (ESP32), the transport list in describe, the lock's owner. Examples: ESP32-P4 + CH32X035 (vendor bulk, USB-Serial/JTAG, HID, CDC), classic ESP32 + CH32V003 (UART bridge), RP2350 + CH32L103, RP2040 Zero, the P4 logic capture probe.
- (JA) OEP v1 の probe のライブラリと firmware の example の最初のリリース（oep-spec 2026-09-29）: シリアルの口で OEP のフレーム（0x00 <COBS> 0x00）と生のバイトを共用、bind（last-reset / manual / mixed）、スロットと NVS の保存を持つ oep.probe.config（ESP32）、describe の経路の一覧、ロックの持ち主（owner）。example: ESP32-P4 + CH32X035（vendor bulk、USB-Serial/JTAG、HID、CDC）、classic ESP32 + CH32V003（UART bridge）、RP2350 + CH32L103、RP2040 Zero、P4 のロジックキャプチャの probe。
