# Changelog / 変更履歴

## Unreleased
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
