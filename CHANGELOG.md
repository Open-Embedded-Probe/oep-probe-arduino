# Changelog / 変更履歴

## Unreleased
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
