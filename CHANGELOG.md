# Changelog / 変更履歴

## Unreleased

## 0.0.2
- (EN) A serial port no longer hands a lone 0x00 to its bind: the closing 0x00 of every OEP frame opened an empty candidate, and after 200 ms without input it went to the bound console as a raw byte (a stray 0x00 on the target after the first answer, a monitor reopened, a flash).
- (JA) シリアルの口が、中身の無い 0x00 を bind に渡さなくなった。OEP のフレームの閉じの 0x00 が空の候補を開き、200 ms 入力が無いと生のバイトとして bind のコンソールへ行っていた（最初の答えの後、monitor の開き直し、flash の後に target に 0x00 が届いていた）。

## 0.0.1
- (EN) First release of the OEP v1 probe library and firmware examples (oep-spec 2026-09-29): serial ports share OEP frames (0x00 <COBS> 0x00) and raw bytes, binds (last-reset / manual / mixed), oep.probe.config with slots and NVS storage (ESP32), the transport list in describe, the lock's owner. Examples: ESP32-P4 + CH32X035 (vendor bulk, USB-Serial/JTAG, HID, CDC), classic ESP32 + CH32V003 (UART bridge), RP2350 + CH32L103, RP2040 Zero, the P4 logic capture probe.
- (JA) OEP v1 の probe のライブラリと firmware の example の最初のリリース（oep-spec 2026-09-29）: シリアルの口で OEP のフレーム（0x00 <COBS> 0x00）と生のバイトを共用、bind（last-reset / manual / mixed）、スロットと NVS の保存を持つ oep.probe.config（ESP32）、describe の経路の一覧、ロックの持ち主（owner）。example: ESP32-P4 + CH32X035（vendor bulk、USB-Serial/JTAG、HID、CDC）、classic ESP32 + CH32V003（UART bridge）、RP2350 + CH32L103、RP2040 Zero、P4 のロジックキャプチャの probe。
