# OEP Arduino probe prototype

破壊的変更を前提とするArduino向けOEP probe実験です。公開protocol、互換libraryまたは製品用
firmwareではありません。

`examples/Esp32P4X035Prototype`はESP32-P4とCH32X035の試作fixture向けである。既知の配線
`GPIO2→PC18/SWDIO`、`GPIO54→PC19/SWCLK`を使い、TargetControl、TargetMemory、TargetFlashを
RVSWD上へ実装する。

ArduinoCore-CH32からbinを書き込むための配線、probe build、全flash退避、差分program、verify、
復旧手順は[`docs/esp32-p4-x035-usage.ja.md`](docs/esp32-p4-x035-usage.ja.md)にまとめた。

現在のP1は仮UART frame、endpoint confirmation、offered function一覧とV003 TargetControlを
実装します。SWDIOで状態取得、user mode正規化、製品bootloader移行を行います。

`examples/Esp32V003Prototype`は無印ESP32向けです。hostとのUARTは`Serial`を使用し、起動時の
ASCII bannerを出さずbinary frameだけを送受信します。

## 現在の実機結果

2026-09-20、ESP32-P4 revision 1.3 `30:ed:a0:e3:11:08`とCH32X035C8T6のfixtureで、OEP
endpoint confirmation、3機能の列挙、target status、flash先頭readを確認した。X035の物理erase
pageは256 byte、現在のOEP prototypeの論理program単位は64 byteなので、backendは256 byteを
read-modify-erase-programし、同一物理page内の残り192 byteを保持する。

62 KiB flashの末尾物理page `0x0800f700..0x0800f7ff`を退避し、論理page
`0x0800f7c0`へOEP `TargetFlash.programPage64`だけでpatternを書いた。独立したTargetMemory readで
pattern 64 byteの一致と隣接192 byteの不変を確認し、同じOEP経路で元の全FFへ戻した。退避前後の
256 byte SHA-256はともに
`3d6876a0146de8576eb2395a858de1213d1b92c65b779df3a331cfd5a4584546`だった。その後software reset、
再attach、先頭word readも成功した。これは一台・低速bit-bangの破壊前提試験であり、性能、電源断、
複数個体および全image書込みの保証ではない。

同じfixtureで64-byte pattern書込みと全FF復元を10周期、合計20操作実行し、2周期ごとのsoftware
reset後照合を含め全件成功した。現在の62 KiB imageをOEP TargetMemoryだけで退避した後、1,032
byteのPA0 HIGHテストアプリへ差分109論理pageを書換えた。reset後に`GPIOA_OUTDR=1`で実行を確認し、
OEPで62 KiB全域を読み直した結果、期待imageとの不一致は0、SHA-256は
`9f472e4b9d2f12634e90eb0d5861eb7addca1d2f5ff5f755dc3e42fb20eb04a5`だった。退避imageへ同じ109
pageを復元し、reset後の全域SHA-256が退避時と同じ
`17ad3777ba42af0bd8d61ae5521ab5a4d5f10057e148d22b3fae5b8fbc235988`であることも確認した。
初期実装では全域read約240秒、109 pageの差分program約298秒だった。P4 GPIO処理とread chunkを
改善した2026-09-20実測では全域read 24.80秒、比較・109 page program・全域verifyの合計80.00秒。

X035 backendはerase前の256 byteをprobe RAMへ保持する。同一物理pageの再要求はその退避像を使い、
未回復中の別page要求は診断`0xe0`で失敗させる。試験buildの
`OEP_X035_INJECT_FLASH_FAILURE=1/2`により、物理erase直後（`0xe1`）と最初の64-byte commit直後
（`0xe2`）を一度だけ中断した。targetが全消去または部分書込みになったことを独立readで確認後、
同一要求の再送で隣接192 byteを含む256 byte全体を回復できた。

この回復cacheはprobeのresetや電源断を跨がない。probeも同時に状態を失う障害から確実に回復する
には、hostが256 byte物理page全体を再送できる複数page/streaming操作、または永続journalが必要で
あり、現在の64-byte操作だけで原子的保持を保証してはならない。

2026-09-19、ESP32-D0WD-V3 `00:70:07:0d:93:94`へ書き込み、Python prototype clientから
endpoint confirmationと7個のoffered function取得に成功しました。

UIAPduino fixtureのESP32 GPIO2はV003 `PC6/MOSI`へ接続されていました。GPIO2はESP32のboot
strapでもあり、この配線を接続した状態ではesptoolが`boot mode 0xa`となってdownload modeへ
入れませんでした。GPIO2を外すと自動reset、flash erase、write、verifyが成立しました。

2026-09-19にstrap pinを避けて配線し直し、専用pin-map sketchで3周再測定した。SPI配線は
`PC5/SCK→GPIO27`、`PC6/MOSI→GPIO4`、`PC7/MISO→GPIO14`である。現在のSPI fixtureは
CSにGPIO19を使用する。
GPIO2、GPIO12、GPIO15は使用しない。

2026-09-20、E129で検証したSWDIO PHYとCPU payload方式を縮約してTargetControl backendへ
接続した。GPIO16のSWDIOだけを使用し、GPIO23の外部RESETは使用しない。OEP clientから
状態取得、user mode正規化、状態再取得、製品bootloader移行を順に実行でき、移行後に
Windows側で`1209:b803`が再列挙した。提供一覧は実装済みのTargetControl 1件だけに変更した。

状態取得は現在attachとhaltを伴う。`flags=3`はこの操作によってattachedかつhaltedになったことを
示す仮値であり、観測だけの操作ではない。この副作用と、操作後にresumeすべきかは今後の
service意味を決めるための検討事項である。

同日、TargetMemory `0x0102`を追加した。現在は4 byte aligned、4～88 byteのbounded readである。
V003 flash `0x08000000`から16 byteをOEP request/resultだけで取得できた。このalignmentと長さは
prototype実装の制約であり、仕様上の上限ではない。

同日、TargetFlash `0x0103`へV003の64-byte page erase/program/verifyを追加した。論理messageへ
addressと64-byte dataを一度に載せるため、prototypeのmaximum messageを64から96 byteへ変更した。
これは将来のHID report sizeを96 byteにする決定ではない。64-byte以下のtransport packetでは
bindingが分割・再結合し、共通protocolへ一つの論理messageとして渡す前提を検証する値である。

実機の`0x08003fc0`へ64-byte patternを書き、flash backend内verifyと、独立したTargetMemory
request 2回によるread-backの両方を通した。最初の独立readではSWDIOがerrorを返さず古い
`DATA0`を返す場合が見つかったため、memory readは同じ値が2回連続するまで確定しないようにした。
範囲外`0x07000000`への要求はflash操作を開始せずrejectedとなることも確認した。

同日、FixtureGpio `0x0201`のread-only prototypeを追加した。board wiringで許可した14 pinだけを
入力として読み、strap pinやSWDIO/RESETは対象にしない。実機で全14 pinを20秒観測し、GPIO14の
LOW/HIGH変化とI2C/UARTのidle HIGHをOEP resultとして取得した。V003上の現imageはpin-map専用の
周期波形ではなかったため、既知のDOUT commandと組み合わせたcontrolled testは次段とする。

同日、FixtureUart `0x0202`へ`configure`、`write bytes`、`read available`を追加した。UART instanceは
ESP32 RX=22/TX=21に固定し、baudrateだけをhostが要求してprobeがactual値を返す。実機で
115200 bpsを構成し、OEP write/readだけでV003の`PING`へ`PONG`を取得した。現在のV003 imageは
`DOUT`等を`ERROR command`として返したため、UART機能の成功とpeer applicationのoperation
非対応を区別して観測できた。

全13,780 byte imageのdirect flashでは、backend内verifyの偽陰性と真の部分書込みを区別する必要が
生じた。failure resultへ非規定の段階診断byteを追加し、既にpage全体が一致する場合はeraseを省略
するようにした。offset `0x700`のpageでは複数回の再実行後も一部wordが`FF`のまま残る事象があり、
全image復旧は未完了である。E131との差分を解消するまで自動retryだけで成功扱いにしない。

製品HIDで最新版fixtureを復旧した後、FixtureUartの`DOUT`とFixtureGpioを組み合わせ、target pin
7→GPIO27とtarget pin 9→GPIO14のLOW/HIGH/LOWを確認した。初回には以前のHIGHが入力へ残る場合が
あったため、試験はLOWへ正規化してからHIGH/LOWを判定する。

追加のmulti-page試験で、cycle-sensitiveなPHY hot pathのGPIO maskをruntime変数にしたことが
E129との重大な差だと判明した。GPIO16 maskを定数へ戻すと大量のDMI read failureは解消した。
一方、flash書込み直後の同一debug sessionでは成功に見え、後のrequestで部分書込みが判明する
caseと、flash操作後にsoftware/external resetでbootへ移行できない状態を観測した。電源再投入前の
安全な回復条件が未確定なため、ESP32 exampleはTargetFlashをoffered functionから一時的に外した。
backend実験コードは比較用に残すが、現在は利用可能な機能として公開しない。

TargetFlash停止後のfirmwareをESP32へ戻し、offered functionがTargetControl、TargetMemory、
FixtureGpio、FixtureUartの4件だけであることを実機確認した。TargetMemoryの連続readでは一部要求が
`completed/failed`となり、再試行で取得できた。失敗を成功へ変換してはいないが、SWDIO backendは
まだ連続操作の安定性を保証しない。またstatus/readはtargetをhaltするため、試験終了時は
`normalize-user`で通常実行へ戻す必要がある。

RMTをGPIO16へ重ねたE133では、12.5 ns単位でsoftware SWIOの実波形を取得できた。係数8〜10の
DMCFGR readは各10/10、DATA1 writeも係数8による独立検証で各100/100だったが、係数8では
abstract memory/flash sequenceが成立しなかった。単発DMI成功率や「遅いtiming」だけではflash用
PHY条件を決められない。page全体再試行とfresh attach後verifyをbackendへ追加したが、reset後の
永続一致は未確立なのでTargetFlashのoffered function停止は維持する。

その後、LinkEの50 MHz実波形とUSB記録を再解析したE134で、LinkEはflash controllerをhostから
逐次操作せず、498 byteのV003用loaderをtarget RAMへ置いて実行していることを確認した。通常の
DMI frame間隔は中央値6.70 us、95%点7.44 us、DMSTATUS poll間隔は中央値9.20 usだった。
backendも同じくloaderを`0x20000000`、dataを`0x20000200`へ配置し、a0/a1/a2、sp、dpcを設定して
target CPUへerase/program/verifyを実行させる方式へ変更した。

最初の実行では書込み自体は64 byte完全一致したものの、完了pollを取りこぼしてfailureとなった。
fresh attachを完了fenceとして追加するとsuccessとなった。さらにLinkEの240/860 nsへ最も近く、
E133でread 100/100だった係数8（実測262.5/862.5 ns）へPHYを変更した。異なるpattern 5個を
`0x08003fc0`へ連続してerase/programし、各回`normalize-user`によるsoftware reset後、別々の
TargetMemory request 2件で64 byte完全一致した。これに先行する1 patternを含め6回連続成功したため、
ESP32 prototypeはTargetFlashを再びoffered functionへ含める。ただしこれはV003一台の破壊前提試験で、
電源再投入、全image、複数個体による安定性確認は未完了である。

E136ではhostがFLASH register操作を逐次実行する経路も係数8で再測定した。追加quiet time 0でも
pattern/全FF復元が成功し、eraseは約3.3 ms、programは約2.9 ms、STATR poll 1回は約0.46 msだった。
backendへこの経路を戻し、`OEP_V003_FORCE_SEQUENTIAL_FLASH=1`でloaderを使わないbuildを作れる。
強制逐次buildで末尾4 pageへpatternを書いて全FFへ戻す計8回がすべて成功し、software reset後も
4 page全体が一致した。通常buildはRAM loaderを優先し、失敗時だけpage全消去から逐次経路を
fallbackとして実行する。固定delayではなくSTATR.BUSY clearを確認し、通信read失敗とは区別する。

loader fallbackには3段階の故障注入も行った。loader転送前、loader/dataのRAM転送後、loaderが
flash書込みを完了した後のすべてで、逐次fallbackによるpattern書込み、全FF復元、software reset後
read-backが成功した。最後の条件では同じpageを再度erase/programするため、部分完了状態からの
回復も確認している。成功resultから使用経路を識別するtelemetryはProtocolへ混ぜず今後の課題とする。

通常loader経路の短期反復はpattern/全FFを50周期、合計100回実行して100/100成功した。10周期ごとの
software reset後照合も5/5一致した。完了pollを20 ms/128回で打ち切ってfresh attachへ移ることで、
1 page中央値は422 msから約296 msへ短縮した。64 byteごとのloader再転送が支配的であり、暗黙の
RAM常駐cacheは採用しない。将来の高速化は明示的な複数page operationで償却する。
