# OEP Arduino probe prototype

破壊的変更を前提とするArduino向けOEP probe実験です。公開protocol、互換libraryまたは製品用
firmwareではありません。

現在のP1は仮UART frame、endpoint confirmation、offered function一覧とV003 TargetControlを
実装します。SWDIOで状態取得、user mode正規化、製品bootloader移行を行います。

`examples/Esp32V003Prototype`は無印ESP32向けです。hostとのUARTは`Serial`を使用し、起動時の
ASCII bannerを出さずbinary frameだけを送受信します。

## 現在の実機結果

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

同日、TargetMemory `0x0102`を追加し、4 byte aligned、4～32 byteのbounded readを実装した。
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
