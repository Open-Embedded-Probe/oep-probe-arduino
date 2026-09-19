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
