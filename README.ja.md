# OEP Arduino probe prototype

破壊的変更を前提とするArduino向けOEP probe実験です。公開protocol、互換libraryまたは製品用
firmwareではありません。

現在のP0は仮UART frame、endpoint confirmation、offered function一覧だけを実装します。
V003 SWIO handlerとUIAPduino fixtureは次段で接続します。

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
