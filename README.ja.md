# OEP Arduino probe prototype

破壊的変更を前提とするArduino向けOEP probe実験です。公開protocol、互換libraryまたは製品用
firmwareではありません。

現在のP0は仮UART frame、endpoint confirmation、offered function一覧だけを実装します。
V003 SWIO handlerとUIAPduino fixtureは次段で接続します。

`examples/Esp32V003Prototype`は無印ESP32向けです。hostとのUARTは`Serial`を使用し、起動時の
ASCII bannerを出さずbinary frameだけを送受信します。
