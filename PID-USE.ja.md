# OEP の USB の ID

[English](PID-USE.md)

参照の firmware は、今は **仮の USB の ID** で動いている。仮の ID はボードの既定の VID:PID（ESP32-P4 では arduino-esp32 の
TinyUSB の既定の `303a:0002`）で、iProduct は `OEP` で始まる。この仮の ID は**配布には使えない**。これに頼る製品や
firmware を出さないこと。

専用の PID を取得できたら、参照の firmware はそれに切り替える予定である。そのとき、使ってよい条件をこの文書に書く。

host は USB の ID に依存しない。iProduct が `OEP` で始まることで OEP の probe を見つけ、何ができるかは probe 自身から読む
（confirm、list、describe。oep-spec の core のとおり）。今、配布のために USB の ID が要る firmware は、自分の ID を使うこと。
