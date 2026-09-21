# ESP32-P4 / CH32X035 OEP prototype利用手順

状態: 破壊的変更を前提とする実験用手順。ArduinoCore-CH32から暫定probeを利用するための現時点の
再現方法であり、公開Protocolや安定CLIではない。

## 対象

- probe: ESP32-P4
- target: CH32X035C8T6（62 KiB flash）
- OEP transport: ESP32-P4のUSB Serial/JTAG CDC、115200 bps
- target transport: RVSWD software bit-bang

既知fixtureの必須配線は次の2本と共通GNDである。

| ESP32-P4 | CH32X035 | 用途 |
|---:|---|---|
| GPIO2 | PC18 | SWDIO |
| GPIO54 | PC19 | SWCLK |

PC15は開発board上でP4 GPIO15とGPIO45の2本へ出ているが、書込みには使用しない。targetは別途給電
する。P4とtargetの信号電圧は3.3 Vを前提とする。

## probe firmware

`oep-probe-arduino`のrootで実行する。

```sh
arduino-cli compile \
  --fqbn 'esp32:esp32:esp32p4:USBMode=hwcdc,CDCOnBoot=cdc' \
  --library . \
  --output-dir /tmp/oep-p4-x035 \
  examples/Esp32P4X035Prototype

arduino-cli upload \
  --port /run/board-identify/by-id/esp32-series-30eda0e31108 \
  --fqbn 'esp32:esp32:esp32p4:USBMode=hwcdc,CDCOnBoot=cdc' \
  --input-dir /tmp/oep-p4-x035 \
  examples/Esp32P4X035Prototype
```

この環境では列挙順で変化する`/dev/ttyACM*`を直接使わず、次の固定名を使用する。

```text
/run/board-identify/by-id/esp32-series-30eda0e31108
```

起動bannerは出さず、CDC stream全体をOEP binary frameに使用する。serial monitorを同時に開かない。

## ArduinoCore-CH32のbinを作る

ArduinoCore-CH32の開発coreがArduino CLIへ`ch32-riscv-ug`として導入済みの例:

```sh
arduino-cli compile \
  --fqbn 'ch32-riscv-ug:ch32v:CH32X035:pnum=CH32X035C8T6' \
  --output-dir /tmp/ch32x035-build \
  path/to/sketch
```

書込み対象は`/tmp/ch32x035-build/<sketch>.ino.bin`のraw binaryである。ELFやIntel HEXを渡さない。

## 退避、書込み、検証

`oep-client-python`のrootで実行する。初回は必ず現在の全flashを退避する。

```sh
uv run python -m oep_client \
  --port /run/board-identify/by-id/esp32-series-30eda0e31108 \
  --backup-flash /tmp/ch32x035-original.bin
```

raw binを差分書込みし、software reset後に全域verifyする。

```sh
uv run python -m oep_client \
  --port /run/board-identify/by-id/esp32-series-30eda0e31108 \
  --program-image /tmp/ch32x035-build/<sketch>.ino.bin \
  --destructive
```

書込み済みimageのverifyのみ:

```sh
uv run python -m oep_client \
  --port /run/board-identify/by-id/esp32-series-30eda0e31108 \
  --verify-image /tmp/ch32x035-build/<sketch>.ino.bin
```

CI/soakでwall timeを機械可読に残すには`--result-json`を併用する。成功時だけ指定fileへ原子的でない
JSONを書き、`verify`/`verify_reset`またはprogramの各phaseを秒で記録する。2026-09-21のPWM image
実機verifyでは`{"verify": 5.025535, "verify_reset": 0.00343}`だった。転送byte数・attach・page別の
詳細telemetryはまだP0.2の未実装項目である。

```sh
uv run python -m oep_client \
  --port /run/board-identify/by-id/esp32-series-30eda0e31108 \
  --verify-image /tmp/ch32x035-build/<sketch>.ino.bin \
  --result-json /tmp/x035-verify.json
```

元imageへ戻す場合、退避ファイルは既に63,488 byteなのでそのまま指定できる。

```sh
uv run python -m oep_client \
  --port /run/board-identify/by-id/esp32-series-30eda0e31108 \
  --program-image /tmp/ch32x035-original.bin \
  --destructive
```

CLIは入力binを63,488 byteまで`0xff`で埋め、target全域を先に読み、異なる64-byte論理pageだけを
programする。最後にresetし、全域を読み直してbyte単位で照合し、memory readによるhaltを解除する
ためもう一度resetする。backupとverify-onlyも終了時にtargetを通常実行へ戻す。

## 結果の見方

正常終了例では最後に次を出す。

```text
program-image pages=109 attempts=109 sha256=<63,488-byte image hash>
```

`pages=0`は既に同じimageだったことを示す。`attempts`が`pages`より多い場合はoperation failure後に
同一pageを再送して回復した。processの終了codeが非zero、`flash program failed`、`verify mismatch`
のいずれかが出た場合は成功扱いにしない。

## 制約と復旧

- X035の物理erase単位256 byteをprobe内部のread-modify-erase-programで吸収する。
- 書込み後の`normalize-user`は単なるdebug resumeではなく、RAM上の短いpayloadからPFIC system resetを
  発行する。これにより旧imageの停止PCではなく、新imageのreset vectorから開始する。
- operation途中で失敗した場合、probe RAMのerase前imageを使って同じ64-byte要求の再送から回復する。
- 未回復中の別物理page要求は診断`0xe0`で拒否する。
- erase後にprobeもresetまたは電源断すると回復cacheは失われる。この場合、退避済みの完全imageを
  `--program-image`で再送する。部分imageや別pageから書込みを続行しない。
- host processがkillされて`normalize-user`を送れない場合にも、P4 firmwareは最後の有効OEP request
  から1.5秒無通信でRVSWD sessionをsystem reset/releaseする。これはhost中断時の安全cleanupであり、
  1.5秒を超える一つのOEP requestを許容するタイムアウトではない。長時間のprobe operationを追加する
  場合は、operation中のkeepaliveまたは明示的なbusy状態を先に設計する。
- targetのflash/option保護、型番、容量を自動識別していない。CH32X035以外へ既定値のまま使わない。
- 2026-09-20の初期実装では全域read/verify約240秒、109 pageの差分書込み約298秒だった。
  SWDIOの不要な方向切替を除去し、追加half-periodを0にし、1要求を32から88 byteへ拡張した後は、
  全域read 24.80秒、109 pageの比較・書込み・全域verifyを含む復元全体80.00秒を実測した。
  続いて同一 OEP 接続の連続 read/program request 間でRVSWD attach/halt sessionを再利用し、
  このX035 fixtureでのみ post-frame guardを0 µsにした結果、62 KiB full verifyは
  20.972/21.005/21.044秒（平均21.007秒、全域hash一致3/3）となった。別配線では保守的な
  20 µs defaultを使い、同等のfull-image検証が通るまで0 µsを選ばない。
- さらに`DMABSTRACTAUTO`とDMDATA1に保持したtarget-side addressを使う連続readを導入した。
  program buffer、register、addressを4 byteごとに再設定せず、DMDATA0 readで次wordを起動する。
  62 KiB full verifyは4.896/4.959/4.961秒（平均4.939秒、全域hash一致3/3）となった。flash終端の
  最後のlook-aheadは範囲外になるため待機せず、次のscalar flash操作前にabstract cmderrをclearする。
  同じimageへの`--program-image --destructive`もpages=0、比較5.111秒、全域verify5.104秒、
  reset成功を確認した。変更pageのerase/program性能は別途測る。
- TargetFlash revision 2 は、image更新clientだけが使う`stage-page64`（4 fragment）と
  `commit-page256`を追加した。OEP message上限を変えず、full physical pageをprobe RAMへ
  完成させてから一回だけ消去・program・readbackする。既存の`--program-page64`は単発診断用の
  互換操作として残る。現在のimageと同じ先頭256 byteをstage/commitし、reset後に全62 KiB hashが
  一致することを実機で確認した。変更fragmentを含むimageの反復性能と故障注入は次のP0試験である。
- PWM probe imageへの実機更新では、27 physical pageの差分を`pages=27, attempts=27`で完走した。
  stage/commitを含む差分更新9.685秒、reset0.0046秒、全62 KiB verify5.119秒、最終reset0.0030秒で
  hash一致した。旧64-byte単発APIなら最大108回になるerase/programを27回へ畳めたことは確認したが、
  同一imageでの旧API比較、反復、fault injectionは未完了である。
- `OEP_X035_INJECT_FLASH_FAILURE=1`でerase直後に一度だけ失敗を注入したstage/commitでは、
  最初のcommitが診断`0xe1`で失敗し、独立readで先頭88 byteが全FFになった。同じP4 sessionの
  staged pageを再commitすると成功し、reset後の全62 KiB hashは元imageと一致した。これは
  probe RAMが生きている間の再送回復だけを示す。P4 reset/電源断後にerase前imageを再構成できない
  制約は変わらない。
- `OEP_X035_INJECT_FLASH_FAILURE=2`では最初の64-byte program直後に一度だけ失敗を注入した。
  最初のcommitは`0xe2`で失敗し、独立readで先頭64 byteだけが希望値、続く24 byteが全FFの部分状態を
  確認した。同じsessionのretry commitは成功し、reset後の全62 KiB hashは元imageと一致した。
- `half_period_us=0`は今回の短いfixture配線で全域hash一致を確認した設定である。配線条件が変わる
  汎用probeでは設定可能なままにし、エラー時は遅い設定へ戻せるようにする。
- target USBはOEP transportではない。USB deviceのbind状態はRVSWD書込みには関係しない。

正式なArduinoCore upload toolへ組み込む段階では、CLI出力文字列ではなくPython APIまたは安定した
machine-readable resultを使用する必要がある。現時点ではcore側の標準upload手段へ登録せず、manual
prototypeとして呼び出す。

## 現時点の使い勝手評価

- 固定alias、書込み前の全域backup、差分書込み、reset、全域verify、hash表示まで一連で行え、
  破壊的prototypeとしての最低限の復旧性はある。
- 高速化後は約80秒で別imageへ更新して全域検証できる。core開発の反復には使えるが、通常の
  Arduino uploadとしてはまだ遅い。今後は256-byte物理page単位の転送・処理が改善候補となる。
- 現在のP4 exampleはTargetControl / TargetMemory / TargetFlashに加え、FixtureGpioとFixtureUartを
  公開する。FixtureUartはUSART4（PB0→P4 GPIO12、PB1←P4 GPIO6）を115200 bpsで使え、`core_api`の
  command/response自己試験を完走した。FixtureGpioは安全なallowed pinのread、input pull、push-pull、
  open-drainを構成できる。target outputとの競合を避け、終了時はfloating inputへ戻す。
- ADC端点はPA5/P4 GPIO4で0 V相当（ADC=2）と3.3 V相当（ADC=1001）を実測した。一方、P4の両pullは
  ADC=345（約1.1 V）であり、1.65 V基準には使えない。中点のrelease検証には校正済みDACまたは外付け
  分圧が必要である。
- I2C/SPI peerとPWM波形計測は未実装である。GPIOのopen-drainはI2C bit-bangの基礎にはなるが、
  速度・clock stretch・SPI slaveの時刻保証を持つ専用capabilityへ発展させる必要がある。

- `FixtureI2c` revision 1 は P4 の一時的な hardware I2C target の状態取得だけを公開する。
  `getStatus()` は peer started、SCL/SDA の現在 level、最後の受信長、受信 transaction 回数、
  read request 回数、設定周波数を返す。I2C の成否をこの値だけで判定せず、RMT observer を
  追加した後は trace と対にして判定する。詳細な段階設計は
  [`esp32-p4-i2c-observation-design.ja.md`](esp32-p4-i2c-observation-design.ja.md) に記録する。
  Arduino-ESP32 3.3.11 の `Wire` slave は本治具で address ACK しなかったため、正式実装の
  基盤には使わない。ESP-IDF I2C slave driver の direct path は同じ10 kHz試験で受信できたが、
  現在は一回受信の診断用である。
- target型番・flash容量・保護状態を自動確認しないため、別targetへ誤って書くことを防げない。
  capabilityだけでなくtarget identityを返す機能が正式probeには必要である。
