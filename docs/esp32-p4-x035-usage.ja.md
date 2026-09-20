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
  --port /dev/ttyACM8 \
  --fqbn 'esp32:esp32:esp32p4:USBMode=hwcdc,CDCOnBoot=cdc' \
  --input-dir /tmp/oep-p4-x035 \
  examples/Esp32P4X035Prototype
```

実機名が利用できる環境では`/dev/ttyACM8`の代わりに次を使用できる。

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

元imageへ戻す場合、退避ファイルは既に63,488 byteなのでそのまま指定できる。

```sh
uv run python -m oep_client \
  --port /run/board-identify/by-id/esp32-series-30eda0e31108 \
  --program-image /tmp/ch32x035-original.bin \
  --destructive
```

CLIは入力binを63,488 byteまで`0xff`で埋め、target全域を先に読み、異なる64-byte論理pageだけを
programする。最後にresetし、全域を読み直してbyte単位で照合する。

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
- operation途中で失敗した場合、probe RAMのerase前imageを使って同じ64-byte要求の再送から回復する。
- 未回復中の別物理page要求は診断`0xe0`で拒否する。
- erase後にprobeもresetまたは電源断すると回復cacheは失われる。この場合、退避済みの完全imageを
  `--program-image`で再送する。部分imageや別pageから書込みを続行しない。
- targetのflash/option保護、型番、容量を自動識別していない。CH32X035以外へ既定値のまま使わない。
- 全域readまたはverifyは約240秒、109 pageの差分書込みは約298秒を実測した。
- target USBはOEP transportではない。USB deviceのbind状態はRVSWD書込みには関係しない。

正式なArduinoCore upload toolへ組み込む段階では、CLI出力文字列ではなくPython APIまたは安定した
machine-readable resultを使用する必要がある。現時点ではcore側の標準upload手段へ登録せず、manual
prototypeとして呼び出す。
