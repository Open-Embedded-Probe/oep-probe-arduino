# ESP32-P4 / CH32X035 OEP probe 利用手順

状態: 仮置きの OEP v1（oep-spec `docs/v1-core-wire-delta.ja.md`）の probe。破壊的変更を前提とする。
v0 の prototype（`Esp32P4X035Prototype`、`python -m oep_client` の CLI）の手順は廃止した（git の履歴に残る）。

## 対象

- probe: ESP32-P4（`examples/Esp32P4X035Probe`）
- target: CH32X035F8U6 の治具
- OEP transport: ESP32-P4 の USB Serial/JTAG（長さつきフレーム）
- target transport: RVSWD（dedicated GPIO の bit-bang）

必須の配線は次の 2 本と共通 GND。target は別に給電し、信号は 3.3 V。

| ESP32-P4 | CH32X035 | 用途 |
|---:|---|---|
| GPIO2 | PC18 | SWDIO |
| GPIO54 | PC19 | SWCLK |

ほかの GPIO は `oep.fixture.*`（gpio / uart / capture）のチャンネルとして使える。この治具には NRST の配線がない。

## probe firmware

`examples/Esp32P4X035Probe` で:

```sh
arduino-cli compile --profile esp32p4 --jobs 2 --output-dir <build dir> .
arduino-cli upload --profile esp32p4 -p "$(readlink -f /run/board-identify/by-id/esp32-series-30eda0e31108)" \
  --input-dir <build dir> .
```

列挙順で変わる `/dev/ttyACM*` ではなく `/run/board-identify/by-id/esp32-series-30eda0e31108` を使う。
CDC 全体を OEP のフレームに使うので、serial monitor を同時に開かない。

## 使い方

host は oep-client-python の `oep_client.v1`（README に例）。書き込みの知識（RAM ローダー、ページ）は host 側に
ある（`ch32_flash`）。実機の一通りの確認は ArduinoCore-CH32 の `tests/manual/oep_smoke/`:

```sh
uv run tests/manual/oep_smoke/oep_smoke.py --target x035 --sketch all
uv run tests/manual/oep_smoke/oep_probe_checks.py --target x035
```

能力の一覧: `uv run python -m oep_client.v1 dump --port /run/board-identify/by-id/esp32-series-30eda0e31108`。
