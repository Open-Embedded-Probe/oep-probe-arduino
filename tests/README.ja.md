# HIL 回帰試験

firmware は**使う前に必ず転送する**。pytest-embedded-arduino-cli が example sketch を build → upload し、
その後 test が OEP v0 client（oep-client-python、path 依存）で probe を叩く。port は `.env` の
`TEST_SERIAL_PORT_<PROFILE>`。P4 の USB-Serial/JTAG は port の open/close で board が reset するため、
client は harness が開いた pyserial 実体を借りる（`hil/conftest.py`）。

```sh
cd tests
cp .env.example .env   # 編集
uv run --env-file .env pytest hil
```
