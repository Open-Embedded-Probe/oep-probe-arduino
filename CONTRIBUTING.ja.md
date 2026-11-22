# 貢献の手引き

[English](CONTRIBUTING.md)

## 文書

文書は日本語（`.ja.md`）と英語（`.md`）を同じ変更で直します。oep-spec の凍結（`v1.0.0`）までは日本語が正です。
文書は今の状態だけを書き、変更の履歴は CHANGELOG.md に書きます。

## リリース

実装・更新・更新後の回帰の責任と検証順は [プローブの検証](docs/testing.ja.md)を参照。image の生成と実機品質の確認を分ける。

[arduino-library-release-toolkit](https://github.com/tanakamasayuki/arduino-library-release-toolkit) の共通の仕組みをそのまま使う。
変更は `CHANGELOG.md` の `## Unreleased` に (EN) / (JA) で書き足し、GitHub Actions の Release（workflow_dispatch）を起動すると、
`library.properties` の版を上げ、`src/openembeddedprobe_version.h` を作り、`release` ブランチで example の `sketch.yaml` の `dir: ../..` を
`OpenEmbeddedProbe (<版>)` に書き換え、`tests/` を除いた ZIP、tag、GitHub Release を作る。続けて、別の `.github/workflows/firmware.yml`（toolkit のものではない）が
tag から各 example の各 profile をビルドし、`<Example>-<profile>-<version>.merged.bin`（ESP32、0x0 に書く）/ `.bin`（ESP32、更新用の app の image。P4 の DFU）/ `.uf2`（RP2040 / RP2350）/ `.bin`（CH32、code flash の全体）と
`firmware-<version>.json`（sha256）を Release に付ける（付けるのは `examples/Firmware/` の分だけ。ほかの example は確かめるためにビルドする）。probe は同じ版を describe の firmware の文字列で返す。
