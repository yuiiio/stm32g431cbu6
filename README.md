# stm32g431cbu6-rust

```
rustup target add thumbv7em-none-eabihf
cargo install cargo-binutils
rustup component add llvm-tools-preview
cargo build
cargo run --release
```

# use stlink-v2 wihtout root
/etc/udev/rules.d/70-st-link.rules
```
# STM32F3DISCOVERY rev A/B - ST-LINK/V2
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", TAG+="uaccess"

# STM32F3DISCOVERY rev C+ - ST-LINK/V2-1
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", TAG+="uaccess"
```

generate rgb565 rawfile using
```
ffmpeg -vcodec png -i image.png -vcodec rawvideo -f rawvideo -pix_fmt rgb565 image.raw
```

# check binary size
arm-none-eabi-objcopy -O binary ./target/thumbv7em-none-eabihf/release/stm32g431cbu6-rust ./release.bin

# LICENSE
GPL-3.0
