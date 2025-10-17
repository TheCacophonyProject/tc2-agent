# Thermal Camera 2 (tc2) agent P2

This is designed to be a drop-in replacement for `leptond` on Cacophony Project 'classic' thermal cameras.
It runs on the newer 'Thermal Camera 2' hardware, and is responsible for connecting to the rp2040 CPU via SPI and reading off frames provided from the attached flir lepton module.
Gathered frames are then output to either a unix domain socket on the Raspberry pi, from which they are picked up by the `thermal-recorder` process – or, in development mode, frames can be output via TCP socket over wifi to a local development computer for debugging purposes. 

### Install dependencies 
`rustup target add aarch64-unknown-linux-musl`
`sudo apt-get install gcc-aarch64-linux-gnu`
### Build and deploy
1. Edit `deploy.sh` to specify the name of your development raspberry pi on your local network.
2. Run `deploy.sh`

## Making release with updated firmware 
- In [`tc2-firmware`](https://github.com/TheCacophonyProject/tc2-firmware) you will first need to.
    - Increase by 1 the `FIRMWARE_VERSION` in `src/constants.rs`.
    - Merge that then make a new release.
- Back in the `tc2-agent` repo, update `RP2040_FIRMWARE_VERSION` in `.github/workflows/release.yaml` to match the new github release version.
- Update `EXPECTED_RP2040_FIRMWARE_VERSION` in `src/main.rs` to the new firmware version.
- Now when making a release it will grab the new firmware and pack that into the .deb file so the RP2040 can be programmed.
