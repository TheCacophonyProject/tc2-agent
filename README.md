# Thermal Camera 2 (tc2) agent P2

This is designed to be a drop-in replacement for `leptond` on Cacophony Project 'classic' thermal cameras.
It runs on the newer 'Thermal Camera 2' hardware, and is responsible for connecting to the rp2040 CPU via SPI and reading off frames provided from the attached flir lepton module.
Gathered frames are then output to either a unix domain socket on the Raspberry pi, from which they are picked up by the `thermal-recorder` process – or, in development mode, frames can be output via TCP socket over wifi to a local development computer for debugging purposes. 

### Install dependencies 
`rustup target add armv7-unknown-linux-musleabihf`
`rustup target add aarch64-unknown-linux-musl`
### Build and deploy
1. Edit `deploy.sh` to specify the name of your development raspberry pi on your local network.
2. Run `deploy.sh`
