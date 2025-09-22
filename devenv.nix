{ pkgs, lib, config, inputs, ... }:

{
  packages = with pkgs; [
    # Rust toolchain
    cargo
    rustc
    rustfmt
    clippy
    
    # ARM cross-compilation tools
    gcc-arm-embedded
    openocd
    
    # Development tools
    gdb
    probe-rs
    
    # Build dependencies
    openssl
    pkg-config
  ];

  languages.rust = {
    enable = true;
    channel = "stable";
    targets = [ "thumbv7em-none-eabihf" ];
  };

  enterShell = ''
    echo "UWB Anchor Firmware Development Environment"
    echo "Rust version: $(rustc --version)"
    echo "Available targets: $(rustup target list --installed)"
  '';
}
