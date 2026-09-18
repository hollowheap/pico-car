{
  description = "Raspberry pi pico development environment";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      nixpkgs,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };
        pico-sdk = pkgs.pico-sdk.override {
          withSubmodules = true;
        };
      in
      {
        devShell = pkgs.mkShell {
          packages = with pkgs; [
            cmake
            coreutils
            gcc-arm-embedded
            gnutar
            picotool
            python3
          ];

          shellHook = ''
            export PICO_SDK_PATH="${pico-sdk}/lib/pico-sdk"
          '';
        };
      }
    );
}
