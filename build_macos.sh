#!/bin/sh
set -eu
cd "$(dirname "$0")"
bundle="build/Mono Delay.clap"
mkdir -p "$bundle/Contents/MacOS"
cp Info.plist "$bundle/Contents/Info.plist"
xcrun clang -std=c11 -O2 -Wall -Wextra -Wpedantic -Wno-unused-function -fvisibility=hidden \
  -mmacosx-version-min=11.0 -arch arm64 -arch x86_64 -bundle \
  -I vendor/clap/include source/mono_delay_clap.c -Wl,-exported_symbol,_clap_entry \
  -o "$bundle/Contents/MacOS/Mono Delay"
codesign --force --sign - --timestamp=none "$bundle"
printf 'Built: %s\n' "$bundle"
