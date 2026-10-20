#!/usr/bin/env bash

# Use this to build the WebAssembly demo (requires the Emscripten SDK)
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release
cmake --build build-web
echo "Serve it with: python3 -m http.server -d build-web/web"
