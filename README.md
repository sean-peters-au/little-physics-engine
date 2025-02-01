# N-Body Simulation

A C++ N-Body simulation that can be compiled for both native (desktop) and web (WASM) targets.

## Setup

This was all built for and assumes MacOS, M2, and zsh.

- Install emscripten (for WASM build)
```bash
brew install emscripten
```

- Install clang-tidy (for linting)
```bash
brew install llvm bear
echo 'export PATH="/opt/homebrew/opt/llvm/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc 
```

- Install entt
```bash
# Create the proper directory structure
mkdir -p external/entt/include/entt

# Download just the single header file
curl -L https://raw.githubusercontent.com/skypjack/entt/v3.12.2/single_include/entt/entt.hpp \
    -o external/entt/include/entt/entt.hpp
```

## Building

```bash
# Build native version
make native

# Build WASM version
make wasm

# Build and serve WASM version (opens in browser)
make serve

# Run tests
make test

# Lint code (requires clang-tidy)
make lint
```