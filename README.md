# Little Physics Engine

<table>
  <tr>
    <td valign="middle">
      <img src="assets/gifs/highlight-reel.gif" alt="Combined Demo" width="100%">
    </td>
    <td valign="top" width="60%" style="padding-left: 20px;">
      This is a little C++ physics engine I built as a curiosity project. The code is >99% LLM generated
      and attempts to build everything from scratch (a couple of concessions were made along the way; EnTT, SFML).
      <br><br><b>Key Features:</b><br>
      - Barnes-Hut N-Body sim.<br>
      - GPU SPH fluids (Metal, Verlet, grid, rigid coupling).<br>
      - Rigid body pipeline (GJK/EPA, LCP/PGS solver, Baumgarte stab.).<br>
      - Solid, fluid (GPU screen-space), gas renderers.<br>
      - Basic UI controls & Scenario support.<br>
      - ECS design (EnTT).
    </td>
  </tr>
</table>


## Setup (Assumes Mac)

- Install build dependencies (includes SFML for graphics):
```bash
brew install sfml
```

- Install clang-tidy (for linting) & bear (for compile_commands.json generation):
```bash
brew install llvm bear
echo 'export PATH="/opt/homebrew/opt/llvm/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

- Install entt (ECS library):
```bash
# Create the proper directory structure
mkdir -p vendor/entt/include/entt

# Download just the single header file
curl -L https://raw.githubusercontent.com/skypjack/entt/v3.12.2/single_include/entt/entt.hpp \
    -o vendor/entt/include/entt/entt.hpp
```

## Building

```bash
# Build the simulator (default target)
make

# Clean the build directory
make clean

# Rebuild from scratch
make rebuild

# Lint code (requires clang-tidy)
make lint

# Fix lint errors automatically
make lint-fix
```

## Running

After building, the executable will be located at `build/simulator`:

```bash
./build/simulator
```