# Little Physics Engine

<table>
  <tr>
    <td valign="middle" width="40%"> <!-- Adjust width % as needed -->
      <img src="assets/gifs/highlight-reel.gif" alt="Combined Demo" width="100%"> <!-- Image scales to cell width -->
    </td>
    <td valign="top" width="60%" style="padding-left: 20px;"> <!-- Add padding for spacing -->
      This is a little C++ physics engine I built as a curiosity to project to see how far I could push a 99+% LLM 
      generated codebase. The ambition was to build this without any 3rd party dependencies, but some
      concenssions were made along the way (EnTT, SFML). Everything else was built from scratch.

      <br><b>Key Features:</b>
      <ul>
        <li>Barnes-Hut N-Body sim.</li>
        <li>GPU SPH fluids (Metal, Verlet, grid, rigid coupling).</li>
        <li>Rigid body pipeline (GJK/EPA, LCP/PGS solver, Baumgarte stab.).</li>
        <li>Solid, fluid (GPU screen-space), gas renderers.</li>
        <li>Basic UI controls & Scenario support.</li>
        <li>ECS design (EnTT).</li>
      </ul>
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