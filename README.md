# Setup

```
brew install emscripten
# Create the proper directory structure
mkdir -p external/entt/include/entt

# Download just the single header file
curl -L https://raw.githubusercontent.com/skypjack/entt/v3.12.2/single_include/entt/entt.hpp \
    -o external/entt/include/entt/entt.hpp
make
```