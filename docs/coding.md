# C++ Coding Standards

## Include Order
Files should organize their includes in the following groups, separated by blank lines:

1. Corresponding header file (if this is foo.cpp, then foo.hpp)
2. C system headers (<stdio.h>, etc.)
3. C++ standard library headers (<iostream>, etc.)
4. Third-party library headers (SFML, Boost, etc.)
5. Project headers (with full paths from include root)

Example:
```cpp
// 1. Corresponding header
#include "foo.hpp"

// 2. C system headers
#include <stdio.h>
#include <stdlib.h>

// 3. C++ standard library headers
#include <iostream>
#include <string>
#include <vector>

// 4. Third-party library headers
#include <SFML/Graphics.hpp>
#include <boost/algorithm/string.hpp>

// 5. Project headers
#include "myproject/foo.hpp"
#include "myproject/bar.hpp"
```

## Naming Conventions

### Classes and Types
- Classes, Structs, Enums: `CamelCase`
- Template Parameters: `CamelCase`
- Type Aliases: `CamelCase`

### Variables and Functions
- Functions: `camelBack()`
- Variables: `camelBack`
- Parameters: `camelBack`
- Class Members: `camelBack`
- Private Class Members: `camelBack_` (with underscore suffix)

### Constants and Enums
- Global Constants: `UPPER_CASE`
- Static Constants: `UPPER_CASE`
- Enum Values: `UPPER_CASE`
- Constexpr Variables: `CamelCase`

### Namespaces
- Namespace Names: `CamelCase`

## Style Checks
Code is automatically checked for:
- Bugprone patterns
- Google style conformance
- Modern C++ practices
- Performance anti-patterns
- Portability issues
- General readability

Notable exceptions:
- Trailing return types are optional
- Namespace comments are not required
- Runtime integer type warnings are disabled
- Non-const reference parameters are allowed