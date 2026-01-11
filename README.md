Autonomous Network Self-Healing (Flooding Prototype)
====================================================

Overview
--------
Small C++17 prototype for autonomous self-healing in drone swarms using a flooding-based
discovery and reply mechanism. The current code runs entirely in a simulated in-memory
communication manager layer for fast iteration.

Current layout
--------------
- [apps/flooding/main.cpp](apps/flooding/main.cpp): demo wiring five nodes together over the fake
  communication manager and starting floods.
- [apps/ns3_flooding/swarm_flooding.cc](apps/ns3_flooding/swarm_flooding.cc): ns-3 scratch scenario that exercises the flooding logic.
- [modules/flooding/flood.cpp](modules/flooding/flood.cpp),
  [modules/flooding/flood.h](modules/flooding/flood.h),
  [modules/flooding/messages.h](modules/flooding/messages.h): flooding logic and message formats.
- [interfaces/communication_manager.h](interfaces/communication_manager.h): abstract communication manager interface (send plus Packet type).
- [tests/fake_communication_manager.cpp](tests/fake_communication_manager.cpp),
  [tests/fake_communication_manager.h](tests/fake_communication_manager.h): in-memory communication manager implementation with neighbor links and
  RX callbacks for simulation/testing.
- [platform/ns3](platform/ns3): ns-3 adapter/backend code used by the scratch scenario.
- [apps/crazyflie](apps/crazyflie): small Python helpers for the Crazyflie “hardware lane”.

Platform setup
--------------

### Linux
Dependencies: CMake, C++17 compiler, clang-format (optional but recommended).

**Arch Linux:**
```bash
sudo pacman -S cmake clang
```

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install cmake g++ clang-format
```

**Fedora:**
```bash
sudo dnf install cmake gcc-c++ clang-tools-extra
```

### macOS
Install Xcode Command Line Tools and Homebrew, then:
```bash
xcode-select --install
brew install cmake llvm
```

Note: clang-format is included in LLVM. You may need to add it to PATH:
```bash
export PATH="/opt/homebrew/opt/llvm/bin:$PATH"  # Apple Silicon
export PATH="/usr/local/opt/llvm/bin:$PATH"      # Intel Mac
```

### Windows
**Option 1: Visual Studio (recommended)**
- Install [Visual Studio 2022 Community](https://visualstudio.microsoft.com/) with "Desktop development with C++"
- CMake is included in the Visual Studio installer
- Open the project folder in Visual Studio (File → Open → Folder)
- Visual Studio will automatically detect CMakeLists.txt

**Option 2: MinGW-w64 + CMake**
- Install [MSYS2](https://www.msys2.org/)
- Open MSYS2 MinGW 64-bit terminal:
  ```bash
  pacman -S mingw-w64-x86_64-cmake mingw-w64-x86_64-gcc mingw-w64-x86_64-clang-tools-extra
  ```
- Use the MSYS2 terminal for all build commands

**Option 3: WSL2 (Windows Subsystem for Linux)**
- Enable WSL2 and install Ubuntu from Microsoft Store
- Follow the Linux instructions above inside WSL

Build and run
-------------
**Linux/macOS:**
```
cmake -S . -B build
cmake --build build
./build/flooding_app
```

Docker + ns-3 (recommended for simulation)
-----------------------------------------
This repo includes a Docker workflow that builds ns-3 once, then lets you run ns-3 scratch programs against your local working tree via a bind-mount.

Build the image (slow, but cached):
```bash
docker build -t ns3-local:dev .
```

Run the flooding ns-3 scenario (fast iteration):
```bash
docker run --rm -v "$PWD":/workspace/Autonomous-Network-Self-Healing ns3-local:dev ./ns3 run scratch/swarm_flooding.cc
```

Notes:
- The container entrypoint symlinks `apps/ns3_flooding/*.cc` into `/opt/ns-3-dev/scratch/` on startup.
- `./ns3` is supported as a convenience: it is routed to `/opt/ns-3-dev/ns3` inside the container.
- Avoid reusing the same `build/` directory between host and container builds; CMake caches absolute paths.

For a deeper explanation of the container workflow, see [tmp/docker_container_explained.md](tmp/docker_container_explained.md).

**Windows (Visual Studio):**
- Open folder in Visual Studio, it will auto-configure CMake
- Press F5 to build and run, or use Build menu
- Executable will be in `build/Debug/flooding_app.exe` or `build/Release/flooding_app.exe`

**Windows (MinGW/MSYS2):**
```bash
cmake -G "MinGW Makefiles" -S . -B build
cmake --build build
./build/flooding_app.exe
```

What the demo does
------------------
- Builds a small topology (0-1-2-3 and 2-4) using FakeCommunicationManager neighbors.
- Each node owns a Flooding instance; RX callbacks hand packets to `Flooding::onPacketReceived`.
- `startFlood(flood_id)` broadcasts a FloodDiscoveryMsg.
- On first receipt of a discovery, a node records the flood_id, sends one FloodReplyMsg back to the
  base (ID 0 in the demo), and rebroadcasts the discovery. Duplicate floods are ignored.

C++ style conventions
---------------------
- Types: PascalCase (Flooding, FakeCommunicationManager).
- Functions/methods: lowerCamelCase (startFlood, onPacketReceived).
- Variables: lower_snake_case; constants ALL_CAPS or kPascalCase for scoped constants.
- Enums: PascalCase type names; enumerators in ALL_CAPS (DISCOVERY, REPLY).
- Headers: include guards via `#pragma once`; order includes as: standard library, third-party,
  project headers.
- Formatting: 4-space indent, K&R braces, limit lines to ~100 cols, no `using namespace std;`.
- Use const, constexpr, references, and smart pointers; avoid owning raw pointers unless
  interop/testing requires them (the fake communication manager keeps raw neighbor pointers by design).
- Prefer small, focused headers; keep implementation details in .cpp files.

Formatting with clang-format
----------------------------
- A baseline style is in [.clang-format](.clang-format). After installing clang-format:
  ```
  clang-format -i $(find apps modules interfaces tests -name '*.cpp' -o -name '*.h')
  ```
- If clang-format is missing, install it via your package manager (e.g., `sudo pacman -S clang` on
  Arch Linux, `sudo apt install clang-format` on Debian/Ubuntu).

Automated formatting
--------------------
- After running `cmake`, use `make format` to reformat all sources in one go.
- To enable the pre-commit hook so formatting happens automatically on commit:
  
  **Linux/macOS/WSL:**
  ```bash
  git config core.hooksPath githooks
  ```
  
  **Windows (Git Bash or PowerShell):**
  ```bash
  git config core.hooksPath githooks
  ```
  Note: The hook requires bash. If using native Windows (not WSL/Git Bash), you may need to run
  formatting manually with `make format` before commits.

The hook will run clang-format on all staged `.cpp`/`.h` files and re-add them.

Git commit convention
---------------------
- Use Conventional Commits: type(scope): summary
  - feat: new behavior or API
  - fix: bug fix
  - docs: documentation-only changes
  - test: add/adjust tests
  - refactor: code change without behavior change
  - chore: tooling, deps, CI, formatting
- Scope is optional but encouraged (e.g., flooding, communication_manager, docs).
- Summary in imperative mood, max ~72 chars. Body (if needed) explains what/why; wrap at 72 chars.

Next steps
----------
- Add real communication manager backends under platform/ (e.g., Crazyflie, ns-3) implementing CommunicationManagerInterface.
- Keep running clang-format before pushes to preserve the shared style.
