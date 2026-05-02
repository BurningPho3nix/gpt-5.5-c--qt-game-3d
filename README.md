# Qt World Shooter

A small C++/Qt 6 first-person 3D prototype. You can explore a larger 300x300 meter outdoor world with rolling hills, grass, trees, rocks, platforms, towers, a mirror, and shootable terrain.

## Build

```bash
cmake -S . -B build
cmake --build build
./build/qt-wall-shooter
```

## Controls

- `W`, `A`, `S`, `D`: move
- `Shift`: move faster while held
- `Space`: jump
- `Ctrl` or `C`: crouch
- Mouse: look around after clicking the game
- Left click: shoot any world surface
- Right click: use the weapon scope
- `R`: reset bullet marks and position
- `Esc`: release the mouse cursor

The world is 300x300 meters, centered at the origin. It includes rolling grass terrain, hills that affect walking and shooting, tree clusters, rocks, shootable boundary walls, raised plazas, block towers, stairs, platforms, a standing mirror near the spawn path, atmospheric fog, a sky gradient, and surface-aligned bullet decals.
