# Pole Dynamics

Concrete utility poles and overhead wires for the São Paulo driving game: a
car hits a pole, which cracks, leans or snaps. The wires react, lines come
down into the street, live conductors arc, and power goes out.

Native C++20, deterministic, allocation-free in `step()`, with a C ABI for
Unity (P/Invoke) and a WebAssembly build for the browser lab.

```sh
cmake -S . -B build && cmake --build build -j && ./build/tests/pd_tests
```

Everything else is in [docs/README.md](docs/README.md): the model, the
tunables and their sources, test results, performance, and simplifications.
