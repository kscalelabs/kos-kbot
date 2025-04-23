# K-Scale OS - K-Bot

Welcome to the K-Scale OS build for the K-Bot!

## Building

### Prerequisites

- `cross` toolchain

### Native build

Native build with stub features:

```
apt install protobuf-compiler libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libglib2.0-dev  pkg-config
```

```bash
cargo build
```

### Cross build

Cross build for `kbot`:

```bash
cross build --release --target aarch64-unknown-linux-gnu
```

## Running

```bash
RUST_LOG=debug cargo run
```

You can specify logging levels for individual modules by adding `module_name=log_level` to the `RUST_LOG` environment variable. For example:

```bash
RUST_LOG=debug,krec=warn cargo run
```

## Contributing

- Use `cargo fmt --all` to format the code.
- Use `cargo clippy` to check for lint errors.
- Use `cargo test` to run the tests.
- Use `tracing` for logging.
- Use `eyre` to handle errors.
- No `unwrap()` or `expect()`.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
