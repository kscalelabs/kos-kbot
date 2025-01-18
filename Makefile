.PHONY: all build check test lint deps clean

all: deps build

# Install system dependencies
deps:
	sudo apt-get update
	sudo apt-get install -y \
		libgstreamer1.0-dev \
		libgstreamer-plugins-base1.0-dev \
		libgstreamer-plugins-bad1.0-dev \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		gstreamer1.0-tools \
		gstreamer1.0-x \
		gstreamer1.0-alsa \
		gstreamer1.0-gl \
		libglib2.0-dev \
		pkg-config

# Build the project
build:
	cargo build

# Run cargo check
check:
	cargo check

# Run tests
test:
	cargo test

# Run linting
lint:
	cargo clippy

# Clean build artifacts
clean:
	cargo clean 
