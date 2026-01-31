#!/bin/bash
# Setup script for multiaxis_slicer

set -e

echo "================================"
echo "MultiAxis Slicer Setup"
echo "================================"
echo ""

# Check if Rust is installed
if ! command -v cargo &> /dev/null; then
    echo "❌ Rust is not installed"
    echo ""
    echo "Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source $HOME/.cargo/env
    echo "✅ Rust installed successfully"
else
    echo "✅ Rust is already installed"
    rustc --version
fi

echo ""
echo "Building project..."
cargo build --release

echo ""
echo "Running tests..."
cargo test --release

echo ""
echo "================================"
echo "✅ Setup complete!"
echo "================================"
echo ""
echo "Try running the example:"
echo "  cargo run --example simple_slicer --release"
echo ""
echo "Or build your own slicer using the library:"
echo "  See examples/simple_slicer.rs for reference"
echo ""
