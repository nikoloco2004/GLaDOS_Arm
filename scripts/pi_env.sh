#!/usr/bin/env bash
# Source this so `uv` is on PATH after Astral's installer (default: ~/.local/bin).
# Usage:  source scripts/pi_env.sh
# Or:     . scripts/pi_env.sh

export PATH="${HOME}/.local/bin:${HOME}/.cargo/bin:${PATH}"
