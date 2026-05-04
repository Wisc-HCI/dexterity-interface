#!/bin/bash
# Builds and serves the frontend dev server.

set -e

DIR="$(cd "$(dirname "$0")/.." && pwd)"

npm run build --prefix "$DIR/app/ui_frontend/"

