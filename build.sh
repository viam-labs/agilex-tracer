#!/usr/bin/env bash
# Packages the module into a source tarball (no PyInstaller).
set -euo pipefail

cd "$(dirname "$0")"

chmod +x setup.sh run.sh

COPYFILE_DISABLE=1 tar -czf module.tar.gz \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.DS_Store' \
    meta.json \
    requirements.txt \
    setup.sh \
    run.sh \
    src \
    README.md

echo "Wrote module.tar.gz"
