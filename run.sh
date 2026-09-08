#!/bin/sh
cd `dirname $0`

VENV_NAME="venv"
PYTHON="$VENV_NAME/bin/python"

sh ./setup.sh

echo "Starting module..."
exec $PYTHON -m src.main $@
