#!/bin/bash
exec /bin/bash "$(dirname "$0")/start-trial.sh" --preview-only "$@"
