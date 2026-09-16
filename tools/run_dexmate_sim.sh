#!/usr/bin/env bash
# Run within the dedicated Conda environment; keep library settings local to this process.
set -euo pipefail
if [[ "${CONDA_DEFAULT_ENV:-}" != "dexmate_isaacsim" || -z "${CONDA_PREFIX:-}" ]]; then
    echo "Activate the environment first: conda activate dexmate_isaacsim" >&2
    exit 2
fi
export LD_PRELOAD="${CONDA_PREFIX}/lib/libstdc++.so.6${LD_PRELOAD:+:${LD_PRELOAD}}"
export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"
exec "${CONDA_PREFIX}/bin/python" -u "$@"
