#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SERVICE_NAME="game-piece-camera.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
CAMERA_DIR="${1:-${CAMERA_REPO_PATH:-}}"

if [[ -z "${CAMERA_DIR}" ]]; then
	CAMERA_DIR="$(realpath "$HOME/Documents/GitHub/Game-Piece-Detection" 2>/dev/null || true)"
	if [[ ! -f "${CAMERA_DIR}/camera.py" ]]; then
		CAMERA_DIR="$(realpath "$HOME/Game-Piece-Detection" 2>/dev/null || true)"
	fi
	if [[ ! -f "${CAMERA_DIR}/camera.py" ]]; then
		CAMERA_DIR="$(realpath "$HOME/Documents/Game-Piece-Detection" 2>/dev/null || true)"
	fi
	if [[ ! -f "${CAMERA_DIR}/camera.py" ]]; then
		CAMERA_DIR="$(realpath "$HOME/GitHub/Game-Piece-Detection" 2>/dev/null || true)"
	fi
fi

if [[ ! -f "${CAMERA_DIR}/camera.py" ]]; then
	echo "Could not find camera repo at: ${CAMERA_DIR:-<empty>}"
	echo "Run again with the camera repo path as first argument:"
	echo "  ./tools/camera/setup-camera-autostart.sh /absolute/path/to/Game-Piece-Detection"
	exit 1
fi

if [[ ! -d /etc/systemd/system ]]; then
	echo "This script requires systemd (/etc/systemd/system not found)."
	echo "Use this script only on systemd-based Linux systems."
	exit 1
fi

cd "${CAMERA_DIR}"

if [[ ! -d "${CAMERA_DIR}/.venv" ]]; then
	echo "Creating Python virtual environment in ${CAMERA_DIR}/.venv"
	python3 -m venv .venv
fi

echo "Installing Python dependencies"
"${CAMERA_DIR}/.venv/bin/python" -m pip install --upgrade pip
"${CAMERA_DIR}/.venv/bin/python" -m pip install flask opencv-python numpy

if ! command -v systemctl >/dev/null 2>&1; then
	echo "systemctl not found; cannot manage systemd services."
	exit 1
fi

USER_NAME="$(whoami)"
GROUP_NAME="$(id -gn "${USER_NAME}")"

cat > "${SERVICE_PATH}.tmp" <<EOF
[Unit]
Description=MECO game-piece camera server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER_NAME}
Group=${GROUP_NAME}
WorkingDirectory=${CAMERA_DIR}
ExecStart=${CAMERA_DIR}/.venv/bin/python ${CAMERA_DIR}/camera.py
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "Installing service ${SERVICE_NAME}"
sudo cp "${SERVICE_PATH}.tmp" "${SERVICE_PATH}"
sudo rm -f "${SERVICE_PATH}.tmp"

echo "Reloading systemd"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

echo "Service status"
systemctl --no-pager status "${SERVICE_NAME}"
echo "If startup fails, view logs:"
echo "  sudo journalctl -u ${SERVICE_NAME} -n 60 -f"
