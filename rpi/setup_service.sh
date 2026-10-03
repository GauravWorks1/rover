#!/bin/bash
# ============================================================
#  Rover Boot Service — Install / Uninstall / Status
#  Run on Raspberry Pi:  sudo bash setup_service.sh install
# ============================================================

set -e

SERVICE_NAME="rover"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SOURCE_SERVICE="$(dirname "$0")/rover.service"
LOG_DIR="/home/pi/rover/logs"

usage() {
    echo ""
    echo "Usage: sudo bash $0 [install|uninstall|status|logs|restart]"
    echo ""
    echo "  install    — Copy service file, enable & start on boot"
    echo "  uninstall  — Stop, disable & remove service"
    echo "  status     — Show current service status"
    echo "  logs       — Show live logs (Ctrl+C to exit)"
    echo "  restart    — Restart the service"
    echo ""
}

check_root() {
    if [ "$EUID" -ne 0 ]; then
        echo "❌ Please run with sudo: sudo bash $0 $1"
        exit 1
    fi
}

install_service() {
    check_root "install"
    
    echo "📦 Installing Rover boot service..."
    echo ""

    # Create log directory
    mkdir -p "$LOG_DIR"
    chown pi:pi "$LOG_DIR"
    echo "✅ Created log directory: $LOG_DIR"

    # Copy service file
    cp "$SOURCE_SERVICE" "$SERVICE_FILE"
    echo "✅ Copied service file to $SERVICE_FILE"

    # Reload systemd
    systemctl daemon-reload
    echo "✅ Reloaded systemd daemon"

    # Enable on boot
    systemctl enable "$SERVICE_NAME"
    echo "✅ Enabled $SERVICE_NAME to start on boot"

    # Start now
    systemctl start "$SERVICE_NAME"
    echo "✅ Started $SERVICE_NAME service"

    echo ""
    echo "🎉 Done! Rover will now auto-start on every boot."
    echo ""
    echo "   Check status:  sudo systemctl status rover"
    echo "   View logs:     sudo journalctl -u rover -f"
    echo "   Stop:          sudo systemctl stop rover"
    echo "   Restart:       sudo systemctl restart rover"
    echo "   Disable boot:  sudo systemctl disable rover"
    echo ""
}

uninstall_service() {
    check_root "uninstall"

    echo "🗑️  Removing Rover boot service..."

    systemctl stop "$SERVICE_NAME" 2>/dev/null || true
    systemctl disable "$SERVICE_NAME" 2>/dev/null || true
    rm -f "$SERVICE_FILE"
    systemctl daemon-reload

    echo "✅ Service removed. Rover will no longer start on boot."
}

show_status() {
    systemctl status "$SERVICE_NAME" --no-pager || true
}

show_logs() {
    echo "📋 Showing live rover logs (Ctrl+C to exit)..."
    journalctl -u "$SERVICE_NAME" -f
}

restart_service() {
    check_root "restart"
    systemctl restart "$SERVICE_NAME"
    echo "✅ Rover service restarted"
    systemctl status "$SERVICE_NAME" --no-pager
}

# Main
case "${1}" in
    install)    install_service ;;
    uninstall)  uninstall_service ;;
    status)     show_status ;;
    logs)       show_logs ;;
    restart)    restart_service ;;
    *)          usage ;;
esac
