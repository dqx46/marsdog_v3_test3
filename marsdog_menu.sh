#!/bin/bash
# Marsdog 日常工具菜单入口
#   ./marsdog_menu.sh
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$DIR/run_with_env.sh" python -m marsdog_control.apps.tools.marsdog_menu "$@"
