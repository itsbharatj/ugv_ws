#!/usr/bin/env bash
# shellcheck disable=SC1091,SC2016,SC2034
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=../setup_vnc.sh
source "$REPO_ROOT/setup_vnc.sh"

TEST_DIR=$(mktemp -d)
trap 'cleanup; rm -rf "$TEST_DIR"' EXIT

assert_contains() {
    local file=$1
    local expected=$2
    grep -Fq "$expected" "$file" ||
        {
            printf 'Expected %s to contain: %s\n' "$file" "$expected" >&2
            exit 1
        }
}

assert_not_contains() {
    local file=$1
    local unexpected=$2
    if grep -Fq "$unexpected" "$file"; then
        printf 'Expected %s not to contain: %s\n' "$file" "$unexpected" >&2
        exit 1
    fi
}

test_config_rendering() {
    local input=$TEST_DIR/config.in
    local output=$TEST_DIR/config.out
    local repeated=$TEST_DIR/config.repeated

    cat >"$input" <<EOF
\$desktopName = "keep-me";
$MANAGED_BEGIN
\$geometry = "800x600";
$MANAGED_END
EOF

    render_vnc_config "$input" "$output" 1920x1080 local

    assert_contains "$output" '$desktopName = "keep-me";'
    assert_contains "$output" '$geometry = "1920x1080";'
    assert_contains "$output" '$vncStartup = "$ENV{HOME}/.vnc/ugv-xstartup";'
    assert_contains "$output" '$localhost = "yes";'
    assert_contains "$output" '$SecurityTypes = "VncAuth";'
    assert_not_contains "$output" '\$session'
    [[ $(grep -Fc "$MANAGED_BEGIN" "$output") -eq 1 ]]

    render_vnc_config "$output" "$repeated" 1920x1080 local
    cmp -s "$output" "$repeated" ||
        {
            echo "rendering the same configuration was not idempotent" >&2
            exit 1
        }

    render_vnc_config "$output" "$input" 1280x720 lan
    assert_contains "$input" '$geometry = "1280x720";'
    assert_contains "$input" '$localhost = "no";'
    assert_contains "$input" '$SecurityTypes = "TLSVnc";'
    assert_not_contains "$input" '$geometry = "1920x1080";'
}

test_startup_rendering() {
    local output=$TEST_DIR/ugv-xstartup
    render_vnc_startup "$output"

    assert_contains "$output" "$STARTUP_MARKER"
    assert_contains "$output" 'exec dbus-run-session -- startxfce4'
    assert_contains "$output" 'unset DBUS_SESSION_BUS_ADDRESS'
}

test_firewall_transition() {
    local calls=$TEST_DIR/firewall.calls
    STATE_FILE=$TEST_DIR/firewall.state
    ACCESS_MODE=local
    DISPLAY_NUMBER=1
    PORT=5901

    cat >"$STATE_FILE" <<'EOF'
access=lan
allow_from=192.168.1.0/24
EOF

    as_root() {
        printf '%s\n' "$*" >>"$calls"
    }

    configure_firewall
    assert_contains "$calls" \
        'ufw --force delete allow from 192.168.1.0/24 to any port 5901 proto tcp'
}

test_display_mapping() {
    local input=$TEST_DIR/users.in
    local output=$TEST_DIR/users.out

    cat >"$input" <<'EOF'
# Existing assignments
:1=old-user
:2=another-user
EOF

    render_users_file "$input" "$output" 1 new-user
    assert_contains "$output" ':1=new-user'
    assert_contains "$output" ':2=another-user'
    assert_not_contains "$output" ':1=old-user'
    [[ $(mapping_owner "$output" 1) == new-user ]]

    remove_users_mapping "$output" "$input" 1 new-user
    assert_not_contains "$input" ':1=new-user'
    assert_contains "$input" ':2=another-user'
}

test_cron_migration() {
    local input=$TEST_DIR/cron.in
    local output=$TEST_DIR/cron.out

    cat >"$input" <<'EOF'
@daily /usr/local/bin/backup
@reboot /usr/bin/vncserver -geometry 1920x1080 -localhost no :1
@reboot /usr/bin/vncserver -geometry 1280x720 :2
EOF

    filter_legacy_crontab "$input" "$output" 1
    assert_contains "$output" '@daily /usr/local/bin/backup'
    assert_contains "$output" ':2'
    assert_not_contains "$output" ':1'
}

test_validators() {
    validate_display 1
    validate_display 99
    validate_geometry 1920x1080
    validate_cidr 192.168.1.0/24

    if (validate_display 0) 2>/dev/null; then
        echo "display validator accepted 0" >&2
        exit 1
    fi
    if (validate_cidr 999.168.1.0/24) 2>/dev/null; then
        echo "CIDR validator accepted an invalid address" >&2
        exit 1
    fi
}

test_config_rendering
test_startup_rendering
test_display_mapping
test_cron_migration
test_firewall_transition
test_validators

echo "setup_vnc.sh tests passed"
