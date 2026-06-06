#!/usr/bin/env bash
# shellcheck disable=SC2016 # Literal Perl variables are written to tigervnc.conf.
set -euo pipefail

readonly PROGRAM_NAME=${0##*/}
readonly MANAGED_BEGIN="# BEGIN UGV_WORKSPACE_VNC"
readonly MANAGED_END="# END UGV_WORKSPACE_VNC"
readonly STARTUP_MARKER="# Managed by ugv_ws/setup_vnc.sh"

ACTION=install
DISPLAY_NUMBER=1
GEOMETRY=1920x1080
ACCESS_MODE=local
ALLOW_FROM=
RESET_PASSWORD=false
ASSUME_YES=false
DRY_RUN=false
FORCE=false

TARGET_USER=
TARGET_HOME=
TARGET_UID=
TARGET_GID=
VNC_DIR=
VNC_CONFIG=
VNC_PASSWORD=
VNC_STARTUP=
STATE_FILE=
USERS_FILE=/etc/tigervnc/vncserver.users
SERVICE_UNIT=
PORT=

TEMP_FILES=()

usage() {
    cat <<EOF
Install and manage a TigerVNC virtual desktop on Ubuntu 22.04+ or Debian 12+.

Usage:
  $PROGRAM_NAME [install] [options]
  $PROGRAM_NAME status [--display NUMBER]
  $PROGRAM_NAME uninstall [--display NUMBER] [--yes]

Options:
  --display NUMBER       VNC display number (default: 1)
  --geometry WIDTHxHEIGHT
                         Virtual desktop size (default: 1920x1080)
  --lan                  Listen on the network instead of localhost
  --allow-from CIDR      Required with --lan; firewall source, e.g. 192.168.1.0/24
  --reset-password       Prompt for a new VNC password
  --force                Replace a conflicting display-to-user assignment
  --yes                  Skip the confirmation prompt
  --dry-run              Show the planned operations without changing the system
  -h, --help             Show this help

Secure default:
  VNC listens only on localhost. Connect through SSH port forwarding:
    ssh -L 5901:localhost:5901 USER@HOST
  Then point the VNC viewer at localhost:1.
EOF
}

log() {
    printf '%s\n' "$*"
}

warn() {
    printf 'Warning: %s\n' "$*" >&2
}

die() {
    printf 'Error: %s\n' "$*" >&2
    exit 1
}

shell_join() {
    printf '%q ' "$@"
}

run() {
    if [[ $DRY_RUN == true ]]; then
        printf '+ '
        shell_join "$@"
        printf '\n'
        return 0
    fi
    "$@"
}

as_root() {
    if (( EUID == 0 )); then
        run "$@"
    else
        run sudo "$@"
    fi
}

as_user() {
    if (( EUID == 0 )); then
        run sudo -u "$TARGET_USER" -H "$@"
    else
        run "$@"
    fi
}

new_temp_file() {
    local variable_name=$1
    local file
    file=$(mktemp)
    TEMP_FILES+=("$file")
    printf -v "$variable_name" '%s' "$file"
}

cleanup() {
    if ((${#TEMP_FILES[@]} > 0)); then
        rm -f -- "${TEMP_FILES[@]}"
    fi
}

trap cleanup EXIT

validate_display() {
    [[ $1 =~ ^[1-9][0-9]?$ ]] ||
        die "display must be a number from 1 to 99"
}

validate_geometry() {
    [[ $1 =~ ^[0-9]{3,5}x[0-9]{3,5}$ ]] ||
        die "geometry must use WIDTHxHEIGHT, for example 1920x1080"
}

validate_cidr() {
    [[ $1 =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}/([0-9]|[12][0-9]|3[0-2])$ ]] ||
        die "allow-from must be an IPv4 CIDR, for example 192.168.1.0/24"

    local address=${1%/*}
    local octet
    local -a octets
    IFS=. read -r -a octets <<<"$address"
    for octet in "${octets[@]}"; do
        ((10#$octet <= 255)) || die "allow-from contains an invalid IPv4 address"
    done
}

parse_arguments() {
    if (($# > 0)); then
        case $1 in
            install | status | uninstall)
                ACTION=$1
                shift
                ;;
        esac
    fi

    while (($# > 0)); do
        case $1 in
            --display)
                (($# >= 2)) || die "--display requires a value"
                DISPLAY_NUMBER=$2
                shift 2
                ;;
            --geometry)
                (($# >= 2)) || die "--geometry requires a value"
                GEOMETRY=$2
                shift 2
                ;;
            --lan)
                ACCESS_MODE=lan
                shift
                ;;
            --allow-from)
                (($# >= 2)) || die "--allow-from requires a value"
                ALLOW_FROM=$2
                shift 2
                ;;
            --reset-password)
                RESET_PASSWORD=true
                shift
                ;;
            --force)
                FORCE=true
                shift
                ;;
            --yes)
                ASSUME_YES=true
                shift
                ;;
            --dry-run)
                DRY_RUN=true
                shift
                ;;
            -h | --help)
                usage
                exit 0
                ;;
            *)
                die "unknown argument: $1"
                ;;
        esac
    done

    validate_display "$DISPLAY_NUMBER"
    validate_geometry "$GEOMETRY"

    if [[ $ACCESS_MODE == lan ]]; then
        [[ -n $ALLOW_FROM ]] ||
            die "--lan requires --allow-from CIDR to avoid exposing VNC globally"
        validate_cidr "$ALLOW_FROM"
    elif [[ -n $ALLOW_FROM ]]; then
        die "--allow-from can only be used with --lan"
    fi
}

detect_target_user() {
    if (( EUID == 0 )); then
        [[ -n ${SUDO_USER:-} && $SUDO_USER != root ]] ||
            die "run as a normal user, or invoke with sudo from a normal user"
        TARGET_USER=$SUDO_USER
    else
        TARGET_USER=$(id -un)
    fi

    [[ $TARGET_USER =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] ||
        die "unsupported account name: $TARGET_USER"

    local passwd_entry
    passwd_entry=$(getent passwd "$TARGET_USER") ||
        die "cannot find account information for $TARGET_USER"

    IFS=: read -r _ _ TARGET_UID TARGET_GID _ TARGET_HOME _ <<<"$passwd_entry"
    [[ -d $TARGET_HOME ]] || die "home directory does not exist: $TARGET_HOME"

    VNC_DIR=$TARGET_HOME/.vnc
    VNC_CONFIG=$VNC_DIR/tigervnc.conf
    VNC_PASSWORD=$VNC_DIR/passwd
    VNC_STARTUP=$VNC_DIR/ugv-xstartup
    STATE_FILE=$VNC_DIR/ugv-vnc-display-$DISPLAY_NUMBER.state
    SERVICE_UNIT="tigervncserver@:${DISPLAY_NUMBER}.service"
    PORT=$((5900 + DISPLAY_NUMBER))
}

check_platform() {
    command -v apt-get >/dev/null ||
        die "this installer currently supports apt-based Ubuntu and Debian systems"
    command -v systemctl >/dev/null || die "systemd is required"

    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        . /etc/os-release
        case ${ID:-} in
            ubuntu)
                ((${VERSION_ID%%.*} >= 22)) ||
                    die "Ubuntu 22.04 or newer is required"
                ;;
            debian)
                ((${VERSION_ID%%.*} >= 12)) ||
                    die "Debian 12 or newer is required"
                ;;
            linuxmint | pop)
                warn "Ubuntu derivative support is best effort: ${PRETTY_NAME:-unknown}"
                ;;
            *)
                warn "untested distribution: ${PRETTY_NAME:-unknown}"
                ;;
        esac
    fi
}

confirm_install() {
    [[ $ASSUME_YES == true || $DRY_RUN == true ]] && return 0

    cat <<EOF
TigerVNC will be configured for:
  User:       $TARGET_USER
  Display:    :$DISPLAY_NUMBER (TCP port $PORT)
  Session:    XFCE
  Geometry:   $GEOMETRY
  Access:     $ACCESS_MODE
EOF
    if [[ $ACCESS_MODE == lan ]]; then
        printf '  Allow from: %s\n' "$ALLOW_FROM"
    fi

    read -r -p "Continue? [y/N] " reply
    [[ $reply == y || $reply == Y ]] || die "installation cancelled"
}

strip_managed_block() {
    local input=$1
    local output=$2

    awk -v begin="$MANAGED_BEGIN" -v end="$MANAGED_END" '
        $0 == begin { managed = 1; next }
        $0 == end { managed = 0; next }
        !managed { print }
    ' "$input" >"$output"
}

render_vnc_config() {
    local input=$1
    local output=$2
    local geometry=$3
    local access=$4
    local base trimmed

    new_temp_file base
    new_temp_file trimmed
    if [[ -f $input ]]; then
        strip_managed_block "$input" "$base"
    else
        : >"$base"
    fi

    awk '
        { lines[NR] = $0 }
        END {
            last = NR
            while (last > 0 && lines[last] ~ /^[[:space:]]*$/) {
                last--
            }
            for (line = 1; line <= last; line++) {
                print lines[line]
            }
        }
    ' "$base" >"$trimmed"

    cat "$trimmed" >"$output"
    if [[ -s $output ]]; then
        printf '\n' >>"$output"
    fi

    {
        printf '%s\n' "$MANAGED_BEGIN"
        printf '$geometry = "%s";\n' "$geometry"
        printf '$depth = "24";\n'
        printf '$vncStartup = "$ENV{HOME}/.vnc/ugv-xstartup";\n'
        if [[ $access == lan ]]; then
            printf '$localhost = "no";\n'
            printf '$SecurityTypes = "TLSVnc";\n'
        else
            printf '$localhost = "yes";\n'
            printf '$SecurityTypes = "VncAuth";\n'
        fi
        printf '%s\n' "$MANAGED_END"
    } >>"$output"
}

render_vnc_startup() {
    local output=$1

    cat >"$output" <<EOF
#!/bin/sh
$STARTUP_MARKER
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP=XFCE
export DESKTOP_SESSION=xfce
exec dbus-run-session -- startxfce4
EOF
}

mapping_owner() {
    local input=$1
    local display=$2

    [[ -f $input ]] || return 0
    awk -v display="$display" '
        {
            line = $0
            sub(/^[ \t]*/, "", line)
            if (line ~ ("^:" display "[ \t]*=")) {
                sub(/^[^=]*=[ \t]*/, "", line)
                sub(/[ \t]*$/, "", line)
                print line
                exit
            }
        }
    ' "$input"
}

render_users_file() {
    local input=$1
    local output=$2
    local display=$3
    local user=$4

    if [[ -f $input ]]; then
        awk -v display="$display" '
            {
                line = $0
                sub(/^[ \t]*/, "", line)
                if (line ~ ("^:" display "[ \t]*=")) {
                    next
                }
                print
            }
        ' "$input" >"$output"
    else
        cat >"$output" <<'EOF'
# TigerVNC display-to-user assignments.
# Format: :DISPLAY=USER
EOF
    fi

    printf ':%s=%s\n' "$display" "$user" >>"$output"
}

remove_users_mapping() {
    local input=$1
    local output=$2
    local display=$3
    local user=$4

    awk -v display="$display" -v user="$user" '
        {
            line = $0
            sub(/^[ \t]*/, "", line)
            if (line ~ ("^:" display "[ \t]*=")) {
                owner = line
                sub(/^[^=]*=[ \t]*/, "", owner)
                sub(/[ \t]*$/, "", owner)
                if (owner == user) {
                    next
                }
            }
            print
        }
    ' "$input" >"$output"
}

filter_legacy_crontab() {
    local input=$1
    local output=$2
    local display=$3

    awk -v display="$display" '
        {
            line = $0
            if (line ~ /^[ \t]*@reboot[ \t]+\/usr\/bin\/(tiger)?vncserver[ \t]/ &&
                line ~ ("(^|[ \t]):" display "([ \t]|$)")) {
                next
            }
            print
        }
    ' "$input" >"$output"
}

install_user_file() {
    local source=$1
    local destination=$2
    local mode=$3

    if (( EUID == 0 )); then
        run install -o "$TARGET_UID" -g "$TARGET_GID" -m "$mode" \
            "$source" "$destination"
    else
        run install -m "$mode" "$source" "$destination"
    fi
}

backup_user_file() {
    local file=$1
    [[ -f $file ]] || return 0
    local backup
    backup=${file}.bak.$(date +%Y%m%d-%H%M%S)
    as_user cp -a "$file" "$backup"
    log "Backed up $file to $backup"
}

install_packages() {
    local packages=(
        openssh-server
        tigervnc-standalone-server
        tigervnc-tools
        xfce4
        dbus-x11
        xfonts-base
    )
    if [[ $ACCESS_MODE == lan ]]; then
        packages+=(ufw)
    fi

    log "Installing TigerVNC and the XFCE desktop..."
    as_root apt-get update
    as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y \
        --no-install-recommends "${packages[@]}"
}

configure_password() {
    as_user mkdir -p "$VNC_DIR"
    if [[ -f $VNC_PASSWORD && $RESET_PASSWORD == false ]]; then
        log "Keeping the existing VNC password."
        return 0
    fi

    [[ $DRY_RUN == false ]] ||
        {
            log "+ vncpasswd $VNC_PASSWORD"
            return 0
        }

    log "Set a unique VNC password when prompted."
    as_user vncpasswd "$VNC_PASSWORD"
    as_user chmod 600 "$VNC_PASSWORD"
}

configure_vnc() {
    local rendered startup
    new_temp_file rendered
    new_temp_file startup
    render_vnc_config "$VNC_CONFIG" "$rendered" "$GEOMETRY" "$ACCESS_MODE"
    render_vnc_startup "$startup"

    if [[ $DRY_RUN == true ]]; then
        log "Would install this managed configuration in $VNC_CONFIG:"
        sed -n "/^${MANAGED_BEGIN}$/,/^${MANAGED_END}$/p" "$rendered"
        log "Would install the managed XFCE startup script at $VNC_STARTUP"
        return 0
    fi

    if [[ ! -f $VNC_STARTUP ]] || ! cmp -s "$startup" "$VNC_STARTUP"; then
        if [[ -f $VNC_STARTUP ]]; then
            backup_user_file "$VNC_STARTUP"
        fi
        install_user_file "$startup" "$VNC_STARTUP" 700
    fi

    if [[ ! -f $VNC_CONFIG ]] || ! cmp -s "$rendered" "$VNC_CONFIG"; then
        backup_user_file "$VNC_CONFIG"
        install_user_file "$rendered" "$VNC_CONFIG" 600
    else
        log "TigerVNC user configuration is already current."
    fi
}

configure_display_mapping() {
    local existing_owner
    existing_owner=$(mapping_owner "$USERS_FILE" "$DISPLAY_NUMBER")
    if [[ -n $existing_owner && $existing_owner != "$TARGET_USER" && $FORCE == false ]]; then
        die "display :$DISPLAY_NUMBER is assigned to $existing_owner; choose another display or use --force"
    fi

    local rendered
    new_temp_file rendered
    render_users_file "$USERS_FILE" "$rendered" "$DISPLAY_NUMBER" "$TARGET_USER"

    if [[ $DRY_RUN == true ]]; then
        log "Would assign display :$DISPLAY_NUMBER to $TARGET_USER in $USERS_FILE"
        return 0
    fi

    if [[ ! -f $USERS_FILE ]] || ! cmp -s "$rendered" "$USERS_FILE"; then
        if [[ -f $USERS_FILE ]]; then
            as_root cp -a "$USERS_FILE" \
                "${USERS_FILE}.bak.$(date +%Y%m%d-%H%M%S)"
        fi
        as_root install -m 644 "$rendered" "$USERS_FILE"
    else
        log "TigerVNC display assignment is already current."
    fi
}

remove_legacy_cron_job() {
    local current filtered backup
    new_temp_file current
    new_temp_file filtered

    if ! as_user crontab -l >"$current" 2>/dev/null; then
        return 0
    fi

    filter_legacy_crontab "$current" "$filtered" "$DISPLAY_NUMBER"
    cmp -s "$current" "$filtered" && return 0

    backup=$VNC_DIR/crontab.before-ugv-vnc.$(date +%Y%m%d-%H%M%S)
    if [[ $DRY_RUN == true ]]; then
        log "Would remove the legacy VNC @reboot job and back up the crontab to $backup"
        return 0
    fi

    install_user_file "$current" "$backup" 600
    as_user crontab "$filtered"
    log "Migrated the legacy VNC cron job to systemd; backup: $backup"
}

configure_firewall() {
    local rule_comment=ugv-vnc-display-$DISPLAY_NUMBER
    local previous_access previous_cidr
    previous_access=$(state_value access)
    previous_cidr=$(state_value allow_from)

    if [[ $previous_access == lan && -n $previous_cidr ]] &&
        [[ $ACCESS_MODE != lan || $previous_cidr != "$ALLOW_FROM" ]]; then
        validate_cidr "$previous_cidr"
        if command -v ufw >/dev/null; then
            as_root ufw --force delete allow from "$previous_cidr" \
                to any port "$PORT" proto tcp || true
        fi
    fi

    [[ $ACCESS_MODE == lan ]] || return 0

    as_root ufw allow from "$ALLOW_FROM" to any port "$PORT" \
        proto tcp comment "$rule_comment"
}

require_active_firewall_for_lan() {
    [[ $ACCESS_MODE == lan ]] || return 0

    if [[ $DRY_RUN == true ]]; then
        log "+ require active UFW before exposing TCP port $PORT"
        return 0
    fi

    local status
    status=$(as_root ufw status 2>/dev/null || true)
    grep -q '^Status: active' <<<"$status" ||
        die "LAN mode requires active UFW. First allow SSH, then enable it: sudo ufw allow OpenSSH && sudo ufw enable"
}

write_state_file() {
    local state
    new_temp_file state
    {
        printf 'display=%s\n' "$DISPLAY_NUMBER"
        printf 'port=%s\n' "$PORT"
        printf 'access=%s\n' "$ACCESS_MODE"
        printf 'allow_from=%s\n' "$ALLOW_FROM"
    } >"$state"

    if [[ $DRY_RUN == true ]]; then
        log "Would write installation state to $STATE_FILE"
    else
        install_user_file "$state" "$STATE_FILE" 600
    fi
}

stop_legacy_server() {
    if as_user vncserver -list 2>/dev/null |
        awk -v display="$DISPLAY_NUMBER" '$1 == display { found = 1 } END { exit !found }'; then
        log "Stopping the existing manually started VNC server on :$DISPLAY_NUMBER..."
        as_user vncserver -kill ":$DISPLAY_NUMBER"
    fi
}

start_service() {
    if [[ $DRY_RUN == true ]]; then
        as_root systemctl daemon-reload
        as_root systemctl stop "$SERVICE_UNIT"
        log "+ vncserver -kill :$DISPLAY_NUMBER  # only if a legacy session is running"
        as_root systemctl enable --now "$SERVICE_UNIT"
        return 0
    fi

    [[ -f /lib/systemd/system/tigervncserver@.service ||
        -f /usr/lib/systemd/system/tigervncserver@.service ]] ||
        die "the TigerVNC systemd service was not installed"

    [[ -f /usr/share/xsessions/xfce.desktop ]] ||
        die "the XFCE session is unavailable in /usr/share/xsessions"

    as_root systemctl daemon-reload
    as_root systemctl stop "$SERVICE_UNIT" 2>/dev/null || true
    stop_legacy_server
    as_root systemctl enable --now "$SERVICE_UNIT"

    if [[ $DRY_RUN == false ]] && ! systemctl is-active --quiet "$SERVICE_UNIT"; then
        systemctl --no-pager --full status "$SERVICE_UNIT" || true
        die "TigerVNC failed to start; inspect the status above"
    fi
}

show_connection_instructions() {
    local host
    host=$(hostname -f 2>/dev/null || hostname)

    printf '\nTigerVNC is configured on display :%s (TCP port %s).\n' \
        "$DISPLAY_NUMBER" "$PORT"
    if [[ $ACCESS_MODE == local ]]; then
        cat <<EOF

From another computer, create an SSH tunnel:
  ssh -L $PORT:localhost:$PORT $TARGET_USER@$host

Keep that SSH session open, then connect the VNC viewer to:
  localhost:$DISPLAY_NUMBER
EOF
    else
        cat <<EOF

Connect from the allowed network ($ALLOW_FROM) to:
  $host:$DISPLAY_NUMBER
EOF
    fi
}

install_vnc() {
    check_platform
    confirm_install

    if (( EUID != 0 )); then
        as_root -v
    fi

    install_packages
    if [[ $DRY_RUN == false ]]; then
        command -v vncpasswd >/dev/null || die "vncpasswd was not installed"
        command -v vncserver >/dev/null || die "vncserver was not installed"
    fi

    require_active_firewall_for_lan
    configure_password
    configure_vnc
    configure_display_mapping
    remove_legacy_cron_job
    configure_firewall
    write_state_file
    start_service
    show_connection_instructions
}

state_value() {
    local key=$1
    [[ -f $STATE_FILE ]] || return 0
    awk -F= -v key="$key" '$1 == key { sub(/^[^=]*=/, ""); print; exit }' "$STATE_FILE"
}

remove_firewall_rule() {
    local saved_access saved_cidr
    saved_access=$(state_value access)
    saved_cidr=$(state_value allow_from)

    [[ $saved_access == lan && -n $saved_cidr ]] || return 0
    validate_cidr "$saved_cidr"

    if command -v ufw >/dev/null; then
        as_root ufw --force delete allow from "$saved_cidr" to any port "$PORT" proto tcp ||
            warn "could not remove the managed UFW rule"
    fi
}

remove_managed_config() {
    [[ -f $VNC_CONFIG ]] || return 0

    local other_state
    for other_state in "$VNC_DIR"/ugv-vnc-display-*.state; do
        if [[ -e $other_state && $other_state != "$STATE_FILE" ]]; then
            log "Keeping the shared TigerVNC configuration because another managed display exists."
            return 0
        fi
    done

    local rendered
    new_temp_file rendered
    strip_managed_block "$VNC_CONFIG" "$rendered"

    if cmp -s "$rendered" "$VNC_CONFIG"; then
        return 0
    fi

    if [[ $DRY_RUN == true ]]; then
        log "Would remove the managed block from $VNC_CONFIG"
        return 0
    fi

    backup_user_file "$VNC_CONFIG"
    install_user_file "$rendered" "$VNC_CONFIG" 600
}

remove_managed_startup() {
    [[ -f $VNC_STARTUP ]] || return 0
    grep -Fq "$STARTUP_MARKER" "$VNC_STARTUP" || return 0

    local other_state
    for other_state in "$VNC_DIR"/ugv-vnc-display-*.state; do
        if [[ -e $other_state && $other_state != "$STATE_FILE" ]]; then
            return 0
        fi
    done

    if [[ $DRY_RUN == true ]]; then
        log "Would remove $VNC_STARTUP"
    else
        backup_user_file "$VNC_STARTUP"
        as_user rm -f "$VNC_STARTUP"
    fi
}

remove_display_mapping_if_owned() {
    [[ -f $USERS_FILE ]] || return 0

    local owner rendered
    owner=$(mapping_owner "$USERS_FILE" "$DISPLAY_NUMBER")
    [[ $owner == "$TARGET_USER" ]] || return 0

    new_temp_file rendered
    remove_users_mapping "$USERS_FILE" "$rendered" "$DISPLAY_NUMBER" "$TARGET_USER"

    if [[ $DRY_RUN == true ]]; then
        log "Would remove display :$DISPLAY_NUMBER from $USERS_FILE"
    else
        as_root cp -a "$USERS_FILE" \
            "${USERS_FILE}.bak.$(date +%Y%m%d-%H%M%S)"
        as_root install -m 644 "$rendered" "$USERS_FILE"
    fi
}

uninstall_vnc() {
    check_platform

    if [[ $ASSUME_YES == false && $DRY_RUN == false ]]; then
        read -r -p "Disable VNC display :$DISPLAY_NUMBER for $TARGET_USER? [y/N] " reply
        [[ $reply == y || $reply == Y ]] || die "uninstall cancelled"
    fi

    if (( EUID != 0 )); then
        as_root -v
    fi

    as_root systemctl disable --now "$SERVICE_UNIT" 2>/dev/null || true
    remove_display_mapping_if_owned
    remove_firewall_rule
    remove_managed_config
    remove_managed_startup
    if [[ $DRY_RUN == false ]]; then
        as_user rm -f "$STATE_FILE"
    else
        log "Would remove $STATE_FILE"
    fi
    as_root systemctl daemon-reload

    log "VNC display :$DISPLAY_NUMBER was disabled."
    log "Packages and $VNC_PASSWORD were kept to avoid deleting user data."
}

show_status() {
    printf 'User: %s\nDisplay: :%s\nPort: %s\nService: %s\n\n' \
        "$TARGET_USER" "$DISPLAY_NUMBER" "$PORT" "$SERVICE_UNIT"

    systemctl --no-pager --full status "$SERVICE_UNIT" || true
    printf '\nListeners on TCP port %s:\n' "$PORT"
    if command -v ss >/dev/null; then
        ss -ltnp 2>/dev/null | awk -v port=":$PORT" 'index($4, port) { print }'
    else
        warn "ss is unavailable; listener check skipped"
    fi
}

main() {
    parse_arguments "$@"
    detect_target_user

    case $ACTION in
        install) install_vnc ;;
        status) show_status ;;
        uninstall) uninstall_vnc ;;
        *) die "unsupported action: $ACTION" ;;
    esac
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    main "$@"
fi
