# Sourced from ~/.bashrc in every interactive container shell (see Dockerfile).
# Loads shared setup/helpers, then the role-specific file (uav.sh or gcs.sh)
# selected by ROLE in .env, and applies its `disros` default.

_bashrc_d="$(dirname "${BASH_SOURCE[0]}")"

source "$_bashrc_d/common.sh"

case "$ROLE" in
  uav) source "$_bashrc_d/uav.sh" ;;
  gcs) source "$_bashrc_d/gcs.sh" ;;
  *)
    echo "ros.sh: ROLE is not 'uav' or 'gcs' (got: '${ROLE:-<unset>}') in .env — defaulting to gcs." >&2
    source "$_bashrc_d/gcs.sh"
    ;;
esac

unset _bashrc_d
disros
