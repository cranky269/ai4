# Source from Bash after the ROS/catkin setup. Keep ROS on the system Python.
AI4_SENSOR_PACKAGES="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/sensor_python"
if [[ -d "$AI4_SENSOR_PACKAGES" ]]; then
  case ":${PYTHONPATH:-}:" in
    *":$AI4_SENSOR_PACKAGES:"*) ;;
    *) export PYTHONPATH="$AI4_SENSOR_PACKAGES${PYTHONPATH:+:$PYTHONPATH}" ;;
  esac
fi
unset AI4_SENSOR_PACKAGES
