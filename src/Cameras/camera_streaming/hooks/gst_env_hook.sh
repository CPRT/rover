#!/bin/bash

_this_script_dir=$(dirname "${BASH_SOURCE[0]}")

_package_share_dir=$(dirname "${_this_script_dir}")
_package_install_prefix=$(dirname $(dirname "${_package_share_dir}"))
_gstreamer_plugin_path="${_package_install_prefix}/lib/gstreamer-1.0"


# --- Update GST_PLUGIN_PATH ---
if [ -n "${GST_PLUGIN_PATH}" ]; then
  export GST_PLUGIN_PATH="${_gstreamer_plugin_path}:${GST_PLUGIN_PATH}"
else
  export GST_PLUGIN_PATH="${_gstreamer_plugin_path}"
fi
