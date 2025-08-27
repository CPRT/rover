#!/bin/bash

# -------------------------------------------------
# Persistent bash history
# -------------------------------------------------
if [[ ! -L ~/.bash_history ]]; then
  rm -f ~/.bash_history
  ln -s "$BASH_HISTORY_FILE" ~/.bash_history
fi

if [ -f "$ELEVATION_MAPPING_DIR/install/local_setup.bash" ]; then
  source "$ELEVATION_MAPPING_DIR/install/local_setup.bash"
fi

# Restart udev daemon
sudo service udev restart

$@
