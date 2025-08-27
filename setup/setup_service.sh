#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
CURRENT_DIR="$(pwd)"
CURRENT_USER=$(whoami)

if [ "$SCRIPT_DIR" == "$CURRENT_DIR" ] || [ "$SCRIPT_DIR" == "." ]; then
    echo "The script is being run from its own directory."
else
    echo "The script is NOT being run from its own directory."
    exit 1
fi

sudo cp start_rover.service /etc/systemd/system/start_rover.service
sudo sed -i "s|User=%i|User=${CURRENT_USER}|" /etc/systemd/system/start_rover.service
sudo chmod 644 /etc/systemd/system/start_rover.service
sudo ln -s $PWD/start_rover.sh /usr/local/bin/start_rover.sh

cd ..
sudo tee /opt/ros/humble/cprt_setup.bash > /dev/null << EOF
#!/bin/bash
source $PWD/install/setup.bash
EOF

sudo systemctl daemon-reload
sudo systemctl enable start_rover.service
sudo systemctl start start_rover.service
