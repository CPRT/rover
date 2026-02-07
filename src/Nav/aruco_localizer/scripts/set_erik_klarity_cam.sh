#!/bin/bash

# ==========================================
# Arducam B0561
# ==========================================

CAM_DEV="/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_B0561_16MP_Klarity_SN0001-video-index0"

# --- TARGET SETTINGS ---
# EXPOSURE STRATEGY: 
# We drastically lowered this from 500 to 50.
# If still blurry, lower to 20. If pitch black, raise to 100.
EXPOSURE_VAL=166     

# GAIN STRATEGY:
# We raised this from 0 to 40 to compensate for the dark image.
# Max is 100. If image is too dark, increase this. If too noisy/white, decrease.
GAIN_VAL=40

# FOCUS STRATEGY:
# Locked focus from previous step (adjust as needed)
FOCUS_VAL=350        

BRIGHTNESS_VAL=140   
WIDTH=1280
HEIGHT=720

# Check if camera exists
if [ ! -e "$CAM_DEV" ]; then
    echo "Error: Camera device not found at $CAM_DEV"
    exit 1
fi

echo "Configuring Arducam for HIGH SPEED MOTION at $CAM_DEV..."

# 1. Set Resolution
v4l2-ctl -d "$CAM_DEV" --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=MJPG

# 2. Disable Autofocus & Set Manual Focus
v4l2-ctl -d "$CAM_DEV" -c focus_automatic_continuous=0
v4l2-ctl -d "$CAM_DEV" -c focus_absolute=$FOCUS_VAL

# 3. Disable Auto-Exposure (Mode 1 = Manual)
v4l2-ctl -d "$CAM_DEV" -c auto_exposure=1

# 4. SET EXPOSURE (The most important step for blur)
v4l2-ctl -d "$CAM_DEV" -c exposure_time_absolute=$EXPOSURE_VAL
echo "Exposure set to FAST ($EXPOSURE_VAL) to stop blur."

# 5. SET GAIN (To fix the brightness)
v4l2-ctl -d "$CAM_DEV" -c gain=$GAIN_VAL
echo "Gain boosted to $GAIN_VAL to brighten image."

# 6. Set Brightness
v4l2-ctl -d "$CAM_DEV" -c brightness=$BRIGHTNESS_VAL

echo "Done. If image is too dark, increase GAIN in the script (max 100)."

echo "----------------------------------------"
v4l2-ctl -d "$CAM_DEV" --list-ctrls | grep -E "focus|exposure|brightness"
echo "----------------------------------------"