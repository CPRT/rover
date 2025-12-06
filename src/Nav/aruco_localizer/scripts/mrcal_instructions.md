# mrcal Calibration Instructions
These instructions explain how to calibrate a camera and validate the calibration using mrcal and then converting it into a opencv format.

NOTE: For these instructions, you must use a mrcal calibration board not an opencv one. It is either a 10x10 or 14x14 and has longer rectangles on the outside black rectangles. Get the print file for the calibration board from this repo: https://github.com/dkogan/mrgingham/. Depending on what you want it has these filenames: chessboard.10x10.fig, chessboard.10x10.pdf, chessboard.14x14.fig, chessboard.14x14.pdf. Here's a direct link to the 10x10 pdf: https://github.com/dkogan/mrgingham/blob/master/chessboard.10x10.pdf.

### Install nessecary apt packages
```bash
sudo apt update && \
sudo apt install -y \
    mrcal \
    libmrcal-dev \
    python3-mrcal \
    mrgingham \
    libmrgingham-dev \
    feedgnuplot \
    vnlog \
    libvnlog-dev \
    libvnlog-perl \
    python3-vnlog \
    libopencv-dev \
    libboost-dev \
    pkg-config \
    mawk \
    perl \
    python3-all \
    python3-all-dev \
    python3-numpy
```

### Record images
```bash
python3 record_images_for_calibration.py \
  --checkerboard_width 10 \
  --checkerboard_height 10 \
  --output_dir ~/Downloads/CalibrationImages \
  --width 1280 \
  --height 720 \
  --camera /dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_B0561_16MP_Klarity_SN0001-video-index0 \
  --name ErikKlarityArducam
```

### Validate calibration using mrcal
Open a terminal in the directory with only the images recorded from the above script.
Run this to generate a corners.vnl file:
```bash
mrgingham --jobs 7 --gridn 10 '*.png'  > corners.vnl
```

The following command will display all corners detected on one camera on a graph. Check that there is good coverage, especially in the corners/edges. If there is not, you can continue validating the camera model but may need to go take more images, append them to your current directory of images and remake the corners.vnl file.
```bash
cat corners.vnl | vnl-filter -p x,y 'filename ~ ".png"' | feedgnuplot --title "All Corners Visualized" --domain --square --set 'xrange [0:1280] noextend' --set 'yrange [720:0] noextend'
```

Run this to see how many images have good/useable corner data:
```bash
< corners.vnl vnl-filter --has x -p filename | uniq | grep -v '#' | wc -l
```

Run the following commands to setup variables for calibrating the camera model. Update these to match your calibration board.
Make sure to measure the size of the squares as accurately as possible.
```bash 
GRID_WIDTH=10
GRID_SPACING_METERS=0.016
```

Run the following command to calibrate the camera and produce a mrcal camera model file. Note the focal is set as a first best
guess to help the model converge on the correct focal length This command might take a bit of time depending on how many images you have.
```bash
mrcal-calibrate-cameras \
    --corners-cache corners.vnl \
    --lensmodel LENSMODEL_OPENCV8 \
    --focal 1000 \
    --object-spacing $GRID_SPACING_METERS \
    --object-width-n $GRID_WIDTH \
    '*.png'
```

Run the following command to visualize the calibration data in a 3D graph. This graph places the camera on the ground facing
upwards, and draws all the calibration boards where they were captured relative to the camera (in meters):
```bash
mrcal-show-geometry camera-0.cameramodel \
    --show-calobjects \
    --unset key \
    --set 'xyplane 0' \
    --set 'view 80,30,1.5' \
    --set 'xrange [-0.8:0.8]' \
    --set 'yrange [-0.8:0.8]' \
    --set 'zrange [0.0:1.2]'
```

Run the following command to visualize the distribution of errors for corner detection as a bar graph with the expected distribution
of errors as a green line. We would like to see a normal distribution centered around 0.
```bash
mrcal-show-residuals \
    --histogram \
    --set 'xrange [-2:2]' \
    --unset key \
    --binwidth 0.1 \
    camera-0.cameramodel
```

Run this command to see the worst fitted image. Likely has motion blurr or too far away. mrcal values this image the least of all images. The magnitude of the vectors indicate the quantity of error for a given corner in relation to the whole model.
```bash
mrcal-show-residuals-board-observation \
    --from-worst \
    --vectorscale 100 \
    --circlescale 0.5 \
    --set 'cbrange [0:2]' \
    camera-0.cameramodel \
    0
```

Now let’s look at the twentieth worst image. Run the following command:
```bash
mrcal-show-residuals-board-observation \
    --from-worst \
    --vectorscale 100 \
    --circlescale 0.5 \
    --set 'cbrange [0:2]' \
    camera-0.cameramodel \
    20
```

Run the following command to visualize the magnitude of the residuals (measurements). We are looking for a very consistent
spread of points, lower values means the model is much better fitting. Look out for where the model doesn't fit as well, such as in
the center or if the corners or edges of the camera are worse.
```bash
mrcal-show-residuals \
    --magnitudes \
    --set 'cbrange [0:2]' \
    camera-0.cameramodel
```

Run the following command to plot the direction of the residuals instead of the magnitudes. Ideally, the graph shown should
represent random noise and no discernible colour pattern in the dots. If there are discernible clusters or rings of colour then the
lens model is not able to accurately fit the data (which typically happens using a pinhole model with a fish eye lens)
```bash
mrcal-show-residuals \
    --directions \
    --unset key \
    camera-0.cameramodel
```

### Convert mrcal to opencv or ros2
Run this script to get opencv values from the mrcal calibration. The values it prints are usable directly:
```
python3 convert_mrcal_to_opencv_ros.py --input camera-0.cameramodel --output camera_info.yaml --name camera
```

This script also generates a yaml file that can be used by ros2 to publish a camera info topic.