# mrcal_record_images.py gemini prompt:
Can you generate a python script that uses argparse to have the following:

--camera option which takes a string and sets a variable called camera_input_device which defaults to "0". 
--width option which takes an integer and sets width which defaults to None
--height option which takes an integer and sets height which defaults to None
--checkerboard_width option which takes an integer
--checkboard_height option that takes an integer
--output_dir option to specify where to save the images
--record_period option to specify the period which images will be saved when possible
--name option to specific a name for files

Then I want it to open a camera with opencv, it should only try to set a width/height if those parameters are not null otherwise it just lets the camera do a resolution that it happens to pick. Then I want the script to read the latest image from the camera, I want it to detect the corners of my calibration checkerboard using opencv, then if the checkboard is detected it should draw the corners onto the image and then use imshow to display it. Then I also want it to store images that have a detected checkerboard but limited so that it can't store an every faster than every 0.5 seconds. It should name the images frame-#####-10x10-1920x1080.png where the ##### is the frame number, the 10x10 is a possible checkboard width and hiehgt, and 1920x1080 is the resolution the camera is recording, and where frame is the default option when name is specify otherwise the frame is replaced by the name option. If the name is set, then the output directory should have another directory appended to it with the same format as naming the files but without the frame count. It should also only increment ##### number when images are saved.


I did more prompting to add more features to the script it made


# Asking claude sonnet 4.5 to generate a ros2 node for aruco board stuff
Can you write a new python node into the aruco_localizer package. I want this node to use pairs of aruco tags as boards, then use solve pnp of 8 corners from pairs of tags to produce a really good estimate of where the board is relative to the camera and publish that to a topic. I have yaml files that can be loaded in order to create the boards for each unique pair of tags that can be see in the config/aruco_boards directory of the aruco_localizer package. I want the node to subscribe to an image topic and camera_info topic then convert to opencv image then process it. There is old code available in the old_design directory of the aruco_localizer package which shows the idea of how to use the aruco cv2 library and how to use solvepnp though I don't have experience with doing a aruco board so I dont know how that will be different, I think in this case solvepnp will want 8 2d corners and then 8 3d corners. The node should publish each pose of aruco boards in the camera's frame and should publish it as a ArucoBoard.msg which will need to be added to the src/interfaces package, you can reference the ArucoMarkers.msg to make it. Anything that has magic numbers or would benefit from parameters should use the ros2 parameters, and then create a yaml file in config to set all the parameters to reasonable values. Can you also create a launch file for this node which loads the params from that yaml and tells the ros2 node all the aruco_boards. 

Other Notes:
Use cv2.aruco.estimatePoseBoard to find the pose of the Board frame relative to the Camera.
Use cv_bridge
Include a try-except block for the image processing.
Publish a debug image to /aruco_debug showing the detected axis drawn on the board.
cv2 version is '4.12.0'



# Next prompt for creating a node to publish the images and camera_info
Can you now also write a python node that will open a camera with opencv, publish it to a ros2 topic so that the aruco_board_detector_node.py can subscribe to it, and also read the camera instrincs from src/Nav/aruco_localizer/config/camera_intrinsics which should already be in the perfect form to publish camera_info topic.
IT should use ros2 parameters where possible and have a config file and launch file. Ideally the device input is a string which is passed right to the opencv video capture and make sure it uses v4l2. 


# Erik note:
I had to run this to make opencv work with python in the docker container
```bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libcairo.so.2
```
