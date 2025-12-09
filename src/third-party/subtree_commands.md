# Git subtree for third party repositories
Explains what commands were used to add the subtree and what commands to run to grab the latest changes from the sub tree.



# elevation_mapping_cupy
Run this from the root directory (rover) to clone:
```bash
git subtree add --prefix src/third-party/elevation_mapping_cupy https://github.com/leggedrobotics/elevation_mapping_cupy.git ros2 --squash
```

To pull new changes from the remote repository (run from root directory which is rover):
```bash
git subtree pull --prefix src/third-party/elevation_mapping_cupy https://github.com/leggedrobotics/elevation_mapping_cupy.git ros2 --squash
```
