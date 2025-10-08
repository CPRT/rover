
sudo pip3 install ros2-numpy simple-parsing cupy-cuda12x shapely ruamel.yaml
python3 -m pip install transforms3d scipy -U

cd /tmp
wget https://developer.download.nvidia.com/compute/cusparselt/0.7.0/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb
sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb
sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.7.0/cusparselt-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install libcusparselt0 libcusparselt-dev

pip3 install --no-cache https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl

export VERSION=20
git clone --branch release/0.$VERSION https://github.com/pytorch/vision torchvision
cd torchvision/
export BUILD_VERSION=0.$VERSION.0
echo $BUILD_VERSION 
python3 setup.py install --user

sudo pip3 install opencv-python
pip3 install "numpy<2"


# Note: add jtop to dockerfile for jetsons only
# 