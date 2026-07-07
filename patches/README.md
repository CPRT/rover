# Patches to Jetson code

At time of completeting the migration - Jetpack 7.2 had poor docker support with no dedicated jetpack images. So I applied and built the libraries on host and copied in the library files. Deepstream was also not fully supported with Jetpack 7.2 further complicating thing. In a perfect world this build would happen as part of the docker build but here we are ...

## nvcompositor
Took the patch from https://forums.developer.nvidia.com/t/nvcompositor-sigsegv-on-jetpack7-0/371694/8

### Commands to build the library on a jetson host
```bash
wget https://developer.nvidia.com/downloads/embedded/L4T/r39_Release_v2.0/sources/public_sources.tbz2
tar -xvf public_sources.tbz2 
tar -xvf Linux_for_Tegra/source/gst-nvcompositor_src.tbz2
patch -p1 < patches/nvcompositor.patch
cd gst-nvcompositor/
make
```