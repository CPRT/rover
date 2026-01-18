# Switch to 'devel' tag to force a fresh download
FROM pytorch/pytorch:2.1.2-cuda11.8-cudnn8-devel

WORKDIR /usr/src/app

# Install system dependencies
RUN apt-get update && apt-get install -y libgl1-mesa-glx libglib2.0-0

# Install Ultralytics
RUN pip install ultralytics
