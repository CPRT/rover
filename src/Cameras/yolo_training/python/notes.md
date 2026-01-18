This command created v1:

```bash
python3 split_yolo_datasets.py --config config/build_computer_mallet_v1.yaml --split 70_20_10 --output-dir ~/erik_yolo/mallet_v1
```

I also ran
pip3 install roboflow scikit-learn



This is how I trained:
First grabbed the docker image:
```bash
docker pull ultralytics/ultralytics:latest
```

Then 
```bash
docker run -it --rm \
  --ipc=host \
  --gpus all \
  --cpus 12 \
  --shm-size 8g \
  -v /home/cprt/erik_yolo:/home/cprt/erik_yolo \
  -w /home/cprt/erik_yolo \
  ultralytics/ultralytics:latest \
  yolo train \
  model=yolov9c.pt \
  data=/home/cprt/erik_yolo/mallet_v1/data.yaml \
  epochs=10 \
  img=640 \
  batch=16 \
  workers=8 \
  device=0 \
  project=/home/cprt/erik_yolo/mallet_v1/runs \
  name=exp_titan_x
```


```bash
sudo nice -n -15 ionice -c 2 -n 0 docker run -it --rm \
  --ipc=host \
  --gpus all \
  --cpus 12 \
  --shm-size 16g \
  -v /home/cprt/erik_yolo:/home/cprt/erik_yolo \
  -w /home/cprt/erik_yolo \
  ultralytics/ultralytics:latest \
  yolo train \
  model=yolov9c.pt \
  data=/home/cprt/erik_yolo/mallet_v1/data.yaml \
  epochs=10 \
  img=640 \
  batch=16 \
  workers=8 \
  device=0 \
  cache=True \
  project=/home/cprt/erik_yolo/mallet_v1/runs \
  name=exp_titan_x_high_prio
```

```bash
sudo nice -n -15 ionice -c 2 -n 0 docker run -it --rm \
  --ipc=host \
  --gpus all \
  --cpus 12 \
  --shm-size 16g \
  -v /home/cprt/erik_yolo:/home/cprt/erik_yolo \
  -w /home/cprt/erik_yolo \
  pytorch/pytorch:2.1.2-cuda11.8-cudnn8-runtime \
  sh -c "pip install ultralytics && yolo train model=yolov9c.pt data=/home/cprt/erik_yolo/mallet_v1/data.yaml epochs=10 imgsz=640 batch=16 workers=8 device=0 cache=True project=/home/cprt/erik_yolo/mallet_v1/runs name=exp_titan_x_compatible"
```



```bash
sudo nice -n -15 ionice -c 2 -n 0 docker build -t yolov9_titan_x_training .
```

```bash
sudo docker run -it --rm \
  --ipc=host \
  --gpus all \
  --cpus 12 \
  --shm-size 16g \
  -v /home/cprt/erik_yolo:/home/cprt/erik_yolo \
  -w /home/cprt/erik_yolo \
  yolov9_titan_x_training \
  yolo train \
  model=yolov9c.pt \
  data=/home/cprt/erik_yolo/mallet_v1/data.yaml \
  epochs=10 \
  imgsz=640 \
  batch=16 \
  workers=8 \
  device=0 \
  cache=True \
  project=/home/cprt/erik_yolo/mallet_v1/runs \
  name=exp_titan_x_final
```
