# Training a yolov9 model on colab - Documentation
Date: Jan 23, 2026

This is the link to the colab doc that I used to train yolo models with: https://colab.research.google.com/drive/15WczeyszzAbw9A6FDl7nGmmtxV1--4VL?usp=sharing
I paid $10 for the montly subscription so that it would train my ~10,000 image dataset within 4 hours. Google colab requires keeping the browser active and running or it will time out. It can't run tasks in the background. The free version would have taken ~20 hours which wouldn't be possible. 

This colab doc grabs scripts from this git repo that are explained below. You can do the training locally as well if you have a nvidia gpu and install all the nessecary packages. I don't have a nvidia gpu so I used google colab.

## download_roboflow_image_dataset.py
- Downloads roboflow datasets to local file system
- Remaps labels to be the same
- Recieves a yaml config file to set who roboflow datasets and how to remap labels
- See config/colab_mallet_v1.yaml as an example
- Example usage: 
```bash
python3 download_roboflow.py --config config/colab_mallet_v1.yaml --model-format yolov9 --output-dir ~/Downloads/raw_mallet_dataset
```

## split_yolov9_datasets.py
- Recieves the same config file used to download the dataset
- Collects all image pairs from each dataset and does a new combined stratified split
- Example usage:
```bash
python3 split_dataset.py --config config/colab_mallet_v1.yaml --split 70_20_10 --input-dir ~/Downloads/raw_mallet_dataset --output-dir ~/Downloads/combined_mallet_dataset
```

## Yolo training notes
I knew I was training to see a specific orange mallet in an orange desert in the bright sun. So I overrite only a few of the default yolo settings. The values were picked by gemini when I asked it. They are:
- hsv_h lowered to prevent learning purple/off-color mallets
- hsv_s increased to rely on saturated orange color
- hsv_v increased to handle bright sun vs dark shadow in desert
- mosaic kept at 1.0 for better small object detection

I also made it possible to resume the training so if google colab did disconnect, it could spend another 10 minutes downloading and splitting the dataset then resume training from where it left off. It doesn't take long to download/split on the Google servers because it has massive file transfer capabilities and mega good internet so caching this stuff wasn't worth it.

Here is how my colab doc ran yolo:
```python
if not os.path.exists(checkpoint_path):
    # --- SCENARIO B: FRESH START ---
    print(f"🆕 NO CHECKPOINT FOUND at {checkpoint_path}")
    print(f"⭐ Starting FRESH training for '{training_conf['name']}'...")
    print(f"Training parameters: {training_conf}")

    # Initialize new model
    model = YOLO(base_model)

    # Train with parameters from YAML
    results = model.train(
        data=data_yaml_path,
        device=0,  # Force GPU

        # Training Parameters
        epochs=training_conf['epochs'],
        patience=training_conf['patience'],
        imgsz=training_conf['imgsz'],
        # batch=training_conf['batch'],
        batch=64, # A100 GPU (used 39 GB ram out of the 40 GB available to max the usage)
        workers=8, # A100 GPU
        cache=True, # A100 GPU
        project=training_conf['project'],
        name=training_conf['name'],

        # Augmentation / Desert Tuning
        hsv_h=aug_conf['hsv_h'],
        hsv_s=aug_conf['hsv_s'],
        hsv_v=aug_conf['hsv_v'],
        mosaic=aug_conf['mosaic'],
    )

else:
    # --- SCENARIO B: RESUME TRAINING ---
    print(f"🔄 FOUND CHECKPOINT: {checkpoint_path}")
    print(f"🚀 Resuming training for '{training_conf['name']}'...")

    # Load the existing weights (this contains all previous epoch info)
    model = YOLO(checkpoint_path)

    # Resume (no need to pass other args, they are saved in the .pt file)
    results = model.train(resume=True)
```


## Validation
The model has a train, val, test split of images. The train and val are both using during training and so cannot be used to prove the model is a good model. The test split is entirely set aside and used to show the model isn't underfit or overfit to the dataset. My model has around a 0.727 mAP50. mAP50 is a generally detection accuracy metric, anything above 0.6 is probably good enough for this scenario, so 0.727 is great. There are also more metrics that can be used to know if the model is weak in specific areas. These can tell you if maybe the model is consistently have false positives meaning its identifying something that is not a mallet as a mallet. You can learn about precision, recall and others on your own.

## Extra validation for mallet scenario
In my colab doc I linked at the top, I did a fancy extra check.

One of the datasets I found on roboflow had images that were taken in an orange desert. They are pretty competition realistic images. So I wrote code to create a new dataset that had only the ~70 images that were from the orange desert dataset that were also set aside for only testing. I couldn't use the train or val images because a metric that uses images that were also used in training shows nothing. It scored 0.99 mAP50 on these ~70 images. It also had perfect recall and precision so it didn't miss a single mallet in this test.

## onnx export
I trained using the ultralyics yolo package. The onnx export script here (https://github.com/marcoslucianops/DeepStream-Yolo/blob/master/docs/YOLOv9.md) was for yolo models trained with the original authors github repo [WongKinYiu](https://github.com/WongKinYiu/yolov9.git). So this export script didn't work. Instead I used the one that was designed for yolov8 but it worked just fine for yolov9 since ultralytics is good at being cross compatible with different yolos. You can find this script in export_yolo_to_onnx.py or here: https://github.com/marcoslucianops/DeepStream-Yolo/blob/master/utils/export_yoloV8.py

I used these params:
```bash
python3 export_yolo_to_onnx.py -w best.pt -s 640 --batch 1 --simplify --opset 12
```
I could not get this to work on google colab because colab uses a very new version of python3 and yolov9 is years old. So I had to do this locally. I did in a venv so that I could control the python packages very carefully and not mess up my system python package versions. You can use the `requirements.txt` in this directory to run the above command. Just need to do:
```bash
python3 -m venv venv
source venv/bin/activate
python3 -m pip install -r requirements.txt
python3 export_yolo_to_onnx.py -w best.pt -s 640 --batch 1 --simplify --opset 12
```

## Files persisted in google drive
Here is what files were persisted in google drive by the colab doc:
https://drive.google.com/drive/folders/1dblwe87q0ymlDde4oUYf2wdA6bfKbT_T?usp=sharing
