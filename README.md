# ROS Package [bob_transformers](https://github.com/bob-ros2/bob_transformers)
This package integrates `Huggingface Transformers` capabilities into `ROS`.

Related websites:

* [https://huggingface.co](https://huggingface.co)
* [https://ros.org](https://ros.org)

## Dependencies
The HF Transformer library depends from a lot of other Python software packages, depending on what want to be done with it. All needed packages for this repository can be found in the `requirments.txt`

```bash
# install Python packages from file
pip3 install -r requirements.txt
```

## Huggingface Model Folder
It's recommendet to maintain a Huggingface model folder to save the downloaded models and to store your HF token. The token is needed if you want access and use models where you need to sign an agreement on the HF websife.\
The config is located here `~/.cache/huggingface/` and will be created at first start of a HF component. 

Further information about the config folder can be found here:
* [https://huggingface.co/docs/huggingface_hub/package_reference/environment_variables](https://huggingface.co/docs/huggingface_hub/package_reference/environment_variables)

### Huggingface Token Handling

```bash
# The Huggingface token can be placed in this file: ~/.cache/huggingface/token
# or use python from command line to store the token
# if the folder does not exists it will be created
python3 -c 'from huggingface_hub import HfFolder; HfFolder.save_token("your_hf_token")'

```

## Building Package

Replace `<colcon_ws>` with your own workspace folder.

```bash
cd <colcon_ws>/src
git clone https://github.com/bob-ros2/bob_transformers.git
# install Python packages if not yet existing
pip3 install -r bob_transformers/requirements.txt
cd ..
colcon build
. install/setup.bash
```

## Docker Support

A precreated `Docker` image with neccesary libraries is available at `ghcr.io`  [here](https://github.com/bob-ros2/bob_transformers/pkgs/container/bob-transformers-tti-ros)

### Example compose.yaml

```YAML
services:
  ros:
    image: ghcr.io/bob-ros2/bob-transformers-tti-ros:latest-humble
    volumes:
      - /home/ros/.cache/huggingface:/root/.cache/huggingface
    command:
      - /bin/bash
      - -c
      - |
        cd /ros_ws
        source /opt/ros/humble/setup.bash
        source ./install/setup.bash
        ros2 run bob_transformers tti
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]

networks:
  default:
    name: bobnet
    external: true
```

## ROS Node TTI
Text to image ROS node

### Models
Tested models
* [`CompVis/stable-diffusion-v1-4`](https://huggingface.co/CompVis/stable-diffusion-v1-4)
* [`stabilityai/stable-diffusion-2`](https://huggingface.co/stabilityai/stable-diffusion-2)
* [`stabilityai/stable-diffusion-xl-base-1.0`](https://huggingface.co/stabilityai/stable-diffusion-xl-base-1.0)

#### Supported StableDiffusion Pipelines
* `StableDiffusionPipeline`
* `StableDiffusionXLPipeline`

### Parameter

> **Parameter name**: frame_rate\
> **Type**: integer\
> **Description**: Framerate to resent image. Environment variable TTI_FRAME_RATE. Default: 1

> **Parameter name**: model_id\
> **Type**: string\
> **Description**: The Huggingface model_id to be used. Environment variable TTI_MODEL_ID. Default: CompVis/stable-diffusion-v1-4

> **Parameter name**: pipeline_type\
> **Type**: string\
> **Description**: Prompt format. Environment variable TTI_PIPELINE_TYPE. Default: StableDiffusionPipeline

> **Parameter name**: result_image\
> **Type**: string\
> **Description**: When provided, where to store the result image. Environment variable TTI_RESULT_IMAGE. Default: ''

> **Parameter name**: show\
> **Type**: string\
> **Description**: Wether to show produced image. Environment variable TTI_SHOW. Default: false

### Subscribed Topics

> ~tti_in (std_msgs/msg/String)\
TTI input String topic.

### Published Topics

> ~image_raw (sensor_msgs/msg/Image)\
Raw image output topic.

> ~tti_image: (bob_msgs/msg/TTImage)\
TTI image with payload output topic.
