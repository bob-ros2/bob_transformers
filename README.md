# ROS Package bob_transformers
This package integrates `Huggingface Transformers` capabilities into `ROS`.

Related websites:

* https://huggingface.co
* https://ros.org

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
* https://huggingface.co/docs/huggingface_hub/package_reference/environment_variables

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

```YAML
services:
  ros:
    build:
      context: .
    volumes:
      - /home/ros/.cache/huggingface:/root/.cache/huggingface
    command:
      - /bin/bash
      - -c
      - |
        cd /ros_ws
        source ./install/setup.bash
        tail -f /dev/null
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```

## ROS Node TTI
