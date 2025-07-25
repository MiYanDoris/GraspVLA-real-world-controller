# Real World Controller for GraspVLA
This repository contains the client code used to control a Franka robot in the real world for [GraspVLA](https://github.com/PKU-EPIC/GraspVLA). 
- Model server code is available [here](https://github.com/PKU-EPIC/GraspVLA).
- Simulation evaluation code is available [here](https://github.com/MiYanDoris/GraspVLA-playground).

**⚠️ CAUTION:
Ensure that the emergency stop button is available. If the robot arm behaves abnormally (e.g., docker images competing for control), press the emergency stop immediately.**

## Hardware Requirements
- **Robot**: A Franka robot. The default Dockerfile supports the latest Franka Research 3. For earlier FER3 versions, use Dockerfile_FER3.
- **Cameras**: Two RGB cameras. We support Intel RealSense D435 and D415i. Any RGB camera with field of view >43° (horizontal and vertical) should work, but requires code modifications.
- **Workstation**: Tested on Ubuntu 20.04. If you use remote server to run the model, GPU is not required on this workstation.


## Installation

1. Install docker (https://docs.docker.com/engine/install/);
2. Run `docker build . -t graspvla_franka_ros:latest`.

## Configuration
### Franka

- **IP**. Modify the `FRANKA_IP` in `demo.env` to your franka desk ip.

    Make sure your workstation ip and franka desk ip are in the same network (e.g., workstation ip = 172.16.0.1 and franka desk ip = 172.16.0.2). For details, see https://frankarobotics.github.io/docs/getting_started.html#setting-up-the-network.

- **FCI Mode**. Visit franka desk in web browser, for example https://172.16.0.2/desk. In top right of the panel, click the `enable fci interface` button. For details, see https://frankarobotics.github.io/docs/getting_started.html#preparing-the-robot-for-fci-usage-in-desk.

- **Extention of Fingers**. To grasp convex-shaped objects more firmly, we extend the original Franka fingers as shown in the figure below. However, our model works robustly with both the original and extended fingers, so you can choose either option based on your needs.

    - To use extensions: Print two copies of `res/extension.STEP` and install them.

    - To use the original fingers: Remove the `--extended_finger` flag in `docker-compose.yml` (this adjusts proprioception accordingly).

      <img src="res/finger_extention.jpg" alt="finger_extension" width="30%" height="30%" style="display: block; margin: 0 auto;">

### Camera
- **Serial Number**. Modify the `FRONT_CAMERA_SERIAL_NUMBER` and `SIDE_CAMERA_SERIAL_NUMBER` in `demo.env` to your camera serial numbers.

    You can find the serial numbers of your Realsense cameras in the RealSense Viewer tool by clicking the 'Info' option at the top of the Viewer's options side-panel.

    <img src="res/camera_setup.jpg" alt="camera_setup_ours" width="70%" height="70%" style="display: block; margin: 0 auto;">

- **Approximate Calibration**. Since camera extrinsics are randomized in our dataset, you only need to approximately match your camera setup to the one shown above. Following the steps below:

  1. Roughly position the two cameras with a tape measure according to the above figure.

  2. Ensure that your table surface is below z=0.2m in robot base frame, as the end effector will automatically move to z=0.2m in the next step.

  3. Run `xhost + && source demo.env && docker compose run --rm calibrate_camera` for further calibration. Once the arm stops moving, adjust your camera poses such that the green reference mask (front_ref/side_ref) aligns with the real-world images in RViz. Also verify that the cameras are level, such that table edges appear roughly horizontal. You can refer to the below image for calibrated status.

    <img src="res/camera_ref.jpg" alt="camera_setup_ours" width="50%" height="50%" style="display: block; margin: 0 auto;">

  4. Run `docker compose down -t 0`.

### Connection to the Model Server
Start the model server following the instructions in [GraspVLA](https://github.com/PKU-EPIC/GraspVLA) and modify the `SERVER_IP` and `SERVER_PORT` in `demo.env` to your server ip and port. If you use remote server, you can set them to be its public address. If you start the server on the same machine as the client, you can set them to be `127.0.0.1` and `6666`.

You can run the following command to validate the server is running. It will return ✓ if the server returns a valid result.
```bash
source demo.env
docker compose run --rm validate_server
```

## Inference
To start inference, run the following commands:
```bash
xhost +
source demo.env
docker compose run --rm main
```

* Remember to run `docker compose down -t 0` after the experiments.

* Start by testing simple cases—like placing 4-5 objects near the center of workspace (x=0.5m, y=0m) on a clean table—to verify pipeline functionality.

* When prompted, type the object name and press enter. The client will auto-complete the instruction as "pick up {object_name}".

* Controls: 
  
  - `p` - pause the robot
  
  - `q` - finish trajectory and reset to initial pose (opens gripper)

* Nonblocking mode:

  - Set `MODE` to `nonblocking` in `demo.env`. 
  
  - Parameters are pre-tuned for ~250ms inference delay. Adjust according to the inference delay in your environment.

* Controller precision decreases near workspace edges, which may reduce success rates. Therefore, begin testing with objects positioned near the center of the workspace.

### Terminal Output
- `observation sent...` - Observation sent to server

- `response received, cost xxx.xxs` - Response received

- Gripper actions and delta translations are printed in console

### Safety Features

- When the robot receives an external force larger than FORCE_LIMIT (default 15N), it will print `large external force or robot abnormal, trying to recover...` and lift by 5cm to avoid collision.

- To avoid objects from falling when opening the gripper (q), enable automatic lowering by adding `--automatically_put_down` flag to `docker-compose.yml`. It will move down the gripper by 10cm before opening the gripper to prevent the object from falling and breaking. If you enable this feature, press `q` after the object is lifted high enough to avoid robot arm colliding with the table.

### Category List
We provide a list of categories we've tested in `res/category_list.txt`. You can start with these verified categories to test the correctness of the pipeline.

## Citation

If you found this repository useful, please consider to cite the following works:

- Our paper:
```bibtex
@article{deng2025graspvla,
    title={GraspVLA: a Grasping Foundation Model Pre-trained on Billion-scale Synthetic Action Data}, 
    author={Shengliang Deng and Mi Yan and Songlin Wei and Haixin Ma and Yuxin Yang and Jiayi Chen and Zhiqi Zhang and Taoyu Yang and Xuheng Zhang and Wenhao Zhang and Heming Cui and Zhizheng Zhang and He Wang},
    year={2025},
    eprint={2505.03233},
    archivePrefix={arXiv},
    primaryClass={cs.RO},
    url={https://arxiv.org/abs/2505.03233}
}
```

- Control Suite:
```bibtex
@misc{luo2025serlsoftwaresuitesampleefficient,
      title={SERL: A Software Suite for Sample-Efficient Robotic Reinforcement Learning}, 
      author={Jianlan Luo and Zheyuan Hu and Charles Xu and You Liang Tan and Jacob Berg and Archit Sharma and Stefan Schaal and Chelsea Finn and Abhishek Gupta and Sergey Levine},
      year={2025},
      eprint={2401.16013},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2401.16013}, 
}
```

[![License](https://licensebuttons.net/l/by-nc/4.0/88x31.png)](LICENSE)