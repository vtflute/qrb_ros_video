# QRB ROS VIDEO
QRB ROS Video is a ROS package which provides video hardware acceleration capabilities on Qualcomm platform.

## Overview
QRB ROS Video utilizes the advanced Qualcomm VPU hardware for video encoding and decoding to meet the requirements of Robotics applications. QRB ROS Video is based on [qrb_ros_transport](https://github.com/quic-qrb-ros/qrb_ros_transport) which help us to implement zero memory copy locally and accept buffers from [qrb_ros_camera](https://github.com/quic-qrb-ros/qrb_ros_camera) to fulfill the recording purpose.

Right now, we support H264 codec. HEVC support will be added in future.

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### System Requirement

* ROS 2 Humble and later, for type adaption support.


### Running

This package supports running it from ROS launch file.

1. Source this file to set up the environment on your device:

   ```bash
   ssh root@[ip-addr]
   (ssh) export HOME=/opt
   (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
   (ssh) export ROS_DOMAIN_ID=xx
   (ssh) source /usr/bin/ros_setup.bash
   ```
2. launch commands

    - Encoding:

    ```bash
    ros launch qrb_ros_video encoder_launch.py
    ```
    
	* Decoding
	
	```bash
	ros launch qrb_ros_video decoder_launch.py
	```

## Building

Currently, we only support build with QCLINUX SDK.

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part
2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`
3. Clone this repository under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

    ```bash
    git clone https://github.com/quic-qrb-ros/qrb_ros_imu.git
    git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
    git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
    git clone https://github.com/quic-qrb-ros/qrb_ros_video.git
    ```

4. Build this project

   * For ROS2 Humble

   ```bash
   export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
   export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages
   
   colcon build --merge-install --cmake-args \
     -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
     -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
     -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
     -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
     -DBUILD_TESTING=OFF
   ```

   * For ROS2 Jazzy

   ```bash
   export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
   export PYTHONPATH=${PYTHONPATH}:${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages

   colcon build --merge-install --cmake-args \
      -DPython3_ROOT_DIR=${OECORE_NATIVE_SYSROOT}/usr \
      -DPython3_NumPy_INCLUDE_DIR=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/numpy/core/include \
      -DSYSROOT_LIBDIR=${OECORE_TARGET_SYSROOT}/usr/lib \
      -DSYSROOT_INCDIR=${OECORE_TARGET_SYSROOT}/usr/include \
      -DPYTHON_SOABI=cpython-312-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
      -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
      -DBUILD_TESTING=OFF
   ```

5. Push to the device & Install

   ```bash
   cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
   tar czvf qrb_ros_video.tar.gz lib share
   scp qrb_ros_video.tar.gz root@[ip-addr]:/opt/
   ssh root@[ip-addr]
   (ssh) tar -zxf /opt/qrb_ros_video.tar.gz -C /opt/qcom/qirp-sdk/usr/
   ```


## Deployment

Add additional notes about how to deploy this on a live system


## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)



## Documentation
Please visit [QRB ROS Documentation](https://quic-qrb-ros.github.io/) for more details.


## Authors

* **Jean Xiao** - *Initial work* - [jian xiao](https://quic_jianxiao.quicinc.com)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

