# img_imu_record

The main purpose of this package is to record incoming imu messages and image messages into a "flat file" format that can be used for offline processing. This is a ros package that will listen to `/camera/image_raw` and `/imu_vn_100/imu`. These two topics will be subscribed too, and when an event happens the data will be recorded to the output folder on disk. It will generate a image folder and a imu folder.

### Dependencies

* This uses ros's opencv 3.
* This should be installed by default but if it is not make sure that you have run `sudo apt-get update` and `sudo apt-get upgrade` on your system.
* The package can manually be gotten by running `sudo apt-get install ros-indigo-opencv3`.
* Ensure that you have boost installed. This should come native on ubuntu
* Tested with ros indigo
* Build with catkin or with [catkin tools](https://catkin-tools.readthedocs.org/en/latest/)