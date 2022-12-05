# 2 Thread Model



## 2.1 Design Consideration

The driver is designed to achieve these objectives.
+ Parallel the construction and the process of point cloud.
+ Avoid to copy point cloud.
+ Avoid to malloc/free point cloud repeatly.

Below is the supposed interaction between rs_driver and user's code. 

![](./img/02_01_components_and_threads.png)

rs_driver runs in its thread `construct_thread`. It
+ Gets free point cloud from user. User fetches it from a free cloud queue `free_point_cloud_queue`. If the queue is empty, then create a new one.
+ Parses packets and constructs point cloud.
+ Returns stuffed point cloud to user.
+ User's code is supposed to shift it to the queue `stuffed_point_cloud_queue`.

User's code runs in its thread `process_thread`. It
+ Fetches stuffed point cloud from the queue `stuffed_point_cloud_queue`
+ Process the point cloud
+ Return the point cloud back to the queue `free_point_cloud_queue`. rs_driver will use it again.

