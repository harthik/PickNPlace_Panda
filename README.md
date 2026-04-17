# PickNPlace_Panda
Pick and place pipeline using panda robot arm

# Video
[![PickNPlace_Panda.mp4](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/kwYf8W0y8AQ)


# Vision Processing
The position of the blocks is caclulated in the camera frame
Segmentation mask
        |
Bounding Rectangle
        |
Center of Bounding Rectangle for position
        |
Angle calculated using long edge and unit 
    vector in vertical direction
        |
Position and rotation transformed to world frame
        |
Position used to solve IK and move arm
        |
Pick block and place ant set location

