# Pose-estimation

POSE ESTIMATION FOR HAND RAILING COMPLAINCE

[written relative to the src folder in poseestimation.]

-- WORKON py virtual environment

-- "sort_sar.py" consists the code for hand railing complaince with the trackers implemented using SORT algorithm.
    - change the paths in args.py file
    - change dst variable to change threshold (check line 144).

    pops up the video with the detections and results (the code to render the video is commented).

-- "hover.py" consists the code to draw lines on the image and print the points of the lines.
   - change the path to the image for different usage.

   returns the points

-- "newhulpose1.py" consists the code for hand railing complaince with the trackers implemented using CSRT
algrithm
    - Requirements - same as that of sort_sar.py
