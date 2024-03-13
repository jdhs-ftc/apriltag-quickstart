# Using AprilTags with Roadrunner 1.0

FIRST added fancy AprilTags onto the field walls and backdrop recently to replace the old Vuforia system. It is extremely simple to get their relative position to the robot's camera, and with a bit of math and calibration, it is possible to use this data to improve your robot's localization.


## Camera Calibration

To get accurate position data from the AprilTags, it is critical to first calibrate your camera's lens intrinsics. FIRST has a good overview [here](https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/camera_calibration/camera-calibration.html). Calibration is a relatively simple process that should take under an hour.

## Accessing the Camera & Detecting Tags

Accessing the camera is quite simple with the use of the new VisionPortal.
Here's a basic example:

    AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
        //add .setLensIntrinsics here with your calibration data
        .build();
    VisionPortal vPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .addProcessor(aprilTag)
        // use another .addProcessor here to use EOCV, tensorflow, etc.
        .build();
