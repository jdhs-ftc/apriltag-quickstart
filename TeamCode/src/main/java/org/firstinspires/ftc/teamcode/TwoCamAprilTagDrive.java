package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Experimental extension of MecanumDrive that uses AprilTags for relocalization.
 *
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the BSD 3-Clause Clear License by Michael from 14343 and by Ryan Brott
 */
public class TwoCamAprilTagDrive extends MecanumDrive {
    @Config
    static class Params {
        // distance FROM robot center TO camera (inches)
        // TODO: tune
        static Vector2d camera1Offset = new Vector2d(
                -6,
                4);
        // if you don't have a second camera this doesn't matter
        static Vector2d camera2Offset = new Vector2d(
                0,
                -5);//6);
        static double cameraYawOffset = Math.toRadians(180); // TODO: tune
        /*
         * Q model covariance (trust in model), default 0.1
         * R sensor covariance (trust in sensor), default 0.4
         */
        static double kalmanFilterQ = 0.1;
        static double kalmanFilterR = 0.4;
    }

    Vector2d cameraOffset;
    final AprilTagProcessor aprilTagBack;
    AprilTagProcessor aprilTagFront = null;
    public List<AprilTagDetection> totalDetections;
    public AprilTagDetection lastDetection;
    final KalmanFilter.Vector2dKalmanFilter posFilter;
    Pose2d aprilPose;
    Pose2d localizerPose;
    Vector2d filteredVector;
    boolean frontCamActive = true;
    boolean backCamActive = true;
    /**
     * Init with just one camera; use instead of MecanumDrive
     * @param hardwareMap the hardware map
     * @param pose the starting pose
     * @param aprilTagBack your camera's AprilTagProcessor
     */
    public TwoCamAprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTagBack) {
        super(hardwareMap, pose);
        this.aprilTagBack = aprilTagBack;
        this.posFilter = new KalmanFilter.Vector2dKalmanFilter(Params.kalmanFilterQ, Params.kalmanFilterR);
        this.cameraOffset = Params.camera1Offset;

    }
    /**
     * Init with two cameras; use instead of MecanumDrive
     * @param hardwareMap the hardware map
     * @param pose the starting pose
     * @param aprilTagBack back camera's apriltag processor
     * @param aprilTagFront your second camera's AprilTagProcessor
     */
    public TwoCamAprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTagBack, AprilTagProcessor aprilTagFront) {
        super(hardwareMap, pose);
        this.aprilTagBack = aprilTagBack;
        this.aprilTagFront = aprilTagFront;
        this.posFilter = new KalmanFilter.Vector2dKalmanFilter(Params.kalmanFilterQ, Params.kalmanFilterR);
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        // RR standard: get the movement between loops from the localizer
        // RR assumes there's no way to get absolute position and gets relative between loops
        // note that this adds on top of existing pose, even if that pose was just corrected by apriltag
        Twist2dDual<Time> twist = localizer.update();
        localizerPose = pose.plus(twist.value());
        // Get the absolute position from the camera
        Vector2d aprilVector = getVectorBasedOnTags();


        // it's possible we can't see any tags, so we need to check for null
        if (aprilVector != null) {
            // if we can see tags, we use the apriltag position
            // however apriltags don't have accurate headings so we use the localizer heading
            // localizer heading, for us and in TwoDeadWheelLocalizer, is IMU and absolute-ish
            // TODO: apriltags unreliable at higher speeds? speed limit? global shutter cam? https://discord.com/channels/225450307654647808/225451520911605765/1164034719369941023

            // we input the change from odometry with the april absolute pose into the kalman filter
            filteredVector = posFilter.update(twist.value(), aprilVector);
            // then we add the kalman filtered position to the localizer heading as a pose
            pose = new Pose2d(aprilVector, localizerPose.heading); // TODO: aprilVector should be filteredVector to use kalman filter (kalman filter is untested)
        } else {
            // if we can't see tags, we use the localizer position to update the kalman fiter
            // not sure if this is logical at all?? seems to work
            filteredVector = posFilter.update(twist.value(), localizerPose.position);

            // then just use the existing pose
            pose = localizerPose;
        }



        // rr standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return twist.velocity().value(); // trust the existing localizer for speeds; because I don't know how to do it with apriltags
    }
    public Vector2d getVectorBasedOnTags() {
        List<AprilTagDetection> currentDetections = new ArrayList<>();
        if (backCamActive) {
            currentDetections = aprilTagBack.getDetections();
        }
        List<AprilTagDetection> cam2Detections = new ArrayList<>();
        if (aprilTagFront != null && frontCamActive) {
            cam2Detections = aprilTagFront.getDetections();
        }
        totalDetections = new ArrayList<>();
        totalDetections.addAll(currentDetections);
        totalDetections.addAll(cam2Detections);
        int totalDetections = 0;
        Vector2d averagePos = new Vector2d(0,0); // starting pose to add the rest to
        if (this.totalDetections.isEmpty()) return null; // if we don't see any tags, give up (USES NEED TO HANDLE NULL)
        Vector2d RobotPos;

        // Step through the list of detections and calculate the robot position from each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                RobotPos = getFCPosition(detection, localizerPose.heading.log(),false);

                // we're going to get the average here by adding them all up and dividingA the number of detections
                // we do this because the backdrop has 3 tags, so we get 3 positions
                // hopefully by averaging them we can get a more accurate position
                lastDetection = detection;
                averagePos = averagePos.plus(RobotPos);
                totalDetections++;

            }
        }   // end for() loop

        if (!cam2Detections.isEmpty()) {
            // Step through the list of detections and calculate the robot position from each one.
            for (AprilTagDetection detection : cam2Detections) {
                if (detection.metadata != null) {
                    RobotPos = getFCPosition(detection, localizerPose.heading.log(), true);

                    // we're going to get the average here by adding them all up and dividing by the number of detections
                    // we do this because the backdrop has 3 tags, so we get 3 positions
                    // hopefully by averaging them we can get a more accurate position
                    lastDetection = detection;
                    averagePos = averagePos.plus(RobotPos);
                    totalDetections++;

                }
            }   // end for() loop
        }
        // divide by the number of detections to get the true average, as explained earlier
        return averagePos.div(totalDetections);
    }

    /**
     * getFCPosition credit Michael from team 14343 (@overkil on Discord)
     * @param botheading In Radians.
     * @return FC Pose of bot.
     */
    public Vector2d getFCPosition(AprilTagDetection detection, double botheading, boolean usingCam2) {
        Vector2d cameraOffset;
        if (usingCam2) {cameraOffset = Params.camera2Offset;} else {cameraOffset = Params.camera1Offset;}
        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC
        double x = detection.ftcPose.x-cameraOffset.x;
        double y = detection.ftcPose.y-cameraOffset.y;

        // invert heading to correct properly
        botheading = -botheading;


        // rotate RC coordinates to be field-centric
        double x2 = x*Math.cos(botheading)+y*Math.sin(botheading);
        double y2 = x*-Math.sin(botheading)+y*Math.cos(botheading);
        double absX;
        double absY;
        // add FC coordinates to apriltag position
        // tags is just the CS apriltag library
        VectorF tagpose = getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;
        if (detection.metadata.id <= 6) {
            absX = tagpose.get(0) + y2;
            absY = tagpose.get(1) - x2;

        } else {
            absX = tagpose.get(0) - y2;
            absY = tagpose.get(1) + x2; // prev -

        }
        return new Vector2d(absX, absY);
    }

    // this custom position library credit Michael from team 14343 (@overkil on Discord)
    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }
}
