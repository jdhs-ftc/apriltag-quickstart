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

import java.util.List;

/**
 * Experimental extension of MecanumDrive that uses AprilTags for relocalization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the BSD 3-Clause Clear License by Michael from 14343 and by Ryan Brott
 */
public class AprilTagDrive extends MecanumDrive {
    @Config
    static class Params {
        // distance FROM robot center TO camera (inches)
        // TODO: tune
        static Vector2d cameraOffset = new Vector2d(
                -6,
                4);

        /*
         * Q model covariance (trust in model), default 0.1 HIGHER IS LESS TRUST
         * R sensor covariance (trust in sensor), default 0.4
         */
        static double kalmanFilterQ = 0.4;
        static double kalmanFilterR = 0.1;
    }

    Vector2d cameraOffset;
    final AprilTagProcessor aprilTag;
    public List<AprilTagDetection> currentDetections;
    final KalmanFilter.Vector2dKalmanFilter posFilter;
    Pose2d localizerPose;
    Vector2d filteredVector;
    /**
     * Init with just one camera; use instead of MecanumDrive
     * @param hardwareMap the hardware map
     * @param pose the starting pose
     * @param aprilTag your camera's AprilTagProcessor
     */
    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTag) {
        super(hardwareMap, pose);
        this.aprilTag = aprilTag;
        this.posFilter = new KalmanFilter.Vector2dKalmanFilter(Params.kalmanFilterQ, Params.kalmanFilterR);
        this.cameraOffset = Params.cameraOffset;

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
            // not sure if this is logical at all??
            // UNTESTED, WE WERENT USING FILTEREDVECTOR :skull:
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
        currentDetections = aprilTag.getDetections();
        Vector2d averagePos = new Vector2d(0,0); // starting pose to add the rest to
        if (this.currentDetections.isEmpty()) return null; // if we don't see any tags, give up (USES NEED TO HANDLE NULL)

        // Step through the list of detections and calculate the robot position from each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                // we're going to get the average here by adding them all up and dividing by the number of detections
                // we do this because the backdrop has 3 tags, so we get 3 positions
                // hopefully by averaging them we can get a more accurate position
                averagePos = averagePos.plus(getFCPosition(detection, localizerPose.heading.log(), Params.cameraOffset));

            }
        }   // end for() loop

        // divide by the number of detections to get the true average, as explained earlier
        return averagePos.div(currentDetections.size());
    }

    /**
     * getFCPosition credit Michael from team 14343 (@overkil on Discord)
     * @param botheading In Radians.
     * @return FC Pose of bot.
     */
    public Vector2d getFCPosition(AprilTagDetection detection, double botheading, Vector2d cameraOffset) {
        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC
        double x = detection.ftcPose.x-cameraOffset.x;
        double y = detection.ftcPose.y-cameraOffset.y;

        // invert heading to correct properly
        botheading = -botheading;

        // rotate RC coordinates to be field-centric
        double x2 = x*Math.cos(botheading)+y*Math.sin(botheading);
        double y2 = x*-Math.sin(botheading)+y*Math.cos(botheading);
        // add FC coordinates to apriltag position
        // tags is just the CS apriltag library
        VectorF tagpose = getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;


        // todo: this will need to be changed for next season (use tag heading to automate??)
        if (!detection.metadata.name.contains("Audience")) { // is it a backdrop tag?
            return new Vector2d(
                    tagpose.get(0) + y2,
                    tagpose.get(1) - x2);

        } else {
            return new Vector2d(
                    tagpose.get(0) - y2,
                    tagpose.get(1) + x2);

        }
    }

    // this custom position library credit Michael from team 14343 (@overkil on Discord)
    // TODO: will need to be changed for 24-25 season
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
