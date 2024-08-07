package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.sun.source.tree.LabeledStatementTree;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.*;

/**
 * Experimental extension of MecanumDrive that uses AprilTags for relocalization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code released under the BSD 3-Clause Clear License by Michael from team 14343,
 * Tarun from team 12791, and by Ryan Brott from team 8367
 */
public class AprilTagDrive extends MecanumDrive { // TODO: if not using MecanumDrive, change to your drive class (e.g. TankDrive, SparkFunOTOSDrive)
    @Config
    static class Params {
        // distance FROM robot center TO camera (inches)
        // TODO: tune
        static Vector2d cameraOffset = new Vector2d(
                -6,
                4);

    }

    Vector2d cameraOffset;
    final AprilTagProcessor aprilTag;
    Pose2d localizerPose;
    long tagDetectTime;
    /**
     * Init with just one camera; use instead of MecanumDrive
     * @param hardwareMap the hardware map
     * @param pose the starting pose
     * @param aprilTag your camera's AprilTagProcessor
     */
    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose, AprilTagProcessor aprilTag) {
        super(hardwareMap, pose);
        this.aprilTag = aprilTag;
        this.cameraOffset = Params.cameraOffset;

    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        // RR standard: get the latest pose from the upstream updatePoseEstimate
        // that will change the pose variable to the pose based on odo or drive encoders (or OTOS)
        PoseVelocity2d posVel = super.updatePoseEstimate();
        localizerPose = pose;
        // Get the absolute position from the camera
        Vector2d aprilVector = getVectorBasedOnTags();


        // it's possible we can't see any tags, so we need to check for a vector of 0
        if (aprilVector != null) {
            // if we can see tags, we use the apriltag position
            // however apriltags don't have accurate headings so we use the localizer heading
            // localizer heading, for us and in TwoDeadWheelLocalizer, is IMU and absolute-ish
            // TODO: apriltags unreliable at higher speeds? speed limit? global shutter cam? https://discord.com/channels/225450307654647808/225451520911605765/1164034719369941023

            // there could potentially be a delay between the time the atags were detected and this update function
            // and during that time, the robot could have moved considerably
            // so we use the current speed to calculate how far we've probably traveled from the atag pose

            aprilVector = aprilVector.plus( // start with the atag vector
                    posVel.linearVel // get the linear velocity from upstream localization (this is in inches/sec)
                            .times((System.nanoTime() - tagDetectTime) // multiply it by the time since the tag was detected
                                    * 1e-9)); // and convert it to seconds

            // then we add the apriltag position to the localizer heading as a pose
            pose = new Pose2d(aprilVector, localizerPose.heading); // TODO: aprilVector should be filteredVector to use kalman filter (kalman filter is untested)
        }

        FlightRecorder.write("APRILTAG_POSE", new PoseMessage(pose));

        return posVel; // trust the existing localizer for speeds because I don't know how to do it with apriltags
    }
    public Vector2d getVectorBasedOnTags() {
        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            return null;
        } else {
            tagDetectTime = aprilTag.getDetections().get(0).frameAcquisitionNanoTime;
            return aprilTag.getDetections().stream() // get the tag detections as a Java stream
                    // convert them to Vector2d positions using getFCPosition
                    .map(detection -> getFCPosition(detection, localizerPose.heading.log(), Params.cameraOffset))
                    // add them together
                    .reduce(new Vector2d(0, 0), Vector2d::plus)
                    // divide by the number of tags to get the average position
                    .div(aprilTag.getDetections().size());
        }
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
    // credit Tarun from 12791 (@_skull.emoji_) for this class
    public class PosePatcher {
        public final TreeMap<Long, Pose2d> map = new TreeMap<>();

        public final int timeout;

        public PosePatcher(int timeoutMS) {
            this.timeout = timeoutMS;
        }

        public void add(Pose2d pose) {
            map.put(System.currentTimeMillis(), pose);
        }

        public void removeOld() {
            long time = System.currentTimeMillis() - timeout;
            while (!map.isEmpty()) {
                Long key = map.floorKey(time);
                if (key != null)
                    map.remove(key);
                else break;
            }
        }

        @Nullable
        public Pose2d patch(Pose2d newPose, long timestampMS) {
            Map.Entry<Long, Pose2d> val = map.floorEntry(timestampMS);
            if (val == null) return null;

            return this.patch(newPose, val);
        }

        @Nullable
        public Pose2d patch(Vector2d newVec, long timestampMS) {
            Map.Entry<Long, Pose2d> val = map.floorEntry(timestampMS);
            if (val == null) return null;

            return this.patch(new Pose2d(newVec, val.getValue().heading), val);
        }

        @Nullable
        public Pose2d patch(double newHeading, long timestampMS) {
            Map.Entry<Long, Pose2d> val = map.floorEntry(timestampMS);
            if (val == null) return null;

            return this.patch(new Pose2d(val.getValue().position, newHeading), val);
        }

        private Pose2d patch(Pose2d newPose, Map.Entry<Long, Pose2d> val) {
            // Find pose difference from reference
            Twist2d diff = newPose.minus(val.getValue());

            // Update reference pose
            val.setValue(newPose);

            Map.Entry<Long, Pose2d> current = val;
            while (true) {
                // Get the next pose in list, otherwise return the most recent one (which should be the current pose)
                Map.Entry<Long, Pose2d> next = map.higherEntry(current.getKey());
                if (next == null) return current.getValue();

                // Add the initial pose difference to the pose
                Pose2d pose = next.getValue().plus(diff);

                // Rotate the pose around the reference pose by the angle difference
                next.setValue(new Pose2d(
                        newPose.position.x + (pose.position.x - newPose.position.x) * Math.cos(diff.angle) - (pose.position.y - newPose.position.y) * Math.sin(diff.angle),
                        newPose.position.y + (pose.position.x - newPose.position.x) * Math.sin(diff.angle) + (pose.position.y - newPose.position.y) * Math.cos(diff.angle),
                        pose.heading.toDouble()
                ));

                current = next;
            }
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
