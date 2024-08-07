package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Map;
import java.util.TreeMap;

// credit Tarun from 12791 (@_skull.emoji_) for this class
public class PosePatcher {
    public final TreeMap<Long, Pose2d> map = new TreeMap<>();

    public final long timeout;

    public PosePatcher(int timeoutNS) {
        this.timeout = timeoutNS;
    }

    public void add(Pose2d pose) {
        map.put(System.nanoTime(), pose);
    }

    public void removeOld() {
        long time = System.nanoTime() - timeout;
        while (!map.isEmpty()) {
            Long key = map.floorKey(time);
            if (key != null)
                map.remove(key);
            else break;
        }
    }

    @Nullable
    public Pose2d patch(Pose2d newPose, long timestampNS) {
        Map.Entry<Long, Pose2d> val = map.floorEntry(timestampNS);
        if (val == null) return null;

        return this.patch(newPose, val);
    }

    @Nullable
    public Pose2d patch(Vector2d newVec, long timestampNS) {
        Map.Entry<Long, Pose2d> val = map.floorEntry(timestampNS);
        if (val == null) return null;

        return this.patch(new Pose2d(newVec, val.getValue().heading), val);
    }

    @Nullable
    public Pose2d patch(double newHeading, long timestampNS) {
        Map.Entry<Long, Pose2d> val = map.floorEntry(timestampNS);
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