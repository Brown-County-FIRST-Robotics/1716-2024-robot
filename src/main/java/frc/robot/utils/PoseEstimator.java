package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import java.util.*;

public class PoseEstimator {
  NavigableMap<Double, PoseRecord> pastSnapshots = new TreeMap<>();

  public void addOdometry(Twist2d odo, double timestamp) {
    ArrayList<Object> l =
        new ArrayList<>(List.of(pastSnapshots.headMap(timestamp).keySet().toArray()));
    Collections.reverse(l);
    double lastOdoTime = 0.0;
    for (var i : l) {
      var snapshot = pastSnapshots.get(i);
      if (snapshot.isOdometryRecord) {
        lastOdoTime = (Double) i;
        break;
      }
    }
    var lastRecordTime = (Double) l.get(0);
    double timeScaleFactor = (timestamp - lastRecordTime) / (timestamp - lastOdoTime);
    Twist2d timeScaledTwist =
        new Twist2d(
            timeScaleFactor * odo.dx, timeScaleFactor * odo.dy, timeScaleFactor * odo.dtheta);
    Pose2d timeGuessedPose = pastSnapshots.get(lastRecordTime).poseEstimate.exp(timeScaledTwist);
    pastSnapshots.put(timestamp, new PoseRecord(timeGuessedPose, odo));
  }

  public void addVision(Pose2d estimate, double t) {
    Pose2d poseAtTime =
        getPose(t).orElseThrow(() -> new IllegalArgumentException("Time given is outside bounds"));
    Twist2d proposedUpdate = poseAtTime.log(estimate);

    // TEMP: Add Kalman matrix code
    Twist2d resultantTwist = new Twist2d();
    pastSnapshots.put(t, new PoseRecord(poseAtTime.exp(resultantTwist), estimate));
    var updatesAfter = pastSnapshots.tailMap(t, false).entrySet().toArray();
    while (pastSnapshots.lastKey() > t) {
      pastSnapshots.remove(pastSnapshots.lastKey());
    }
    for (Object update2 : updatesAfter) {
      Map.Entry<Double, PoseRecord> update = (Map.Entry<Double, PoseRecord>) update2;
      if (update.getValue().isOdometryRecord) {
        addOdometry(update.getValue().odometryData, update.getKey());
      } else {
        addVision(update.getValue().visionData, update.getKey());
      }
    }
  }

  public Pose2d getPose() {
    return pastSnapshots.lastEntry().getValue().poseEstimate;
  }

  public Optional<Pose2d> getPose(double t) {
    if (pastSnapshots.isEmpty()) {
      return Optional.empty();
    }
    if (pastSnapshots.get(t) != null) {
      return Optional.of(pastSnapshots.get(t).poseEstimate);
    }
    var ceil = pastSnapshots.ceilingEntry(t);
    var floor = pastSnapshots.floorEntry(t);
    if (ceil == null && floor == null) {
      return Optional.empty();
    } else if (ceil == null) {
      return Optional.of(floor.getValue().poseEstimate);
    } else if (floor == null) {
      return Optional.of(ceil.getValue().poseEstimate);
    } else {
      return Optional.of(
          floor
              .getValue()
              .poseEstimate
              .interpolate(
                  ceil.getValue().poseEstimate,
                  (t - floor.getKey()) / (ceil.getKey() - floor.getKey())));
    }
  }

  static class PoseRecord {
    public Pose2d poseEstimate;
    public boolean isOdometryRecord = false;
    public Twist2d odometryData;
    public Pose2d visionData;

    public PoseRecord(Pose2d poseEstimate) {
      this.poseEstimate = poseEstimate;
    }

    public PoseRecord(Pose2d poseEstimate, Pose2d visionData) {
      this.poseEstimate = poseEstimate;
      this.visionData = visionData;
    }

    public PoseRecord(Pose2d poseEstimate, Twist2d odometryData) {
      this.poseEstimate = poseEstimate;
      this.odometryData = odometryData;
      isOdometryRecord = true;
    }
  }
}
