package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;

public class PoseEstimator {
  // Ship of Theseused from
  // https://github.com/wpilibsuite/allwpilib/blob/1db3936965bd8ed33224ad388cf9f16d12999a08/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java
  NavigableMap<Double, PoseRecord> pastSnapshots = new TreeMap<>();

  /**
   * Adds an odometry record at the current time
   *
   * @param odo The change in position since the last update
   */
  public void addOdometry(Twist2d odo) {
    addOdometry(odo, Timer.getFPGATimestamp());
  }

  public void setPose(Pose2d pose) {
    pastSnapshots.clear();
    pastSnapshots.put(Timer.getFPGATimestamp(), new PoseRecord(pose, new Twist2d()));
  }
  /**
   * Adds an odometry record at the given time
   *
   * @param odo The change in position since the last update
   * @param timestamp The time at which the twist was measured
   */
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

  /**
   * Adds a vision update
   *
   * @param estimate The pose estimate
   * @param visionMeasurementStdDevs The measurement error standard deviations [x, y, theta]
   *     [meters,meters,radians]
   * @param t The time at which the camera captured the pose estimate
   */
  public void addVision(Pose2d estimate, Vector<N3> visionMeasurementStdDevs, double t) {
    Pose2d poseAtTime =
        getPose(t).orElseThrow(() -> new IllegalArgumentException("Time given is outside bounds"));
    Twist2d proposedUpdate = poseAtTime.log(estimate);
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

    Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int i = 0; i < 3; ++i) {
      m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }

    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I
    for (int row = 0; row < 3; ++row) {
      if (m_q.get(row, 0) == 0.0) {
        m_visionK.set(row, row, 0.0);
      } else {
        m_visionK.set(
            row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
      }
    }
    var k_times_twist =
        m_visionK.times(
            VecBuilder.fill(proposedUpdate.dx, proposedUpdate.dy, proposedUpdate.dtheta));

    Twist2d resultantTwist =
        new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));
    pastSnapshots.put(
        t, new PoseRecord(poseAtTime.exp(resultantTwist), estimate, visionMeasurementStdDevs));
    var updatesAfter = pastSnapshots.tailMap(t, false).entrySet().toArray();
    while (pastSnapshots.lastKey() > t) {
      pastSnapshots.remove(pastSnapshots.lastKey());
    }
    for (Object update2 : updatesAfter) {
      Map.Entry<Double, PoseRecord> update = (Map.Entry<Double, PoseRecord>) update2;
      if (update.getValue().isOdometryRecord) {
        addOdometry(update.getValue().odometryData, update.getKey());
      } else {
        addVision(update.getValue().visionData, update.getValue().visionStdDevs, update.getKey());
      }
    }
  }

  /**
   * Gets the current pose
   *
   * @return The current pose
   */
  public Pose2d getPose() {
    return pastSnapshots.lastEntry().getValue().poseEstimate;
  }

  /**
   * Gets the pose at a given time
   *
   * @param t The time
   * @return If there has been no updates or the time is out of range, Optional.empty(). Otherwise,
   *     the pose at the time.
   */
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

  /** Constructs a new pose estimator */
  public PoseEstimator() {}

  static class PoseRecord {
    public Pose2d poseEstimate;
    public boolean isOdometryRecord = false;
    public Twist2d odometryData;
    public Pose2d visionData;
    public Vector<N3> visionStdDevs;

    public PoseRecord(Pose2d poseEstimate) {
      this.poseEstimate = poseEstimate;
    }

    public PoseRecord(Pose2d poseEstimate, Pose2d visionData, Vector<N3> visionStdDevs) {
      this.poseEstimate = poseEstimate;
      this.visionData = visionData;
      this.visionStdDevs = visionStdDevs;
    }

    public PoseRecord(Pose2d poseEstimate, Twist2d odometryData) {
      this.poseEstimate = poseEstimate;
      this.odometryData = odometryData;
      isOdometryRecord = true;
    }
  }
}
