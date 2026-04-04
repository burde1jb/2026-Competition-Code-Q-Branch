package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConstants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.AlphaBots.NT;
import frc.robot.LimelightHelpers.*;

public class VisionSubsystem extends SubsystemBase {
  private final String classname = this.getClass().getSimpleName(); //this will autopopulate to the classname which is "VisionSubsystem"
  private final boolean kUseLimelight = true; //false if not using Limelight, true if using Limelight
  private RawFiducial[] fiducials;
  private final CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;//assumes the drivetrain is made before the vision subsystem, which is true in our current code

  // Uncomment the line below to enable dashboard-driven vision tuning.
  // private final VisionTuning tuning = new VisionTuning("Vision Tuning", this);

  private final StructPublisher<Pose2d> LLPose = NT.getStructEntry_Pose2D(classname, "LLPose", new Pose2d());
  private final StructPublisher<Pose2d> drivePose = NT.getStructEntry_Pose2D(classname, "DrivetrainPose", new Pose2d());

  //these will tell you if at a given time we accepted the MegaTag1 reading from Limelight A and B, which can be useful for debugging and tuning.
  private final BooleanEntry MT1AUsed = NT.getBooleanEntry(classname, "MT1AUsed", false);
  private final BooleanEntry MT1BUsed = NT.getBooleanEntry(classname, "MT1BUsed", false);

  //these will tell you if at a given time we accepted the MegaTag2 reading from Limelight A and B, which can be useful for debugging and tuning.
  private final BooleanEntry MT2AUsed = NT.getBooleanEntry(classname, "MT2AUsed", false);
  private final BooleanEntry MT2BUsed = NT.getBooleanEntry(classname, "MT2BUsed", false);

  //these are the limelight names set by the web interface, which are also the network table subtable names for the limelight data. If you change the limelight names in the web interface, make sure to change them here too.
  private static String LimelightA = "limelight-four";
  private static String LimelightB = "limelight-three";

  public VisionSubsystem() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {
    LimelightHelpers.setCameraPose_RobotSpace(
        LimelightA,
        -0.32, 
        -0.29,
        0.23,
        0,
        20,
        180);
    LimelightHelpers.setCameraPose_RobotSpace(
        LimelightB,
        -0.28, 
        0.33,
        0.22,
        0,
        15,
        -90);    
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", new int[] {5,7,8,12,15,21,23,24,28,31});
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-three", new int[] {5,7,8,12,15,21,23,24,28,31});
  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials(LimelightA);
    UpdateDrivetrainFromLimelight();

  }
  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;

    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }

    return closest;
  }

  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {
  StringBuilder availableIds = new StringBuilder();

  for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
          availableIds.append(", ");
      } //Error reporting
      availableIds.append(fiducial.id);
      
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
  }

  public double getTX1(){
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME1);
  }
  public double getTY1(){
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME1);
  }
  public double getTA1(){
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME1);
  }
  public boolean getTV1(){
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME1);
  }
  public double getTX2(){
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME2);
  }
  public double getTY2(){
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME2);
  }
  public double getTA2(){
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME2);
  }
  public boolean getTV2(){
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME2);
  }

  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }

  /*
   * first we process the MegaTag1 readings from both limelights, 
   * which give us a pose estimate that is independent of our current robot heading (MT1 does its own solve). 
   * We apply tunable rejection criteria to decide whether to accept each MT1 reading, 
   * and we scale the MT1 measurement std devs based on the average tag distance (e.g. if tags are far away, we trust MT1 less and increase the std devs so the pose estimator relies more on the gyro and less on MT1).
   * 
   * after megatag1 processing, we set the robot orientation in the limelight network tables 
   * to the possibly updated robot heading from the drivetrain pose estimator,
   *  if the MT1 were rejected we STILL need to update our limelights orientation because our drivetrain has it own odometry! 
   * This allows MegaTag2 to do a more accurate multi-tag solve that relies on the gyro for heading and vision for x/y position. 
   * Then we process the MegaTag2 readings from both limelights, applying tunable rejection criteria and scaling the MT2 std devs based on average tag distance.
   */
  /* STANDARD DEVIATIONS EXPLAINED:
   * The pose estimator uses a Kalman filter to fuse odometry and vision measurements.
   * setVisionMeasurementStdDevs(Matrix<N3, N1>) -> This method sets how much the pose estimator trusts vision measurements.
   *  The matrix is a 3×1 column vector (3 rows, 1 column) representing standard deviations for each component of the vision pose:
   * How to think about it
   * Smaller values = more trust in vision (the estimator weights vision more heavily)
   * Larger values = less trust in vision (the estimator favors odometry)
   * A standard deviation of 0.1 meters means "I believe vision is accurate to about ±10 cm." A value of 1.0 meters means "vision could be off by ±1 meter, don't trust it much."
   */
  // ---- Tunable rejection / weighting thresholds ----
  /** Minimum number of tags a MegaTag1 reading must see to be accepted */
  int kMT1_MinTagCount = 2;
  /** Max chassis speed (m/s) before we reject MegaTag1 readings */
  double kMT1_MaxSpeedMps = 3.0;
  /** Max rotation rate (rotations/s) before we reject MegaTag1 readings */
  double kMT1_MaxOmegaRps = 1.5;
  /** Distance (m) beyond which MegaTag1 std devs are increased */
  double kMT1_FarDistanceMeters = 3.0;
  /** Base std devs for MegaTag1 when tags are close [x, y] in meters */
  double kMT1_CloseStdDevXY = 0.5;
  /** Std devs for MegaTag1 when tags are far [x, y] in meters */
  double kMT1_FarStdDevXY = 2.0;
  /** Yaw std dev (radians) for MegaTag1 when tags are close — low = trust vision yaw */
  double kMT1_CloseStdDevYaw = 0.5;
  /** Yaw std dev (radians) for MegaTag1 when tags are far — high = trust gyro only */
  double kMT1_FarStdDevYaw = 9999999;
  /** Single-tag disabled scaling factor for MT1 std devs */
  double kMT1_SingleTagDisabledStdDevScale = 2.0;

  /** Omega threshold (rotations/s) where yaw std dev soft gating begins (270 deg/s) */
  double kVisionYawSoftGateStartOmegaRps = 0.75;
  /** Yaw scale factor per extra rotation/s above the soft gate threshold */
  double kVisionYawSoftGateFactor = 2.0;
  /** Maximum yaw scale multiplier from rotation speed gating */
  double kVisionYawSoftGateMaxScale = 6.0;

  /** Max rotation rate (rotations/s) before we reject MegaTag2 readings */
  double kMT2_MaxOmegaRps = 2.0;
  /** Multiplier applied to avgTagDist for MegaTag2 std dev scaling (tune this!) */
  double kMT2_DistanceStdDevMultiplier = 0.7;
  /** Minimum std dev floor so we never trust MegaTag2 infinitely */
  double kMT2_MinStdDev = 0.3;
  /** Base yaw std dev for MegaTag2 when not spinning quickly */
  double kMT2_BaseStdDevYaw = 0.75;
  /** Minimum yaw std dev for MegaTag2 so vision yaw is never overtrusted */
  double kMT2_MinStdDevYaw = 0.3;

  // Vision tuning is managed in the separate VisionTuning subsystem.

  private void UpdateDrivetrainFromLimelight() {
    if (!kUseLimelight) return;
    var driveState = drivetrain.getState();
    double omegaRps = Math.abs(Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond));
    double chassisSpeedMps = Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond);

    // ===== PHASE 1: MegaTag1 from both cameras =====
    // Grab MegaTag1 BEFORE setting robot orientation (MT1 does its own multi-tag solve)
    var mt1A = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightA);
    var mt1B = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightB);

    boolean usedMT1A = processMegaTag1(mt1A, "MT1-A", omegaRps, chassisSpeedMps);
    boolean usedMT1B = processMegaTag1(mt1B, "MT1-B", omegaRps, chassisSpeedMps);

    MT1AUsed.set(usedMT1A);
    MT1BUsed.set(usedMT1B);

    // ===== PHASE 2: Set robot orientation for MegaTag2 using our (now possibly updated) pose =====
    double updatedHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(LimelightA, updatedHeadingDeg, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightB, updatedHeadingDeg, 0, 0, 0, 0, 0);

    // ===== PHASE 3: MegaTag2 from both cameras =====
    var mt2A = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightA);
    var mt2B = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightB);

    boolean usedMT2A = processMegaTag2(mt2A, "MT2-A", omegaRps);
    boolean usedMT2B = processMegaTag2(mt2B, "MT2-B", omegaRps);

    MT2AUsed.set(usedMT2A);
    MT2BUsed.set(usedMT2B);

    // Publish best available pose for dashboard
    if (mt2A != null && mt2A.tagCount > 0) {
      LLPose.set(mt2A.pose);
    } else if (mt1A != null && mt1A.tagCount > 0) {
      LLPose.set(mt1A.pose);
    }
    //show the latest drivetrain pose from the pose estimator, which may have been updated by the vision measurements we added above. This is useful for seeing how the pose estimator is doing and for debugging/tuning the vision measurement std devs.
    drivePose.set(drivetrain.getState().Pose);
  }

  /**
   * Processes a MegaTag1 reading. Rejects if tag count < 2, speed too high, or omega too high.
   * Scales std devs up when average tag distance is large.
   * @return true if the measurement was accepted and added
   */
  private boolean processMegaTag1(LimelightHelpers.PoseEstimate mt1, String label,
                                   double omegaRps, double chassisSpeedMps) {
  boolean singleTagDisabled = DriverStation.isDisabled() && mt1 != null && mt1.tagCount == 1;
    if (mt1 == null || (mt1.tagCount < kMT1_MinTagCount && !singleTagDisabled)) return false;
    if (chassisSpeedMps > kMT1_MaxSpeedMps) return false;
    if (omegaRps > kMT1_MaxOmegaRps) return false;

    // Scale std devs based on distance
    boolean isClose = mt1.avgTagDist <= kMT1_FarDistanceMeters;
    double stdXY = isClose ? kMT1_CloseStdDevXY : kMT1_FarStdDevXY;
    double stdYaw = isClose ? kMT1_CloseStdDevYaw : kMT1_FarStdDevYaw;
    if (omegaRps > kVisionYawSoftGateStartOmegaRps && stdYaw < kMT1_FarStdDevYaw) {
      double yawScale = 1.0
          + Math.min(kVisionYawSoftGateMaxScale - 1.0,
              (omegaRps - kVisionYawSoftGateStartOmegaRps) * kVisionYawSoftGateFactor);
      stdYaw = Math.min(kMT1_FarStdDevYaw, stdYaw * yawScale);
    }

    //drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(stdXY, stdXY, stdYaw));
    if (singleTagDisabled) {
      stdXY *= kMT1_SingleTagDisabledStdDevScale;
      stdYaw = Math.min(kMT1_FarStdDevYaw, stdYaw * kMT1_SingleTagDisabledStdDevScale);
    }
    drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, VecBuilder.fill(stdXY, stdXY, stdYaw));
                                    
    SmartDashboard.putNumber("Vision/" + label + " Tags", mt1.tagCount);
    SmartDashboard.putNumber("Vision/" + label + " AvgDist", mt1.avgTagDist);
    SmartDashboard.putNumber("Vision/" + label + " StdDev", stdXY);
    
    return true;
  }

  /**
   * Processes a MegaTag2 reading. Rejects if no tags or omega too high.
   * Weights std devs by average tag distance * tunable multiplier.
   * @return true if the measurement was accepted and added
   */
  private boolean processMegaTag2(LimelightHelpers.PoseEstimate mt2, String label, double omegaRps) {
    if (mt2 == null || mt2.tagCount == 0) return false;
    if (omegaRps > kMT2_MaxOmegaRps) return false;

  // Weight by distance: farther tags → less trust
  double stdXY = Math.max(kMT2_MinStdDev, mt2.avgTagDist * kMT2_DistanceStdDevMultiplier);
  double yawScale = 1.0 + Math.min(kVisionYawSoftGateMaxScale - 1.0,
    Math.max(0.0, omegaRps - kVisionYawSoftGateStartOmegaRps) * kVisionYawSoftGateFactor);
  double stdYaw = Math.max(kMT2_MinStdDevYaw, kMT2_BaseStdDevYaw * yawScale);
  drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(stdXY, stdXY, stdYaw));

    SmartDashboard.putNumber("Vision/" + label + " Tags", mt2.tagCount);
    SmartDashboard.putNumber("Vision/" + label + " AvgDist", mt2.avgTagDist);
    SmartDashboard.putNumber("Vision/" + label + " StdDev", stdXY);
    return true;
  }

  public static void setlimelightsThrottles(int throttle){
        LimelightHelpers.SetThrottle(LimelightA,throttle);
        LimelightHelpers.SetThrottle(LimelightB,throttle);  
  }
}