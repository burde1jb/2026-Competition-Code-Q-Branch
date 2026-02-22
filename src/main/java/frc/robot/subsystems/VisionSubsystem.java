package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.*;

public class VisionSubsystem extends SubsystemBase {
  private RawFiducial[] fiducials;

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
        "limelight-four",
        -0.32, 
        -0.29,
        0.23,
        0,
        20,
        180);
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight-three",
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
    fiducials = LimelightHelpers.getRawFiducials("limelight-four");

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
}