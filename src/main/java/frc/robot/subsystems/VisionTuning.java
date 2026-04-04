package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionTuning extends SubsystemBase {
  private final String prefix;
  private final VisionSubsystem visionSubsystem;

  public VisionTuning(String prefix, VisionSubsystem visionSubsystem) {
    this.prefix = prefix;
    this.visionSubsystem = visionSubsystem;
    publishDefaults();
  }

  private String key(String name) {
    return prefix + "/" + name;
  }
  private void publishDefaults() {
    SmartDashboard.putNumber(key("MT1 Min Tag Count"), visionSubsystem.kMT1_MinTagCount);
    SmartDashboard.putNumber(key("MT1 Max Speed Mps"), visionSubsystem.kMT1_MaxSpeedMps);
    SmartDashboard.putNumber(key("MT1 Max Omega Rps"), visionSubsystem.kMT1_MaxOmegaRps);
    SmartDashboard.putNumber(key("MT1 Far Distance Meters"), visionSubsystem.kMT1_FarDistanceMeters);
    SmartDashboard.putNumber(key("MT1 Single Tag Disabled StdDev Scale"), visionSubsystem.kMT1_SingleTagDisabledStdDevScale);
    SmartDashboard.putNumber(key("MT1 Close StdDev XY"), visionSubsystem.kMT1_CloseStdDevXY);
    SmartDashboard.putNumber(key("MT1 Far StdDev XY"), visionSubsystem.kMT1_FarStdDevXY);
    SmartDashboard.putNumber(key("MT1 Close StdDev Yaw"), visionSubsystem.kMT1_CloseStdDevYaw);
    SmartDashboard.putNumber(key("MT1 Far StdDev Yaw"), visionSubsystem.kMT1_FarStdDevYaw);

    SmartDashboard.putNumber(key("MT2 Max Omega Rps"), visionSubsystem.kMT2_MaxOmegaRps);
    SmartDashboard.putNumber(key("MT2 Distance StdDev Multiplier"), visionSubsystem.kMT2_DistanceStdDevMultiplier);
    SmartDashboard.putNumber(key("MT2 Min StdDev"), visionSubsystem.kMT2_MinStdDev);
    SmartDashboard.putNumber(key("MT2 Base StdDev Yaw"), visionSubsystem.kMT2_BaseStdDevYaw);
    SmartDashboard.putNumber(key("MT2 Min StdDev Yaw"), visionSubsystem.kMT2_MinStdDevYaw);

    SmartDashboard.putNumber(key("Yaw Soft Gate Start Omega Rps"), visionSubsystem.kVisionYawSoftGateStartOmegaRps);
    SmartDashboard.putNumber(key("Yaw Soft Gate Factor"), visionSubsystem.kVisionYawSoftGateFactor);
    SmartDashboard.putNumber(key("Yaw Soft Gate Max Scale"), visionSubsystem.kVisionYawSoftGateMaxScale);
  }

  public void updateFromDashboard() {
    visionSubsystem.kMT1_MinTagCount = (int) SmartDashboard.getNumber(key("MT1 Min Tag Count"), visionSubsystem.kMT1_MinTagCount);
    visionSubsystem.kMT1_MaxSpeedMps = SmartDashboard.getNumber(key("MT1 Max Speed Mps"), visionSubsystem.kMT1_MaxSpeedMps);
    visionSubsystem.kMT1_MaxOmegaRps = SmartDashboard.getNumber(key("MT1 Max Omega Rps"), visionSubsystem.kMT1_MaxOmegaRps);
    visionSubsystem.kMT1_FarDistanceMeters = SmartDashboard.getNumber(key("MT1 Far Distance Meters"), visionSubsystem.kMT1_FarDistanceMeters);
    visionSubsystem.kMT1_SingleTagDisabledStdDevScale = SmartDashboard.getNumber(key("MT1 Single Tag Disabled StdDev Scale"), visionSubsystem.kMT1_SingleTagDisabledStdDevScale);
    visionSubsystem.kMT1_CloseStdDevXY = SmartDashboard.getNumber(key("MT1 Close StdDev XY"), visionSubsystem.kMT1_CloseStdDevXY);
    visionSubsystem.kMT1_FarStdDevXY = SmartDashboard.getNumber(key("MT1 Far StdDev XY"), visionSubsystem.kMT1_FarStdDevXY);
    visionSubsystem.kMT1_CloseStdDevYaw = SmartDashboard.getNumber(key("MT1 Close StdDev Yaw"), visionSubsystem.kMT1_CloseStdDevYaw);
    visionSubsystem.kMT1_FarStdDevYaw = SmartDashboard.getNumber(key("MT1 Far StdDev Yaw"), visionSubsystem.kMT1_FarStdDevYaw);

    visionSubsystem.kMT2_MaxOmegaRps = SmartDashboard.getNumber(key("MT2 Max Omega Rps"), visionSubsystem.kMT2_MaxOmegaRps);
    visionSubsystem.kMT2_DistanceStdDevMultiplier = SmartDashboard.getNumber(key("MT2 Distance StdDev Multiplier"), visionSubsystem.kMT2_DistanceStdDevMultiplier);
    visionSubsystem.kMT2_MinStdDev = SmartDashboard.getNumber(key("MT2 Min StdDev"), visionSubsystem.kMT2_MinStdDev);
    visionSubsystem.kMT2_BaseStdDevYaw = SmartDashboard.getNumber(key("MT2 Base StdDev Yaw"), visionSubsystem.kMT2_BaseStdDevYaw);
    visionSubsystem.kMT2_MinStdDevYaw = SmartDashboard.getNumber(key("MT2 Min StdDev Yaw"), visionSubsystem.kMT2_MinStdDevYaw);

    visionSubsystem.kVisionYawSoftGateStartOmegaRps = SmartDashboard.getNumber(key("Yaw Soft Gate Start Omega Rps"), visionSubsystem.kVisionYawSoftGateStartOmegaRps);
    visionSubsystem.kVisionYawSoftGateFactor = SmartDashboard.getNumber(key("Yaw Soft Gate Factor"), visionSubsystem.kVisionYawSoftGateFactor);
    visionSubsystem.kVisionYawSoftGateMaxScale = SmartDashboard.getNumber(key("Yaw Soft Gate Max Scale"), visionSubsystem.kVisionYawSoftGateMaxScale);
  }

  @Override
  public void periodic() {
    updateFromDashboard();
  }

}
