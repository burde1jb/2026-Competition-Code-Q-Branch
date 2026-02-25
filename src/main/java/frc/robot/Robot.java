// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.HubTracker;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    //CanBridge.runTCP(); //Used with Grapplehook LaserCAN
  }

  @Override
  public void robotPeriodic() {
  // UpdateDrivetrainFromLimelight();
    CommandScheduler.getInstance().run();

  double startTime = Timer.getFPGATimestamp();

  SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    SmartDashboard.putNumber(
        "HubTracker/Time Until Shift",
        HubTracker.timeRemainingInCurrentShift().orElse(Seconds.of(0)).in(Seconds));
    SmartDashboard.putBoolean(
        "HubTracker/RedWonAuto", HubTracker.getAutoWinner().orElse(Alliance.Blue) == Alliance.Red);
    SmartDashboard.putBoolean("HubTracker/GameDataPresent", !HubTracker.getAutoWinner().isEmpty());

    SmartDashboard.putNumber(
        "HubTracker/TimeUtilActive",
        HubTracker.timeUntilActive().orElse(Seconds.of(0)).in(Seconds));


  }
  // private final boolean kUseLimelight = true; //false if not using Limelight, true if using Limelight
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = inst.getTable("Q Branch");
  // private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("LLPose", Pose2d.struct).publish();

  private void UpdateDrivetrainFromLimelight() {
    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    // if (kUseLimelight) {
    //   var driveState = RobotContainer.drivetrain.getState();
    //   double headingDeg = driveState.Pose.getRotation().getDegrees();
    //   double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    //   LimelightHelpers.SetRobotOrientation("limelight-four", headingDeg, 0, 0, 0, 0, 0);
    //   var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four");
    //   if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
    //     RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
    //     SmartDashboard.putBoolean("usingLL",true);
    //     RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
    //   }
    //   else{
    //     SmartDashboard.putBoolean("usingLL",false);
    //   }
    //   drivePose.set(llMeasurement.pose);
    // }
  }

  @Override
  public void disabledInit() {
      VisionSubsystem.setlimelightsThrottles(200);//slows down limelight processing to keep from overheating while disabled. 
  }


  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    VisionSubsystem.setlimelightsThrottles(0);//sets limelight throttles back to 0 so they run at full speed during the match.
  }

  @Override
  public void autonomousInit() {
    CameraServer.startAutomaticCapture();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    CameraServer.startAutomaticCapture();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
