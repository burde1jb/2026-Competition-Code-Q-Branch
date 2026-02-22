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

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    //CanBridge.runTCP(); //Used with Grapplehook LaserCAN
  }

  @Override
  public void robotPeriodic() {
  //UpdateDrivetrainFromLimelight();//vision subsystem is a subsystem and has its own periodic
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
  
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public static final NetworkTable driveStateTable = inst.getTable("Q Branch");


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
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
