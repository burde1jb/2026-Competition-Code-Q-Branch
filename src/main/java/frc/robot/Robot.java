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
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public static final NetworkTable driveStateTable = inst.getTable("Q Branch");

  public Robot() {
    m_robotContainer = new RobotContainer();
    //CanBridge.runTCP(); //Used with Grapplehook LaserCAN
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    double startTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    HubTracker.PutHubTrackerInfoToDashboard();

  }


  
 


  @Override
  public void disabledInit() {
    VisionSubsystem.setlimelightsThrottles(100);//slows down limelight processing to keep from overheating while disabled. 
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
    //CameraServer.startAutomaticCapture();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);// updated deprecated command -> m_autonomousCommand.schedule();
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
   // CameraServer.startAutomaticCapture();
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
