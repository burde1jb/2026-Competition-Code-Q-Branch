package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
class PIDControllerConfigurable extends PIDController {
  public PIDControllerConfigurable(double kP, double kI, double kD) {
      super(kP, kI, kD);
  }
  
  public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
      super(kP, kI, kD);
      this.setTolerance(tolerance);
  }
}
public class AlignCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_Limelight;
  // private final int tagID;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.075, 0, 0, 0.01);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.3, 0.01, 0.01, 0.01);
  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  private static int tagID = -1;
  
  public double rotationalRate = 0;
  public double velocityX = 0;

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    addRequirements(m_Limelight);
  }

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int ID) throws IllegalArgumentException{
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    if (ID<0){throw new IllegalArgumentException("april tag id cannot be negative");}
    tagID = ID;
    addRequirements(m_Limelight);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    RawFiducial fiducial;
    Pose3d TagFromRobot = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    try {
      if (tagID==-1){
        fiducial = m_Limelight.getFiducialWithId(m_Limelight.getClosestFiducial().id);
      }
      else{
        fiducial = m_Limelight.getFiducialWithId(tagID);
      }

      final double rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;
      
      final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.0;
      final double velocityY = yPidController.calculate(fiducial.distToRobot, 0.1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.2;
        
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) {
        this.end(true);
      }

      // SmartDashboard.putNumber("txnc", fiducial.txnc);
      // SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
      // SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      // SmartDashboard.putNumber("xPidController", velocityX);
      // SmartDashboard.putNumber("yVelocity", velocityY);
      m_drivetrain.setControl(
          alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX).withVelocityY(velocityY));
 
    } catch (VisionSubsystem.NoSuchTargetException nste) {
      System.out.println("Tag not found");
    }
  }

  @Override
  public boolean isFinished() {
    return rotationalPidController.atSetpoint() && yPidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }
}