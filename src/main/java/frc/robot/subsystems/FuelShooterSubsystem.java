package frc.robot.subsystems;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class FuelShooterSubsystem extends SubsystemBase {
    //name of this subsystem for dashboard labeling
    String className = this.getClass().getSimpleName();
    private SparkFlex FuelShooterMotor = new SparkFlex(RobotConstants.FuelShooterMotorCANid, MotorType.kBrushless);
    private SparkFlex FuelShooterMotor2 = new SparkFlex(RobotConstants.FuelShooterMotor2CANid, MotorType.kBrushless);
    private SparkClosedLoopController FuelShooterMotorLoop = FuelShooterMotor.getClosedLoopController();
    private SparkClosedLoopController FuelShooterMotorLoop2 = FuelShooterMotor2.getClosedLoopController();;
    private SparkFlexConfig FuelShooterMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig FuelShooterMotorConfig2 = new SparkFlexConfig();
    private RelativeEncoder FuelShooterEncoder = FuelShooterMotor.getEncoder();
    private RelativeEncoder FuelShooterEncoder2 = FuelShooterMotor2.getEncoder();
   
    private double MaxVelocity =  RobotConstants.FuelShooterMaxVelocity; // rotations
    private double FuelShooterVelocity = FuelShooterEncoder.getVelocity();
    private double FuelShooterVelocity2 = FuelShooterEncoder2.getVelocity();

      //the built in PID controller on the Spark(Max Or Flex) motor controller, these will not take processing power from the roborio because they are running on the motor controller itself at a much higher loop rate (this is good for fast response and precision)
    private SparkClosedLoopController SparkMaxBuiltInPidController;

    private double FuelShooterTargetVelocity = 0.0;

   
    public FuelShooterSubsystem() {

        // Set PID gains
        FuelShooterMotorConfig
        .closedLoop
          .pid(0.00005, 0, 0) // slot 0
          .pid(0, 0, 0, ClosedLoopSlot.kSlot1) // slot 1
          .feedForward
            .kS(0.0) // slot 0 by default
            .kV(0.0019, ClosedLoopSlot.kSlot0) // slot 0 explicitly
            .kA(0.0)
            // .kG(0) // Only use one of kG and kCos
            // .kCos(0)
            // .kCosRatio(1)
           
            .sva(0, 0, 0, ClosedLoopSlot.kSlot1); // slot 1

        FuelShooterMotorConfig2
        .closedLoop
          .pid(0.00005, 0, 0) // slot 0
          .pid(0, 0, 0, ClosedLoopSlot.kSlot1) // slot 1
          .feedForward
            .kS(0.0) // slot 0 by default
            .kV(0.0019, ClosedLoopSlot.kSlot0) // slot 0 explicitly
            .kA(0.0)
            // .kG(0) // Only use one of kG and kCos
            // .kCos(0)
            // .kCosRatio(1)
        
            .sva(0, 0, 0, ClosedLoopSlot.kSlot1); // slot 1

        FuelShooterMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        FuelShooterMotorConfig2.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        // FuelShooterMotorConfig.closedLoop.maxMotion.cruiseVelocity(2000);
        // FuelShooterMotorConfig.closedLoop.maxMotion.maxAcceleration(1000);
        // FuelShooterMotorConfig2.closedLoop.maxMotion.cruiseVelocity(2000);
        // FuelShooterMotorConfig2.closedLoop.maxMotion.maxAcceleration(1000);

        
        // SetupMotorConfig();

        FuelShooterMotor.configure(FuelShooterMotorConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        FuelShooterMotor2.configure(FuelShooterMotorConfig2, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        FuelShooterEncoder.setPosition(0);
        FuelShooterEncoder2.setPosition(0);

    }

    public void stop() {
    //     //same thing but we are using .set(0);
        FuelShooterMotor.stopMotor();
        FuelShooterMotor2.stopMotor();
        FuelShooterMotor.set(0);
        FuelShooterMotor2.set(0);
        // FuelShooterMotor3.set(0);

     }
    public void shooterOn(double velocity){
        FuelShooterMotorLoop.setSetpoint(velocity, ControlType.kVelocity);
        FuelShooterMotorLoop2.setSetpoint(velocity, ControlType.kVelocity);
        FuelShooterTargetVelocity = velocity;
    }

    public void shooterSpeed(double power){
      FuelShooterMotor.set(power);
      FuelShooterMotor2.set(power);
    }

    public void shooterBangBang(double power){
      if (FuelShooterEncoder.getVelocity() < (1000 * 0.98)) {
        FuelShooterMotor.set(power);
        FuelShooterMotor2.set(power);
      }
      else {
        FuelShooterMotor.set(0);
        FuelShooterMotor2.set(0);
      }
    }

    public Command runFlywheelCommand() {
    return this.startEnd(
        () -> {
          this.shooterOn(-1300);
        },
        () -> {
          this.shooterOn(0.0);
        });
  }

    public boolean atSpeed () {
      if (FuelShooterEncoder.getVelocity() <= RobotConstants.FuelShooterMaxVelocity * 0.98){
        return true;
      }
      else {
        return false;
      }
    }


    @Override
   
    public void periodic() {
        SmartDashboard.putNumber("Shooter | Flywheel | Applied Output", FuelShooterMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter | Flywheel | Current", FuelShooterMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter2 | Flywheel | Applied Output", FuelShooterMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter2 | Flywheel | Current", FuelShooterMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", FuelShooterTargetVelocity);
        SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity", FuelShooterEncoder.getVelocity());
        // pidtune();
    }
     

}
