package frc.robot.subsystems;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class FuelShooterSubsystem extends SubsystemBase {
    //name of this subsystem for dashboard labeling
    String className = this.getClass().getSimpleName();
    private SparkFlex FuelShooterMotor = new SparkFlex(RobotConstants.FuelShooterMotorCANid, MotorType.kBrushless);
    private SparkFlex FuelShooterMotor2 = new SparkFlex(RobotConstants.FuelShooterMotor2CANid, MotorType.kBrushless);
    private SparkFlex FuelShooterMotor3 = new SparkFlex(RobotConstants.FuelShooterMotor3CANid, MotorType.kBrushless);
    private SparkClosedLoopController FuelShooterMotorLoop = FuelShooterMotor.getClosedLoopController();
    private SparkFlexConfig FuelShooterMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig FuelShooterMotorConfig2 = new SparkFlexConfig();
    private SparkFlexConfig FuelShooterMotorConfig3 = new SparkFlexConfig();
    private RelativeEncoder FuelShooterEncoder = FuelShooterMotor.getEncoder();

    public double MaxVelocity =  RobotConstants.FuelShooterMaxVelocity; // rotations
    public double FuelShooterVelocity = 0;
    private double FuelShooterTargetVelocity = 0.0;
    private double P = 0.00005;
    private double i = 0.0;
    private double d = 0.0;
    private double S = 0.0;
    private double v = 0.0019;
    private double A = 0.0;
   
    public FuelShooterSubsystem() {
      //setup defaults
      SmartDashboard.putNumber("Shooter | PID | kP", P);
      SmartDashboard.putNumber("Shooter | PID | kI", i);  
      SmartDashboard.putNumber("Shooter | PID | kD", d);
      SmartDashboard.putNumber("Shooter | PID | kS", S);
      SmartDashboard.putNumber("Shooter | PID | kA", A);
      SmartDashboard.putNumber("Shooter | PID | kV", v);
        // Set PID gains
        FuelShooterMotorConfig
        .closedLoop
          .pid(P, i, d) // slot 0
          .feedForward
            .sva(S, v, A);
            // .kG(0) // Only use one of kG and kCos
            // .kCos(0)
            // .kCosRatio(1)

        FuelShooterMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // FuelShooterMotorConfig.closedLoop.maxMotion.cruiseVelocity(2000);
        // FuelShooterMotorConfig.closedLoop.maxMotion.maxAcceleration(1000);

        FuelShooterMotorConfig.encoder //https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567/2
        .uvwMeasurementPeriod(8)
        .quadratureAverageDepth(2)
        .quadratureMeasurementPeriod(8);

        //FuelShooterMotorConfig.inverted(true); // invert the first motor because of how the motors are mounted we want Shooting to be a positive velocity not -3300 (its a semantic change really but it makes it easier to understand when we are trying to shoot at a certain velocity, we can just set the velocity to a positive number instead of a negative number)
        //FuelShooterMotorConfig.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2); //https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567/2
        //FuelShooterMotorConfig2.follow(FuelShooterMotor, false); // set the second motor to follow the first motor, and dont invert it because of how the motors are mounted.
        
        // Configure follower behavior through the motor configs before applying them.
        FuelShooterMotorConfig2.follow(FuelShooterMotor, false);
        FuelShooterMotorConfig3.follow(FuelShooterMotor, false);

        FuelShooterMotor.configure(FuelShooterMotorConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        FuelShooterMotor2.configure(FuelShooterMotorConfig2, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        FuelShooterMotor3.configure(FuelShooterMotorConfig3, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        FuelShooterEncoder.setPosition(0);
    }

    public void stop() {
        FuelShooterMotor.set(0);
    
    }
    public void shooterOn(double velocity){
      // Followers will automatically follow the first motor.
      FuelShooterMotorLoop.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
      FuelShooterTargetVelocity = velocity;
    }

    public void shooterSpeed(double power){
      FuelShooterMotor.set(power);
    }

    public boolean atSpeed () {
      if (Math.abs(FuelShooterEncoder.getVelocity()) >= RobotConstants.FuelShooterMaxVelocity * 0.96){
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
        SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", FuelShooterTargetVelocity);
        SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity", FuelShooterEncoder.getVelocity());
        FuelShooterVelocity = FuelShooterEncoder.getVelocity();
         pidtune();
    }
         
    private void pidtune() {
      double kP = SmartDashboard.getNumber("Shooter | PID | kP", 0.00005);
      double kI = SmartDashboard.getNumber("Shooter | PID | kI", 0);  
      double kD = SmartDashboard.getNumber("Shooter | PID | kD", 0);
      double kS = SmartDashboard.getNumber("Shooter | PID | kS", 0.0);
      double kA = SmartDashboard.getNumber("Shooter | PID | kA", 0.0);
      double kV = SmartDashboard.getNumber("Shooter | PID | kV", 0.0019);

      if(kP != P || kI != i || kD != d || kS != S || kA != A || kV != v) {
          P = kP;
          i = kI;
          d = kD;
          S = kS;
          A = kA;
          v = kV;
          //setup the config
          FuelShooterMotorConfig
              .closedLoop
                .pid(P, i, d) // slot 0
                .feedForward
                  .sva(S, v, A);
            //push config
          FuelShooterMotor.configure(FuelShooterMotorConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
      }
    }
     

}
