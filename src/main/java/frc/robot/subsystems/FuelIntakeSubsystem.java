package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class FuelIntakeSubsystem extends SubsystemBase {
    SparkFlex intakeMotor;
    SparkFlex FuelIntakeWristMotor;
    AbsoluteEncoder FuelIntakeWristEncoder;
    SparkFlexConfig FuelIntakeWristMotorConfig;
    private SparkClosedLoopController FuelIntakeMotorLoop;
    private SparkFlexConfig FuelIntakeMotorConfig;
    private RelativeEncoder FuelIntakeEncoder;
    private final double rangeOffset = RobotConstants.FuelWristrangeOffset;
    private final double encoderOffset = RobotConstants.FuelWristencoderOffset;

    public FuelIntakeSubsystem() {
        intakeMotor = new SparkFlex(RobotConstants.FuelIntakeCANid,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        FuelIntakeMotorLoop = intakeMotor.getClosedLoopController();
        FuelIntakeWristMotor = new SparkFlex(RobotConstants.FuelIntakeWristMotorCANid, MotorType.kBrushless);
        FuelIntakeWristEncoder = FuelIntakeWristMotor.getAbsoluteEncoder();
        FuelIntakeWristMotorConfig = new SparkFlexConfig();

        FuelIntakeMotorConfig
        .closedLoop
          .pid(0.00005, 0, 0) // slot 0
          .pid(0, 0, 0, ClosedLoopSlot.kSlot1) // slot 1
          .feedForward
            .kS(0.0) // slot 0 by default
            .kV(0.0019, ClosedLoopSlot.kSlot0) // slot 0 explicitly
            .kA(0.0)
            .sva(0, 0, 0, ClosedLoopSlot.kSlot1); // slot 1
        
            FuelIntakeMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            intakeMotor.configure(FuelIntakeMotorConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

            FuelIntakeEncoder.setPosition(0);

        
    }

    public void FuelIntakeOn(double velocity) {
        FuelIntakeMotorLoop.setSetpoint(velocity, ControlType.kVelocity);
    }
    
    public void FuelIntakeOut(boolean backwards) {
        intakeMotor.set(RobotConstants.FuelIntakeOnspeed);
    }

    public void FuelIntakeOff() {
        intakeMotor.stopMotor();
    }

    public void wristOn(boolean forward) {
        if (forward) {
            FuelIntakeWristMotor.set(RobotConstants.FuelWristExtendpower);
        } else {
            FuelIntakeWristMotor.set(RobotConstants.FuelWristRetractpower);
        }
    }

    public void goTo(double encoderGoal) {
        if ((FuelIntakeWristEncoder.getPosition() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1) {
            FuelIntakeWristMotor.set(RobotConstants.FuelWristExtendpower);
        } else if ((FuelIntakeWristEncoder.getPosition() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1) {
            FuelIntakeWristMotor.set(RobotConstants.FuelWristRetractpower);
        } else {
            FuelIntakeWristMotor.stopMotor();
        }
        }

    public boolean wentTo(double encoderGoal) {
        if ((FuelIntakeWristEncoder.getPosition() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1) {
            this.wristExtend();
            return false;
        } else if ((FuelIntakeWristEncoder.getPosition() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1) {
            this.wristRetract();
            return false;
        } else {
            this.wristOff();
            return true;
        }
    }

    public void wristExtend(){
        FuelIntakeWristMotor.set(RobotConstants.FuelWristExtendpower);
    }

     public void wristRetract(){
        FuelIntakeWristMotor.set(RobotConstants.FuelWristRetractpower);
    }


    public void wristOff() {
        FuelIntakeWristMotor.stopMotor();
    }

    // public void FuelIntakeSlow(boolean forward) {
    //     if (forward) {
    //         intakeMotor.set(RobotConstants.FuelIntakeSlowspeed);
    //     } else {
    //         intakeMotor.set(-RobotConstants.FuelIntakeSlowspeed);
    //     }

    @Override

    public void periodic(){
        SmartDashboard.putNumber("IntakeEncoder", FuelIntakeWristEncoder.getPosition());
    }
}