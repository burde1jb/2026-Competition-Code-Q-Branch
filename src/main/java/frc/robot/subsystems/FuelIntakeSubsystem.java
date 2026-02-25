package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
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
    private final double rangeOffset = RobotConstants.FuelWristrangeOffset;
    private final double encoderOffset = RobotConstants.FuelWristencoderOffset;

    public FuelIntakeSubsystem() {
        intakeMotor = new SparkFlex(RobotConstants.FuelIntakeCANid,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        FuelIntakeWristMotor = new SparkFlex(RobotConstants.FuelIntakeWristMotorCANid, MotorType.kBrushless);
        FuelIntakeWristEncoder = FuelIntakeWristMotor.getAbsoluteEncoder();
        FuelIntakeWristMotorConfig = new SparkFlexConfig();
    }

    public void FuelIntakeOn(boolean forward) {
        intakeMotor.set(RobotConstants.FuelIntakeOnspeed);
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