package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {
    private SparkFlex climberMotorUPPER;
    private SparkFlex climberMotorLOWER;
    private AbsoluteEncoder climberEncoderUPPER;
    // private AbsoluteEncoder climberEncoderLOWER;
    private final double rangeOffset = RobotConstants.ClimberrangeOffset;
    private final double encoderOffsetUPPER = RobotConstants.ClimberencoderLOWERoffset;
    private final double encoderOffsetLOWER = RobotConstants.ClimberencoderUPPERoffset;

    public ClimberSubsystem() {
        climberMotorUPPER = new SparkFlex(RobotConstants.ClimbermotorUPPERcanID,
                MotorType.kBrushless);
        climberEncoderUPPER = climberMotorUPPER.getAbsoluteEncoder();
        // climberMotorLOWER = new SparkFlex(RobotConstants.ClimbermotorLOWERcanID, MotorType.kBrushless);
        // climberEncoderLOWER = climberMotorLOWER.getAbsoluteEncoder();
    }

    // public void LowergoTo(double encoderGoalLOWER) {
    //     if ((climberEncoderLOWER.getPosition()) < (encoderGoalLOWER + encoderOffsetLOWER - rangeOffset)) {
    //         climberMotorLOWER.set(RobotConstants.ClimberReleasepower);
    //     } else if ((climberEncoderLOWER.getPosition()) > (encoderGoalLOWER + rangeOffset)) {
    //         climberMotorLOWER.set(RobotConstants.ClimberClimbpower);
    //     } else {
    //         climberMotorLOWER.stopMotor();
    //     }
    // }

    public void UppergoTo(double encoderGoalUPPER) {
        if ((climberEncoderUPPER.getPosition()) < (encoderGoalUPPER + encoderOffsetUPPER - rangeOffset)) {
            climberMotorUPPER.set(RobotConstants.ClimberReleasepower);
        } else if ((climberEncoderUPPER.getPosition()) > (encoderGoalUPPER + rangeOffset)) {
            climberMotorUPPER.set(RobotConstants.ClimberClimbpower);
        } else {
            climberMotorUPPER.stopMotor();
        }
    }

    public boolean UPPERwentTo (double encoderGoalUPPER) {
        if ((climberEncoderUPPER.getPosition()) < (encoderGoalUPPER + encoderOffsetUPPER - rangeOffset)) {
            climberMotorUPPER.set(RobotConstants.ClimberReleasepower);
            return false;
        } else if ((climberEncoderUPPER.getPosition()) > (encoderGoalUPPER + rangeOffset)) {
            climberMotorUPPER.set(RobotConstants.ClimberClimbpower);
            return false;
        } else {
            climberMotorUPPER.stopMotor();
            return true;
        }
    }

    public void setDOWN() {
        // this.LowergoTo(0);
        this.UppergoTo(0);
    }

    public void stop() {
        climberMotorUPPER.stopMotor();
        // climberMotorLOWER.stopMotor();
    }

    public boolean encoderCheckUpper(double distance) {
        if (climberEncoderUPPER.getPosition() == distance) {
            return true;
        }
        return false;
    }
    // public boolean encoderCheckLower(double distance) {
    //     if (climberEncoderLOWER.getPosition() == distance) {
    //         return true;
    //     }
    //     return false;
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Encoder", (climberEncoderUPPER.getPosition()));
        // SmartDashboard.putNumber("Climber Encoder", (climberEncoderLOWER.getPosition()));
    }
}
