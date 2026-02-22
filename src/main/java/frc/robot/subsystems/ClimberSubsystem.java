package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX climberMotorUPPER;
    private TalonFXConfiguration climberMotorUPPERconfig = new TalonFXConfiguration();
    private TalonFX climberMotorLOWER;
    private BaseStatusSignal climberEncoderUPPER;
    // private AbsoluteEncoder climberEncoderLOWER;
    private final double rangeOffset = RobotConstants.ClimberrangeOffset;
    private final double encoderOffsetUPPER = RobotConstants.ClimberencoderUPPERoffset;
    private final double encoderOffsetLOWER = RobotConstants.ClimberencoderLOWERoffset;

    public ClimberSubsystem() {
        climberMotorUPPER = new TalonFX(RobotConstants.ClimbermotorUPPERcanID, "drivetrain");
        climberMotorUPPERconfig = new TalonFXConfiguration();
        BaseStatusSignal.setUpdateFrequencyForAll(250,climberMotorUPPER.getPosition());
        climberEncoderUPPER = climberMotorUPPER.getPosition();
        // climberMotorLOWER = new SparkFlex(RobotConstants.ClimbermotorLOWERcanID, MotorType.kBrushless);
        // climberEncoderLOWER = climberMotorLOWER.getAbsoluteEncoder();
    }

    // public void LowergoTo(double encoderGoalLOWER) {
    //     if ((climberEncoderLOWER.getValueAsDouble()) < (encoderGoalLOWER + encoderOffsetLOWER - rangeOffset)) {
    //         climberMotorLOWER.set(RobotConstants.ClimberReleasepower);
    //     } else if ((climberEncoderLOWER.getValueAsDouble()) > (encoderGoalLOWER + rangeOffset)) {
    //         climberMotorLOWER.set(RobotConstants.ClimberClimbpower);
    //     } else {
    //         climberMotorLOWER.stopMotor();
    //     }
    // }

    public void UppergoTo(double encoderGoalUPPER) {
        double position = climberMotorUPPER.getPosition().getValueAsDouble(); 
        if ((position) < (encoderGoalUPPER + encoderOffsetUPPER - rangeOffset)) {
            climberMotorUPPER.set(RobotConstants.ClimberClimbpower);
        } else if ((position) > (encoderGoalUPPER + rangeOffset)) {
            climberMotorUPPER.set(RobotConstants.ClimberReleasepower);
        } else {
            climberMotorUPPER.stopMotor();
        }
    }

    public boolean UPPERwentTo (double encoderGoalUPPER) {
        double position = climberMotorUPPER.getPosition().getValueAsDouble(); 
        if ((position) < (encoderGoalUPPER + encoderOffsetUPPER - rangeOffset)) {
            climberMotorUPPER.set(RobotConstants.ClimberClimbpower);
            return false;
        } else if ((position) > (encoderGoalUPPER + rangeOffset)) {
            climberMotorUPPER.set(RobotConstants.ClimberReleasepower);
            return false;
        } else {
            climberMotorUPPER.stopMotor();
            return true;
        }
    }

    public void goDOWN() {
        climberMotorUPPER.set(RobotConstants.ClimberClimbpower);
    }

    public void goUP() {
        climberMotorUPPER.set(RobotConstants.ClimberReleasepower);
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
        double position = climberMotorUPPER.getPosition().getValueAsDouble(); 
        if (position == distance) {
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
        SmartDashboard.putNumber("Climber Encoder", (climberEncoderUPPER.getValueAsDouble()));
        // SmartDashboard.putNumber("Climber Encoder", (climberEncoderLOWER.getPosition()));
    }
}
