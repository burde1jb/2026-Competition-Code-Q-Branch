package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class ConveyorSubsystem extends SubsystemBase {
    SparkFlex ConveyorMotor;
    // SparkFlexConfig ConveyorMotorConfig;
    // public DigitalInput sensor;
    // public SparkLimitSwitch beambreak;
    // public LimitSwitchConfig beambreakconfig;

    public ConveyorSubsystem() {
        ConveyorMotor = new SparkFlex(RobotConstants.ConveyorIntakeCANid, MotorType.kBrushless);
        // ConveyorMotorConfig = new SparkFlexConfig();
    }

    public void conveyorOn(boolean forward) {
        if (forward) {
            ConveyorMotor.set(RobotConstants.ConveyorIntakeOnspeed);
        } else {
            ConveyorMotor.set(RobotConstants.ConveyorIntakeOutspeed);
        }
    }

    // public void intakeOnAuton(boolean forward) {
    //     if (forward) {
    //         ConveyorMotor.set(RobotConstants.ConveyorIntakeOnspeedAuton);
    //     } else {
    //         ConveyorMotor.set(RobotConstants.ConveyorIntakeOutspeed);
    //     }
    // }

    // public void intakeOnBypass() {
    //     ConveyorMotor.set(RobotConstants.ConveyorIntakeOnspeed);
    // }

    public void conveyorOff() {
        ConveyorMotor.stopMotor();
    }

    public void conveyorSlow(boolean forward) {
        if (forward) {
            ConveyorMotor.set(RobotConstants.ConveyorIntakeSlowspeed);
        } else {
            ConveyorMotor.set(-RobotConstants.ConveyorIntakeSlowspeed);
        }
    }


}