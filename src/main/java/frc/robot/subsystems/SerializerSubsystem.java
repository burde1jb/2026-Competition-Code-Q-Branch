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

public class SerializerSubsystem extends SubsystemBase {
    SparkFlex SerializerMotor;

    public SerializerSubsystem() {
        SerializerMotor = new SparkFlex(RobotConstants.SerializerMotorCAN, MotorType.kBrushless);
    }
    
    public void serializerOn(boolean forward) {
        if (forward) {
            SerializerMotor.set(RobotConstants.SerializerOnspeed);
        } else {
            SerializerMotor.set(RobotConstants.SerializerOutspeed);
        }
    }

    public void serializerOff() {
        SerializerMotor.stopMotor();
    }

    public void serializerSlow(boolean forward) {
        if (forward) {
            SerializerMotor.set(RobotConstants.SerializerSlowspeed);
        } else {
            SerializerMotor.set(-RobotConstants.SerializerSlowspeed);
        }
    }
}