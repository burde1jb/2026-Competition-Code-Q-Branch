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
  SparkFlex SerializerMotor2;

    public SerializerSubsystem() {
        SerializerMotor = new SparkFlex(RobotConstants.SerializerMotorCAN, MotorType.kBrushless);
           SerializerMotor2 = new SparkFlex(RobotConstants.SerializerMotorCAN2, MotorType.kBrushless);
    }
    
    
    public void serializerOn(boolean forward) {
        if (forward) {
            SerializerMotor.set(RobotConstants.SerializerOnspeed);
                  SerializerMotor2.set(-RobotConstants.SerializerOnspeed);
        } else {
            SerializerMotor.set(RobotConstants.SerializerOutspeed);
             SerializerMotor2.set(-RobotConstants.SerializerOutspeed);
        }
    }

    public void serializerOnAuto(double power) {
        SerializerMotor.set(power);
          SerializerMotor2.set(-power);
    }

    public void serializerOff() {
        SerializerMotor.stopMotor();
        SerializerMotor2.stopMotor();
    }

    public void serializerSlow(boolean forward) {
        if (forward) {
            SerializerMotor.set(RobotConstants.SerializerSlowspeed);
             SerializerMotor2.set(-RobotConstants.SerializerSlowspeed);
        } else {
            SerializerMotor.set(-RobotConstants.SerializerSlowspeed);
            SerializerMotor2.set(RobotConstants.SerializerSlowspeed);
        }
        }
    }
