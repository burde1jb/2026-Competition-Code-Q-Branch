package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class ConveyorSubsystem extends SubsystemBase {
    SparkFlex ConveyorMotor;
    private SparkClosedLoopController ConveyorMotorLoop;
    private SparkFlexConfig ConveyorMotorConfig;
    private RelativeEncoder ConveyorEncoder;
    // SparkFlexConfig ConveyorMotorConfig;
    // public DigitalInput sensor;
    // public SparkLimitSwitch beambreak;
    // public LimitSwitchConfig beambreakconfig;

    public ConveyorSubsystem() {
        ConveyorMotor = new SparkFlex(RobotConstants.ConveyorIntakeCANid, MotorType.kBrushless);
        ConveyorMotorLoop = ConveyorMotor.getClosedLoopController();
        ConveyorEncoder = ConveyorMotor.getEncoder();
        // ConveyorMotorConfig = new SparkFlexConfig();

        ConveyorMotorConfig
        .closedLoop
          .pid(0.00005, 0, 0) // slot 0
          .pid(0, 0, 0, ClosedLoopSlot.kSlot1) // slot 1
          .feedForward
            .kS(0.0) // slot 0 by default
            .kV(0.0019, ClosedLoopSlot.kSlot0) // slot 0 explicitly
            .kA(0.0)
            .sva(0, 0, 0, ClosedLoopSlot.kSlot1); // slot 1
        
            ConveyorMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            ConveyorMotor.configure(ConveyorMotorConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

            ConveyorEncoder.setPosition(0);
    }

    public void conveyorOn(double velocity) {
        ConveyorMotorLoop.setSetpoint(velocity, ControlType.kVelocity);
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