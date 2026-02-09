// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.LimitSwitchConfig;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotConstants;
// import com.revrobotics.AbsoluteEncoder;

// public class FuelIntakeWristSubsystem extends SubsystemBase {
//     SparkFlex FuelIntakeWristMotor;
//     AbsoluteEncoder FuelIntakeWristEncoder;
//     SparkFlexConfig FuelIntakeWristMotorConfig;
//     // SparkFlexConfig ConveyorMotorConfig;
//     private final double rangeOffset = RobotConstants.FuelWristrangeOffset;
//     private final double encoderOffset = RobotConstants.FuelWristencoderOffset;

//     public FuelIntakeWristSubsystem() {
//         FuelIntakeWristMotor = new SparkFlex(RobotConstants.FuelIntakeWristMotorCANid, MotorType.kBrushless);
//         FuelIntakeWristEncoder = FuelIntakeWristMotor.getAbsoluteEncoder();
//         FuelIntakeWristMotorConfig = new SparkFlexConfig();
//     }

//     public void wristOn(boolean forward) {
//         if (forward) {
//             FuelIntakeWristMotor.set(RobotConstants.FuelWristExtendpower);
//         } else {
//             FuelIntakeWristMotor.set(RobotConstants.FuelWristRetractpower);
//         }
//     }

//     public void goTo(double encoderGoal) {
//         if ((FuelIntakeWristEncoder.getPosition() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1) {
//             FuelIntakeWristMotor.set(RobotConstants.FuelWristExtendpower);
//         } else if ((FuelIntakeWristEncoder.getPosition() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1) {
//             FuelIntakeWristMotor.set(RobotConstants.FuelWristRetractpower);
//         } else {
//             FuelIntakeWristMotor.stopMotor();
//         }
//         }

//     public boolean wentTo(double encoderGoal) {
//         if ((FuelIntakeWristEncoder.getPosition() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1) {
//             this.wristExtend();
//             return false;
//         } else if ((FuelIntakeWristEncoder.getPosition() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1) {
//             this.wristRetract();
//             return false;
//         } else {
//             this.wristOff();
//             return true;
//         }
//     }

//     public void wristExtend(){
//         FuelIntakeWristMotor.set(RobotConstants.FuelWristExtendpower);
//     }

//      public void wristRetract(){
//         FuelIntakeWristMotor.set(RobotConstants.FuelWristRetractpower);
//     }


//     public void wristOff() {
//         FuelIntakeWristMotor.stopMotor();
//     }


// }