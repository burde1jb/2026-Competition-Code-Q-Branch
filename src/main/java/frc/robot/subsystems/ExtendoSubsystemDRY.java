// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.LimitSwitchConfig;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.AbsoluteEncoder;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;
// import frc.robot.RobotConstants;

// public class ExtendoSubsystemDRY extends SubsystemBase {
//     SparkFlex ExtendoMotor;
//     AbsoluteEncoder ExtendoEncoder;
//     private final double rangeOffset = RobotConstants.ExtendoRangeOffset;
//     private final double encoderOffset = RobotConstants.ExtendoEncoderOffset;
//     private final double PositionalTolerance = 2 * rangeOffset;
    
//     public double simposition = 0;

//     public ExtendoSubsystemDRY() {
//         ExtendoMotor = new SparkFlex(RobotConstants.ExtendoMotorCANid, MotorType.kBrushless);
//         ExtendoEncoder = ExtendoMotor.getAbsoluteEncoder();
//     }

//     // This will return the subsytems current position 
//     public double getEncoderMeasurement() {
//         double RawPosition = ExtendoEncoder.getPosition();
//         double position =  RawPosition + rangeOffset + encoderOffset % 1; //why do we modulo 1 here? this will always return a remainder and not the correct position?
//         SmartDashboard.putNumber("Extendoposition", position);
//         SmartDashboard.putNumber("RawExtendoposition", RawPosition);
        
//         if(Robot.isSimulation())
//         {
//             return simposition;
//         }
//         else
//         {
//             return position;
//         }
        
//     }
    
//     //Goto Holds logic about going to a position, it does not care about implementation of those commands. its a single responsibility. 
//     public boolean goTo(double degrees) {

//         var position = getEncoderMeasurement();
        
//         //if we are within tolerance of the requested position, then this is true.
//         //requested positions must be outside tolerance for action to be taken.
//         boolean isAtposition = MathUtil.isNear(degrees, position, PositionalTolerance);
//         if(isAtposition)
//         {
//             stop();
//             return true;
//         }

//         //we now know we arent within the tolerance of the requested position. do we go up or down. 
//         if (position < degrees) {
//             Extend();
//         } else {
//             Retract();
//         }
//         return false;
//     }
    
//     //this will extend the motor unless it is overextended then it stops. 
//     public void Extend() {
//         if (getEncoderMeasurement() < (RobotConstants.ExtendoExtendL4)) {
//         MoveMotor(RobotConstants.ExtendoExtendSpeed);
//         }
//         else {
//             stop();
//         }
//     } 
//     //this will retract unless motor is too retracted then it will only stop motor.
//     public void Retract() {
//         if (getEncoderMeasurement() > (RobotConstants.ExtendoRetract)){
//         MoveMotor(RobotConstants.ExtendoRetractSpeed);
//         }
//         else {
//             this.stop();
//         }
//     } 

//     //another option for the joystick control
//     // public void Move(double MovePower) {
//     //     if (MovePower > 0) {
//     //         MoveMotor(MovePower*RobotConstants.ExtendoExtendSpeed);
//     //     }
//     //     else {
//     //         MoveMotor(MovePower*RobotConstants.ExtendoRetractSpeed);
//     //     }
//     // } 

//     //everything will call this function to move the motor. seperated from the business logic this allows us to change implementation without touching the login of the subsystem.
//     public void MoveMotor(double speed)
//     {
//         if(Robot.isSimulation())
//         {
//             simposition += speed / 50; //Fake Sim numbers to demo operation. 
//         }
//         ExtendoMotor.set(speed);
//     }
//     //whichever way we choose to stop the motor goes here. this way we can change the entire class from brake to coast or to hold position without having to change any other business logic or codes. the entire implementation is here.
//     public void stop() {
//         //same thing but we are using .set(0);
//         MoveMotor(0);//ExtendoMotor.stopMotor();
//     }

//     @Override
//     public void periodic() {
//          SmartDashboard.putNumber("Extendo SIM Encoder", simposition);//why does this not include the offset?
//         SmartDashboard.putNumber("Extendo Encoder", (ExtendoEncoder.getPosition() + encoderOffset) % 1);//why does this not include the offset?
//         getEncoderMeasurement();
//     }
// }