// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
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

// public class ExtendoSubsystemPID1 extends SubsystemBase {
//     SparkFlex ExtendoMotor;
//     AbsoluteEncoder ExtendoEncoder;
//     // min position .006
//     // max positoions .66
//     private final double rangeOffset = RobotConstants.ExtendoRangeOffset;
//     private final double encoderOffset = RobotConstants.ExtendoEncoderOffset;
//     private final double PositionalTolerance = 2 * rangeOffset;
    
//     public double simposition = RobotConstants.ExtendoRetract;//this starts the simulation at the min postion

//     public ExtendoSubsystemPID1() {
//         ExtendoMotor = new SparkFlex(RobotConstants.ExtendoMotorCANid, MotorType.kBrushless);
//         ExtendoEncoder = ExtendoMotor.getAbsoluteEncoder();
//         SparkFlexConfig newcConfig = new SparkFlexConfig();
//         newcConfig.absoluteEncoder.zeroOffset(.67);
//         ExtendoMotor.configure(newcConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

//     // this returns a command that can be called when needed. it is part of this class but is not really "part"  of this subsystem. its a factory that builds a command for other parts of the program to use.  
//     //https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#capturing-state-in-inline-commands
//     public Command goTo(double targetRotorPosition) {
//         // Create a controller for the inline command to capture
//         PIDController controller = new PIDController(RobotConstants.ExtendoMotorP, 0, 0);
//         // We can do whatever configuration we want on the created state before returning from the factory
//         controller.setTolerance(PositionalTolerance);
//         // calculate the DutyCycle applied to the motor (-1.0 to 1.0)
        
//         // Try to move until we're at the setpoint, then stop
//         return run(() -> 
//         {
//             var PidControllerForce = controller.calculate(getEncoderMeasurement(), targetRotorPosition); 
//             SmartDashboard.putNumber("targetRotorPosition", targetRotorPosition);
//             MoveMotor(PidControllerForce);

//         })
//             .until(controller::atSetpoint)
//             .andThen(runOnce(() -> stop()));
//     }   
    
//     //another option for the joystick control
//     public void Move(double MovePower) {
//         if (MovePower > 0) {
//             MoveMotor(MovePower*RobotConstants.ExtendoExtendSpeed);
//         }
//         else {
//             MoveMotor(MovePower*RobotConstants.ExtendoRetractSpeed);
//         }
//     } 

//     //everything will call this function to move the motor. seperated from the business logic this allows us to change implementation without touching the login of the subsystem.
//     public void MoveMotor(double speed)
//     {
//         SmartDashboard.putNumber("Extendo Speed", speed);//data collection of current requested "speed" which is currently duty cycle.
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