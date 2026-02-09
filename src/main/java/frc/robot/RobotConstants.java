package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.DuplicateFormatFlagsException;

public class RobotConstants {
  // AprilTag Constants
    public final static double TowerOffset = 0.32; //Distance robot is to be away from Tower
    public final static double TowerOffsetRight = 0.432; //Distance between AprilTags
    public final static double HubOffset = - 1.0; //Distance robot is to be away from Hub
    public final static double HubOffsetLeft = 0.356; //Distance between AprilTags on Hub

    // Fuel Shooter Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motors Spinning
    public final static int FuelShooterMotorCANid = 20;
    public final static int FuelShooterMotor2CANid = 21;
    public final static int FuelShooterMotor3CANid = 22;
    public final static int FuelShooterEncoderid = 2;
    //shooter power values
    public final static double FuelShooterMaxVelocity = -1600;
    public final static double FuelShooterSpeed = -0.50;

    // Fuel Wrist Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor moving the
    // Fuel Intake in/out of robot
    public final static int FuelIntakeWristMotorCANid = 23;
    public final static int FuelIntakeWristMotorCANidJOE = 33;
    public final static int FuelIntakeWristEncoderDIOid = 1;
    // Fuel Wrist PID Values for Motors
    public final static double FuelWristmotorP = 0;
    public final static double FuelWristmotorI = 0;
    public final static double FuelWristmotorD = 0;
    public final static double FuelWristmotorFF = 0;

    // Fuel Wrist Values for encoders - Offsets used to make end values between 0
    // and 1
    public final static double FuelWristencoderOffset = 0.00;
    public final static double FuelWristrangeOffset = 0.03;
    public final static double FuelWristExtendgoal = 0.933;
    public final static double FuelWristRetractgoal = 0.437;
 
    // Fuel Wrist Power values for motors
    public final static double FuelWristExtendpower = -0.40;
    public final static double FuelWristRetractpower = +0.80;

    // Fuel Intake Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor Spinning
    // Wheels on Fuel Intake
    public final static int FuelIntakeCANid = 57;

    // Fuel Intake PID Values for Motors
    public final static double FuelIntakemotorP = 0;
    public final static double FuelIntakemotorI = 0;
    public final static double FuelIntakemotorD = 0;
    public final static double FuelIntakemotorFF = 0;

    // Fuel Intake Power values for motors
    public final static double FuelIntakeOnspeed = -0.90;
    // public final static double FuelIntakeOutspeed = 0.50;
    // public final static double FuelIntakeSlowspeed = -0.2;

    // Conveyor Intake Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor Spinning
    // Wheels on Fuel Intake
    public final static int ConveyorIntakeCANid = 24;
    public final static int ConveyorIntakeSensorDIOid = 7;

    // Fuel Intake PID Values for Motors
    public final static double ConveyorIntakemotorP = 0;
    public final static double ConveyorIntakemotorI = 0;
    public final static double ConveyorIntakemotorD = 0;
    public final static double ConveyorIntakemotorFF = 0;

    // Fuel Intake Power values for motors
    public final static double ConveyorIntakeOnspeed = -1.00;
    public final static double ConveyorIntakeOnspeedAuton = -1.00;
    public final static double ConveyorIntakeOutspeed = 0.80;
    public final static double ConveyorIntakeSlowspeed = 0.60;

        // Climber Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor moving the
    // Climber in/out of robot
    public final static int ClimbermotorUPPERcanID = 25;
    public final static int ClimbermotorLOWERcanID = 26;
    public final static double ClimberencoderLOWERoffset = 0.2;
    public final static double ClimberencoderUPPERoffset = 0.2;
    

    // Climber PID Values for Motors
    public final static double ClimbermotorP = 0;
    public final static double ClimbermotorI = 0;
    public final static double ClimbermotorD = 0;
    public final static double ClimbermotorFF = 0;

    // Climber Values for encoders - Offsets used to make end values between 0
    // and 1
    public final static double ClimberencoderOffset = 0.00;
    public final static double ClimberrangeOffset = 0.03;
    public final static double LOWERClimberGoal = 0.400;
    public final static double LOWERClimberHome = 0.000;
    public final static double UPPERClimberGoal = 0.400;
    public final static double UPPERClimberHome = 0.000;

    // Climber Power values for motors
    public final static double ClimberExtendpower = 0.40;
    public final static double ClimberRetractpower = -0.40;
    public final static double ClimberClimbpower = 0.50;
    public final static double ClimberReleasepower = -0.50;
    // Serealizer
    public final static int SerializerMotorCAN = 27;
    public final static double SerializerOnspeed = -0.7;
    public final static double SerializerOutspeed = +0.5;
    public final static double SerializerSlowspeed = 0.2;

    // LED Values
    public final static double LEDintakesensor = 0.81;
    public final static double LEDintakeReady = 0.75;
    public final static double LEDdefault = 0.57;

    public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
    public static final Distance LIMELIGHT_LENS_HEIGHT = Distance.ofBaseUnits(8, Inches);
    public static final Angle LIMELIGHT_ANGLE = Angle.ofBaseUnits(0, Degrees);

    public static final Distance HUB_APRILTAG_HEIGHT = Distance.ofBaseUnits(44.25, Inches);
    public static final Distance TOWER_APRILTAG_HEIGHT = Distance.ofBaseUnits(21.75, Inches);
    public static final Distance OUTPOST_APRILTAG_HEIGHT = Distance.ofBaseUnits(21.75, Inches);
  }
}
