package frc.robot.subsystems;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.encoder.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class FuelShooterSubsystem extends SubsystemBase {
    //name of this subsystem for dashboard labeling
    String className = this.getClass().getSimpleName();
    SparkFlex FuelShooterMotor;
    SparkFlex FuelShooterMotor2;
    SparkClosedLoopController FuelShooterMotorLoop;
    SparkClosedLoopController FuelShooterMotorLoop2;
    SparkFlexConfig FuelShooterMotorConfig;
    SparkFlexConfig FuelShooterMotorConfig2;
    //
    RelativeEncoder FuelShooterEncoder;
    RelativeEncoder FuelShooterEncoder2;
   
    //private final double encoderOffset = RobotConstants.ExtendoEncoderOffset;
    // private final double PositionalTolerance = 0.01;//2 * rangeOffset;
    // private SparkFlexConfig motorConfig = new SparkFlexConfig();

    private double PGain = 5.1;
    private double IGain = 0.0;
    private double DGain = 0.0;
    private double MaxAccel = 0.0;
    private double Velocity = 0.0;
    private double speed = 1200.0;
    // these positions are for the Soft limits. these are used by the motor controller to attempt to
    // control the movement range of the motor
    // These are often found by cold booting the mechanism to a known location (like a hard stop) and
    // setting that position as 0 (min position), then moving to the other end of travel and reading
    // the position (maxposition).
    private double MaxVelocity =  RobotConstants.FuelShooterMaxVelocity; // rotations
    private double FuelShooterVelocity;
    private double FuelShooterVelocity2;

      //the built in PID controller on the Spark(Max Or Flex) motor controller, these will not take processing power from the roborio because they are running on the motor controller itself at a much higher loop rate (this is good for fast response and precision)
    private SparkClosedLoopController SparkMaxBuiltInPidController;

    
    public FuelShooterSubsystem() {
        FuelShooterMotor = new SparkFlex(RobotConstants.FuelShooterMotorCANid, MotorType.kBrushless);
        FuelShooterMotor2 = new SparkFlex(RobotConstants.FuelShooterMotor2CANid, MotorType.kBrushless);
        FuelShooterEncoder = FuelShooterMotor.getEncoder();
        FuelShooterEncoder2 = FuelShooterMotor2.getEncoder();
        SparkMaxBuiltInPidController = FuelShooterMotor.getClosedLoopController();
        FuelShooterMotorLoop = FuelShooterMotor.getClosedLoopController();
        FuelShooterMotorLoop2 = FuelShooterMotor2.getClosedLoopController();
        FuelShooterMotorConfig = new SparkFlexConfig();
        FuelShooterMotorConfig2 = new SparkFlexConfig();
        FuelShooterVelocity = FuelShooterEncoder.getVelocity();
        FuelShooterVelocity2 = FuelShooterEncoder2.getVelocity();
        // SetupMotorConfig();
        FuelShooterMotorLoop.setSetpoint(-3000, ControlType.kMAXMotionVelocityControl);
        FuelShooterMotorLoop2.setSetpoint(-3000, ControlType.kMAXMotionVelocityControl);

    }

    // This will return the subsytems current position 
    // public double getEncoderMeasurement() {
    //     if(Robot.isSimulation())
    //     {
    //         return simposition;
    //     }

    //     double RawPosition = ExtendoEncoder.getPosition();
    //     double position =  RawPosition + rangeOffset; // + encoderOffset % 1; //why do we modulo 1 here? this will always return a remainder and not the correct position?
    //     SmartDashboard.putNumber("Extendoposition", position);
    //     SmartDashboard.putNumber("RawExtendoposition", RawPosition);
    //     return position; 
    // }

    // this returns a command that can be called when needed. it is part of this class but is not really "part"  of this subsystem. its a factory that builds a command for other parts of the program to use.  
    //https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#capturing-state-in-inline-commands
    
    //another option for the joystick control
    // public void MoveByJoystick(double MovePower) {
    //     if (MovePower > 0) {
    //         MoveMotor(getEncoderMeasurement()+(MovePower*RobotConstants.ExtendoExtendSpeed));
    //     }
    //     else {
    //         MoveMotor(getEncoderMeasurement()+(MovePower*RobotConstants.ExtendoRetractSpeed));
    //     }
    // } 

    //everything will call this function to move the motor. seperated from the business logic this allows us to change implementation without touching the login of the subsystem.
  
    //whichever way we choose to stop the motor goes here. this way we can change the entire class from brake to coast or to hold position without having to change any other business logic or codes. the entire implementation is here.
    public void stop() {
    //     //same thing but we are using .set(0);
        FuelShooterMotor.stopMotor();
        FuelShooterMotor2.stopMotor();
        // FuelShooterMotor3.stopMotor();
        speed = 0;
        FuelShooterMotor.set(0);
        FuelShooterMotor2.set(0);
        // FuelShooterMotor3.set(0);

     }
    public void shooterOn(double velocity){
        double v = RobotConstants.FuelShooterSpeed;
        double x = (velocity - FuelShooterEncoder.getVelocity())/velocity;
        FuelShooterMotor.set(v);
        FuelShooterMotor2.set(v);
        
        // // FuelShooterMotor3.set(v);
        // if (Math.abs(FuelShooterEncoder.getVelocity()) < Math.abs(0.94 * velocity)){
        //   FuelShooterMotor.set(v);
        //   FuelShooterMotor2.set(v);
        // }
        // if (Math.abs(FuelShooterEncoder.getVelocity()) > Math.abs((1.06 * velocity))){
        //   FuelShooterMotor.set(-0.02);
        //   FuelShooterMotor2.set(-0.02);
        // }
        // else if (Math.abs(x) < 0.06 ){
        //     if (FuelShooterEncoder.getVelocity() < velocity){
        //         FuelShooterMotor.set(-0.02);
        //         FuelShooterMotor2.set(-0.02);
        //     }
        //     else if (FuelShooterEncoder.getVelocity() > velocity){
        //         FuelShooterMotor.set(-0.02);
        //         FuelShooterMotor2.set(-0.02);
        //     }
        //     // FuelShooterMotor3.set(v);
        // }
        
        // else {
        // FuelShooterMotor.set(v);
        // FuelShooterMotor2.set(v);
        // }
    }

    public boolean atSpeed () {
      if (FuelShooterEncoder.getVelocity() <= RobotConstants.FuelShooterMaxVelocity * 0.98){
        return true;
      }
      else {
        return false;
      }
    }

    public void restart() {
      speed = 1200;
      }
    // public void velocitySetpoint(double velocitySetpoint) {
    //     //limit the setpoint to the max and min positions
    //     double limitedSetpoint = MathUtil.clamp(velocitySetpoint, -speed, speed);
    //     SparkMaxBuiltInPidController.setReference(limitedSetpoint, ControlType.kVelocity);
    // }
    @Override
    
    public void periodic() {
        SmartDashboard.putNumber("FuelShooter Encoder", FuelShooterEncoder.getVelocity());//why does this not include the offset?
        // SmartDashboard.putNumber("motorPosition",FuelShooterMotor.getEncoder().getPosition());
        // SmartDashboard.putNumber("FuelShooterTemp",FuelShooterMotor.getMotorTemperature());
        // SmartDashboard.putBoolean("FuelShooterISAtSetpoint",SparkMaxBuiltInPidController.isAtSetpoint());
        
        // pidtune();
    }
      // Configure the motor controller
  //   private void SetupMotorConfig() {
  //       /*
  //       * Create a new SPARK MAX configuration object. This will store the
  //       * configuration parameters for the SPARK MAX that we will set below.
  //       */
  //       //motorConfig = new SparkMaxConfig();
  //       /*
  //       * Configure soft limits. These limits will prevent the motor from moving
  //       * beyond the specified positions.
  //       */
  //       //if using absolute encoder, user min and max absolute position. 
  //      // motorConfig.softLimit.forwardSoftLimit(MaxPostition).reverseSoftLimit(MinPostiion);
  //      //motorConfig.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
  //       // Invert Motor Direction (if needed)
  //       motorConfig.inverted(false);
  //       /*
  //       * Configure the encoder. For this specific example, we are using the
  //       * integrated encoder of the NEO, and we don't need to configure it. If
  //       * needed, we can adjust values like the position or velocity conversion
  //       * factors.
  //       */
  //       motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
  //       /* Apply the configuration to the SPARK MAX.
  //       *
  //       * kResetSafeParameters is used to get the SPARK MAX to a known state. This
  //       * is useful in case the SPARK MAX is replaced.
  //       *
  //       * kPersistParameters is used to ensure the configuration is not lost when
  //       * the SPARK MAX loses power. This is useful for power cycles that may occur
  //       * mid-operation.
  //       */
  //       FuelShooterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //       FuelShooterMotor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //       // FuelShooterMotor3.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //   }
  //   public void setupTuningDashboardDefaults()
  //   {
  //   SmartDashboard.setDefaultNumber(className + " P Gain", PGain);
  //   SmartDashboard.setDefaultNumber(className + " I Gain", IGain);
  //   SmartDashboard.setDefaultNumber(className + " D Gain", DGain);
  //   SmartDashboard.setDefaultNumber(className + " Max Accel", MaxAccel);
  //   SmartDashboard.setDefaultNumber(className + " Max Velocity", Velocity);
  //   }
  //  public void pidtune() {
  //   // get the latest user written values from the dashboard for the PID values.
  //   // these are local and are created/destroyed each time this function is called
  //   double p = SmartDashboard.getNumber(className + " P Gain", 0);
  //   double i = SmartDashboard.getNumber(className + " I Gain", 0);
  //   double d = SmartDashboard.getNumber(className + " D Gain", 0.1);
  //   double A = SmartDashboard.getNumber(className + " Max Accel", 0);
  //   double V = SmartDashboard.getNumber(className + " Max Velocity", 0);
  //   // if the value has changed, update the local variable AND controller with the new value.
  //   if ((p != PGain)) {
  //     PGain = p;
  //     motorConfig.closedLoop.p(p, ClosedLoopSlot.kSlot0);
  //     FuelShooterMotor.configureAsync(motorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //   }
  //   if ((i != IGain)) {
  //     IGain = i;
  //     motorConfig.closedLoop.i(i, ClosedLoopSlot.kSlot0);
  //     FuelShooterMotor.configureAsync(motorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //   }
  //   if ((d != DGain)) {
  //     DGain = d;
  //     motorConfig.closedLoop.d(d, ClosedLoopSlot.kSlot0);
  //     FuelShooterMotor.configureAsync(motorConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  //   }
  //   if ((A != MaxAccel)) {
  //     MaxAccel = A;
  //     motorConfig.closedLoop.maxMotion.maxAcceleration(A, ClosedLoopSlot.kSlot0);
  //     FuelShooterMotor.configureAsync(motorConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  //   }
  //   if ((V != Velocity)) {
  //     Velocity = V;
  //     motorConfig.closedLoop.maxMotion.cruiseVelocity(V, ClosedLoopSlot.kSlot0);
  //     FuelShooterMotor.configureAsync(motorConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  


  //   }
  // }

}