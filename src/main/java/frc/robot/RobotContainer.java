// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import java.util.function.IntSupplier;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.units.*;
import frc.robot.subsystems.FuelShooterSubsystem;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.FuelIntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.SerializerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.AutonCommands.*;
import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.subsystems.FuelIntakeWristSubsystemJOE;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.SerializerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.AprilTagManager;
// import frc.robot.subsystems.ExtendoSubsystemDRY;
// import frc.robot.subsystems.ExtendoSubsystemPID1;
// import frc.robot.subsystems.ExtendoSubsystemPIDRevlib;

public class RobotContainer {
  // kSpeedAt12Volts desired top speed
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(3.0).in(RadiansPerSecond);
  private final CommandJoystick joystick = new CommandJoystick(0);
  private final XboxController xboxController = new XboxController(1);
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  // /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
       .withDeadband(MaxSpeed * 0.1)
       .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
       .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final FuelIntakeSubsystem intakeSubsystem;
  
  //CHOOSE ONE
      //private final ExtendoSubsystem extendoSubsystem = new ExtendoSubsystem();;
      //private final ExtendoSubsystemDRY extendoSubsystem = new ExtendoSubsystemDRY();
      // private final ExtendoSubsystemPIDRevlib extendoSubsystem = new ExtendoSubsystemPIDRevlib();

  private final LEDSubsystem ledSubsystem;
  private final FuelShooterSubsystem shooterSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final SerializerSubsystem serializerSubsystem;
  private SendableChooser<Command> autoChooser;
  private final ClimberSubsystem climberSubsystem;
  // private final FuelIntakeWristSubsystemJOE intakeWristSubsystem;
  private final AprilTagManager ATMan;

  public RobotContainer() {
    
    this.intakeSubsystem = new FuelIntakeSubsystem();
    this.visionSubsystem = new VisionSubsystem();
    // this.extendoSubsystem = new ExtendoSubsystem();
    this.ledSubsystem = new LEDSubsystem();
    this.shooterSubsystem = new FuelShooterSubsystem();
    this.conveyorSubsystem = new ConveyorSubsystem();
    this.serializerSubsystem = new SerializerSubsystem();
    this.climberSubsystem = new ClimberSubsystem();
    ATMan = new AprilTagManager(drivetrain); 

    //Note that X is defined as forward according to WPILib convention,
    //and Y is defined as to the left according to WPILib convention.

    NamedCommands.registerCommand("AutonIntakeExtend", new AutonIntakeExtend(intakeSubsystem));
    NamedCommands.registerCommand("AutonIntakeRetract", new AutonIntakeRetract(intakeSubsystem));
    NamedCommands.registerCommand("AutonConveyorOnTimed", new AutonConveyorOnTimed(conveyorSubsystem));
    NamedCommands.registerCommand("AutonIntakeOnCommand", new AutonIntakeOnCommand(intakeSubsystem));
    NamedCommands.registerCommand("AutonIntakeOffCommand", new AutonIntakeOffCommand(intakeSubsystem));
    NamedCommands.registerCommand("AutonConveyorOn", AutonConveyorOn());
    NamedCommands.registerCommand("AutonConveyorOff", AutonConveyorOff());
    NamedCommands.registerCommand("AutonSerializerOnTimed", new AutonSerializerOnTimed(serializerSubsystem));
    NamedCommands.registerCommand("AutonSerializerOff", AutonSerializerOff());
    NamedCommands.registerCommand("AutonShooterOff", AutonShooterOff());
    NamedCommands.registerCommand("AutonIntakeOn", AutonIntakeOn());
    NamedCommands.registerCommand("AutonIntakeOff", AutonIntakeOff());
    NamedCommands.registerCommand("AutonShooterOn",AutonShooterOn());
    NamedCommands.registerCommand("AutonShooterOnTimed", new frc.robot.commands.AutonCommands.AutonShooterOnTimed(shooterSubsystem));
    NamedCommands.registerCommand("AutonClimber", new frc.robot.commands.AutonCommands.AutonClimber(climberSubsystem));
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        // Drive forward with negative Y (forward)
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getRawAxis(4) * MaxSpeed)
            // Drive left with negative X (left)
            .withVelocityY(-joystick.getRawAxis(3) * MaxSpeed)
            // Drive counterclockwise with negative X (left)
            .withRotationalRate(-joystick.getRawAxis(0) * MaxAngularRate)));

    intakeSubsystem.setDefaultCommand(new FuelIntakeCommand(intakeSubsystem, xboxController));
    shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem, xboxController));
    serializerSubsystem.setDefaultCommand(new SerializerCommand(serializerSubsystem, xboxController));
    conveyorSubsystem.setDefaultCommand(new ConveyorCommand(conveyorSubsystem, xboxController));
    ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem, shooterSubsystem));
    visionSubsystem.setDefaultCommand(new AlignCommand(drivetrain, visionSubsystem,6));
    climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem, xboxController));

    configureBindings();
    configureAutoChooser();
    // autoChooser = AutoBuilder.buildAutoChooser("test.");
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    FollowPathCommand.warmupCommand().schedule();
    
  }

  private void configureAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

    //  private final IntSupplier OptionalButtonSupplier = ()-> {
    //     if(joystick.button(12).getAsBoolean())//button 12 is the bottom right 3 position switch. 
    //     {
    //         return 2;//2 is the right side option on the reef
    //     }
    //         //default option is that x is not pressed and we score left side.
    //     else return 0;//0 is the left side option on reef
        
    // };

  public Command alignHub()
  {
      return new ConditionalCommand(ATMan.C_HubCommand(),ATMan.C_HubCommand(),()->{return true;}).asProxy();//.until(MantaState.getLimeLightBypassed)
      // return new ConditionalCommand( ATMan.C_HubCommand(), ATMan.C_TowerCommand(),()->{return OptionalButtonSupplier.getAsInt() == 0;}).asProxy();//.until(MantaState.getLimeLightBypassed)
   }
  public Command alignTower()
  {
    return new ConditionalCommand(ATMan.C_TowerCommand(), ATMan.C_TowerCommand(), ()->{return true;}).asProxy();//.until(MantaState.getLimeLightBypassed)
  }
  public Command xstance() {
    return new InstantCommand(() -> {
      drivetrain.brake(true);
    });
  }

  double JoystickAxisDeadZone = 0.2;//REMOVING MAGIC NUMBER AND GIVING IT A NAME
  int LeftXAxis = 0;
  int RightXAxis = 0; //REMOVING MAGIC NUMBER AND GIVING IT A NAME
  
  private void ConfigExtendoPID()
  {
    //Extendo subsystem PID version
      // CommandXbox.a().onTrue(extendoSubsystem.goTo(RobotConstants.ExtendoRetract));
      // CommandXbox.x().onTrue(
      // //   extendoSubsystem.goTo(.2)
      // // .andThen(extendoSubsystem.goTo(.6))
      // // .andThen(extendoSubsystem.goTo(.35))
      //  );
      // CommandXbox.y().onTrue(extendoSubsystem.goTo(10));
      // CommandXbox.b().onTrue(extendoSubsystem.goTo(RobotConstants.ExtendoExtend));
      //CommandXbox.axisMagnitudeGreaterThan(LeftXAxis, JoystickAxisDeadZone).whileTrue((new InstantCommand(()->{extendoSubsystem.Move(CommandXbox.getRawAxis(LeftXAxis));}).repeatedly()));
      // CommandXbox.start().onTrue(extendoSubsystem.goTo(RobotConstants.ExtendoBarge));
      //no stop command here becuase pid goes to a location and never "stops" it just only uses the force it needs (some exceptions exist for park commands or locking positions)
  }
  private void ConfigExtendo()
  {
      //THE ORIGINAL WAY THE BUTTONS ARE CHECKED EACH LOOP (NOT USED THIS VERSION)
      //extendoSubsystem.setDefaultCommand(new ElevatorCommand(extendoSubsystem, xboxController));
      //OR
      //Extendo subsystem and DRY version.
      // CommandXbox.a().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoRetract);}).repeatedly());
      // CommandXbox.x().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoExtend);}).repeatedly());
      // CommandXbox.y().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoExtendL4);}).repeatedly());
      // CommandXbox.b().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoExtend);}).repeatedly());
      // //CommandXbox.axisMagnitudeGreaterThan(LeftXAxis, JoystickAxisDeadZone).whileTrue((new InstantCommand(()->{extendoSubsystem.Move(CommandXbox.getRawAxis(LeftXAxis));}).repeatedly()));
      // CommandXbox.axisGreaterThan(LeftXAxis, JoystickAxisDeadZone).whileTrue((new InstantCommand(()->{extendoSubsystem.Extend();}).repeatedly()));
      // CommandXbox.axisLessThan(LeftXAxis, -JoystickAxisDeadZone).whileTrue((new InstantCommand(()->{extendoSubsystem.Retract();}).repeatedly()));
      // CommandXbox.start().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoBarge);}).repeatedly());
      // CommandXbox.a().negate()
      //   .and(CommandXbox.b().negate())
      //   .and(CommandXbox.x().negate())
      //   .and(CommandXbox.y().negate())
      //   .and(CommandXbox.start().negate())
      //   .and(CommandXbox.axisMagnitudeGreaterThan(LeftXAxis, JoystickAxisDeadZone).negate())
      //   .onTrue(new InstantCommand(()->{extendoSubsystem.stop();}));
  }
  public void ConfigExtendoOriginal()
  {

    //this is a snippet found in Elevatorcommand.java
    // if (controller2.getAButton()) { //Home Value
    //         //elevatorSubsystem.lcgoToHome(RobotConstants.lcHomeValue);
    //         extendoSubsystem.goTo(RobotConstants.ExtendoRetract);
    //     } else if (controller2.getXButton()) { //L2 Value
    //         //elevatorSubsystem.lcgoTo(RobotConstants.lcL2Value);
    //         extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
    //     } else if (controller2.getYButton()) { //L4 Value
    //         //elevatorSubsystem.lcgoTo(RobotConstants.lcL4Value);
    //         extendoSubsystem.goToL4(RobotConstants.ExtendoExtendL4);
    //     } else if (controller2.getBButton()) { //L3 Value
    //         //elevatorSubsystem.lcgoTo(RobotConstants.lcL3Value);
    //         extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
    //     } else if (controller2.getLeftY() < -0.2) {
    //         //elevatorSubsystem.goUp();
    //     } else if (controller2.getLeftY() > 0.2) {
    //         //elevatorSubsystem.goDown();
    //     } else if (controller2.getLeftX() > 0.2) {
    //         extendoSubsystem.Extend();
    //     } else if (controller2.getLeftX() < -0.2) {
    //         extendoSubsystem.Retract();
    //     } else if (controller2.getStartButton()) { // Barge Algae Value
    //         //elevatorSubsystem.lcgoTo(RobotConstants.lcL4Value);
    //         extendoSubsystem.goTo(RobotConstants.ExtendoBarge);
    //     } else {
    //         //elevatorSubsystem.stop();
    //         extendoSubsystem.stop();
    //     }
  }

  private void configureBindings() {

    //ConfigExtendo();
    ConfigExtendoPID();   

    // joystick.button(13).whileTrue(commandSwerveDrivetrain.applyRequest(() -> brake));
    // joystick.button(14).whileTrue(commandSwerveDrivetrain.applyRequest(
    //     () -> point.withModuleDirection(new Rotation2d(-joystick.getRawAxis(3), -joystick.getRawAxis(4)))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.button(1).and(joystick.button(12)).whileTrue(commandSwerveDrivetrain.sysIdDynamic(Direction.kForward));
    // joystick.button(1).and(joystick.button(11)).whileTrue(commandSwerveDrivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.button(3).and(joystick.button(12)).whileTrue(commandSwerveDrivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.button(3).and(joystick.button(11)).whileTrue(commandSwerveDrivetrain.sysIdQuasistatic(Direction.kReverse));

    //joystick.button(1).whileTrue(new AlignCommand(commandSwerveDrivetrain, visionSubsystem));
    joystick.button(1).whileTrue(alignHub());
    joystick.button(4).whileTrue(alignTower());
    joystick.button(14).whileTrue(brake(true));
        
        // // reset the field-centric heading on left bumper press
        joystick.button(13).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
        // commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
      }
      
      private Command brake(boolean b) {
        return new InstantCommand(() -> {
      drivetrain.brake(b);
        });
      }
    
      public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public Command AutonShooterOn() {
    return new InstantCommand(() -> {
      shooterSubsystem.shooterOn(1200);
    });
  }
  public Command AutonIntakeOn() {
     return new InstantCommand(() -> {
       intakeSubsystem.FuelIntakeOn(RobotConstants.FuelIntakeMaxVelocity);
     });
   }

  public Command AutonIntakeOff() {
    return new InstantCommand(() -> {
      intakeSubsystem.FuelIntakeOff();
    });
  }

  public Command AutonConveyorOn() {
    return new InstantCommand(() -> {
      conveyorSubsystem.conveyorOn(RobotConstants.ConveyorMaxVelocity);
    });
  }

  public Command AutonConveyorOff() {
    return new InstantCommand(() -> {
      conveyorSubsystem.conveyorOff();
    });
  } 

  public Command AutonSerializerOn() {
    return new InstantCommand(() -> {
      serializerSubsystem.serializerOn(true);
    });
  }

  public Command AutonSerializerOff() {
    return new InstantCommand(() -> {
      serializerSubsystem.serializerOff();
    });
  }
  
  public Command AutonShooterOff() {
    return new InstantCommand(() -> {
      shooterSubsystem.stop();
    });
  } 
}
