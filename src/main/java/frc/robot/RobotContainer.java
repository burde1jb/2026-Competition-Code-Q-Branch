// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.units.*;
import frc.robot.subsystems.FuelShooterSubsystem;
import frc.robot.commands.AimAndDriveCommand;
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


public class RobotContainer {
  // kSpeedAt12Volts desired top speed
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(3.0).in(RadiansPerSecond);
  private final CommandJoystick joystick = new CommandJoystick(0);
  private final CommandXboxController xboxController = new CommandXboxController(1);
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  // /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
       .withDeadband(MaxSpeed * 0.1)
       .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
       .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  //private final Telemetry logger = new Telemetry(MaxSpeed);
  private final FuelIntakeSubsystem intakeSubsystem;
  

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
    RegisterPathplannerCommands();
 
    configureTriggers();
    configureBindings();
    configureSecondControllerBindings();
    configureAutoChooser();
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand()); // Replaces deprecated method -> FollowPathCommand.warmupCommand().schedule();    
        
  }
  
  public double RPMTolerance = 200; // 4% tolerance, this is pretty tight but it is important to be at the correct speed for shooting and passing.
  public Trigger ShooterRPMOK; // we could use shooter.atspeed() but meh. 
  
  private void configureTriggers() {
    ShooterRPMOK = new Trigger(() -> MathUtil.isNear(shooterSubsystem.MaxVelocity, shooterSubsystem.FuelShooterVelocity, RPMTolerance));

    //anytime the shooter is up to speed, run the serializer and conveyor to feed fuel into the shooter. 
    //the shooter is only up to speed when the trigger is pulled, wether for passing or for shooting, so this should not cause any issues with intaking.
    ShooterRPMOK
      .onTrue(Commands.parallel(TeleopSerializerOn(),TeleopConveyorOn()))
      .onFalse(Commands.parallel(TeleopConveyorOff(),TeleopSerializerOff())
    );
  }
    
  private void configureBindings() {
    //Note that X is defined as forward according to WPILib convention,
    //and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        // Drive forward with negative Y (forward)
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getRawAxis(4) * MaxSpeed)
            // Drive left with negative X (left)
            .withVelocityY(-joystick.getRawAxis(3) * MaxSpeed)
            // Drive counterclockwise with negative X (left)
            .withRotationalRate(-joystick.getRawAxis(0) * MaxAngularRate)));

    intakeSubsystem.setDefaultCommand(new FuelIntakeCommand(intakeSubsystem, xboxController.getHID()));
    //shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem, xboxController.getHID()));
    //serializerSubsystem.setDefaultCommand(new SerializerCommand(serializerSubsystem, xboxController.getHID()));
    //conveyorSubsystem.setDefaultCommand(new ConveyorCommand(conveyorSubsystem, xboxController.getHID()));
    ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem, shooterSubsystem));
    //visionSubsystem.setDefaultCommand(new AlignCommand(drivetrain, visionSubsystem,6));
    climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem, xboxController.getHID()));

    
    
    /*
     * Move to firing position and fire
     */
    joystick.button(15)
      .whileTrue(
        Commands.parallel(
            new AimAndDriveCommand(drivetrain, () -> joystick.getRawAxis(4) * MaxSpeed, () -> -joystick.getRawAxis(3) * MaxSpeed),
            TeleopShooterOn()
        )
      )
      .onFalse(TeleopShooterOff());

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
    joystick.button(14).whileTrue(xstance());
  
    // // reset the field-centric heading on left bumper press
    joystick.button(13).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    // commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureSecondControllerBindings() {
    /*
     * Shooter Bindings
     */
    xboxController.y().onTrue(TeleopShooterOn());
    xboxController.y().onFalse(TeleopShooterOff());
    /*
     * Serializer Bindings
     */
    xboxController.a().onTrue(TeleopSerializerOn());
    xboxController.rightTrigger(.2).onTrue(new InstantCommand(()->{serializerSubsystem.serializerOn(false);}, serializerSubsystem)).onFalse(new InstantCommand(()->{serializerSubsystem.serializerOff();}, serializerSubsystem));
    xboxController.a().onFalse(new InstantCommand(()->{serializerSubsystem.serializerOff();}, serializerSubsystem));
    /*
     * conveyor Bindings
     */
    xboxController.leftBumper()
    .onTrue(TeleopConveyorOn())
    .onFalse(TeleopConveyorOff());

    xboxController.back()
    .onTrue(new InstantCommand(() -> {conveyorSubsystem.conveyorOn(-RobotConstants.ConveyorIntakeOnspeed);}, conveyorSubsystem))
    .onFalse(new InstantCommand(() -> {conveyorSubsystem.conveyorOff();}, conveyorSubsystem));
  }
      
  private void RegisterPathplannerCommands() {
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
  }
  private void configureAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
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
  public Command TeleopShooterOn() {
    return new InstantCommand(() -> {
      shooterSubsystem.shooterOn(RobotConstants.FuelShooterMaxVelocity);},shooterSubsystem);
  }
  public Command TeleopShooterOff() {
    return AutonShooterOff();
  }
  public Command TeleopSerializerOn() {
    return new InstantCommand(() -> {
      serializerSubsystem.serializerOn(true);
    }, serializerSubsystem);
  }
  public Command TeleopSerializerOff() {
    return AutonSerializerOff();
  }
  public Command TeleopConveyorOn() {
    return AutonConveyorOn();
  }
  public Command TeleopConveyorOff() {
    return AutonConveyorOff();
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
