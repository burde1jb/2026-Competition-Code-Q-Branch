package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.FuelShooterSubsystem;


public class AutonShooterOnTimed extends Command {
    Timer timer;
    FuelShooterSubsystem shooterSubsystem;
    boolean isItFinished;


    public AutonShooterOnTimed(FuelShooterSubsystem shooterSubsystem) {
        timer = new Timer();
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // isItFinished = false;
        timer.restart();

    }

    @Override
    public void execute() {
        shooterSubsystem.shooterOn(RobotConstants.FuelShooterMaxVelocity);
        
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >=5.0) {
            shooterSubsystem.stop();
            return true;
        }
        return false;
        // return isItFinished;
    }
}