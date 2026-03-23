package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.FuelIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonIntakeOnCommandLong extends Command {
    FuelIntakeSubsystem intakeSubsystem;
    Timer timer;
    boolean isItFinished;

    public AutonIntakeOnCommandLong(FuelIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        timer = new Timer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        isItFinished = false;
        timer.reset();
        timer.restart();
        
    }

    @Override
    public void execute() {
        intakeSubsystem.FuelIntakeOn(RobotConstants.FuelIntakeOnspeedAuto);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= 10.0) {
            intakeSubsystem.FuelIntakeOff();
            return true;
        }
        return false;
    }
}