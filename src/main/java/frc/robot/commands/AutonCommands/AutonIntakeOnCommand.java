package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonIntakeOnCommand extends Command {
    FuelIntakeSubsystem intakeSubsystem;
    Timer timer;

    public AutonIntakeOnCommand(FuelIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        timer = new Timer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        intakeSubsystem.FuelIntakeOn(true);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= 5.0) {
            intakeSubsystem.FuelIntakeOff();
            return true;
        }
        return false;
    }
}