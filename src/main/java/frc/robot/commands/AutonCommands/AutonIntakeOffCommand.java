package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelIntakeSubsystem;

public class AutonIntakeOffCommand extends Command {
    FuelIntakeSubsystem intakeSubsystem;

    public AutonIntakeOffCommand(FuelIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.FuelIntakeOff();
    }
}
