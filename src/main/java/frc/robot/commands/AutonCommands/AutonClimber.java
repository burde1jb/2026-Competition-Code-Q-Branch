package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonClimber extends Command {
    ClimberSubsystem climberSubsystem;
    boolean isItFinished;
    boolean climberFinished;
    Timer timer;

    public AutonClimber(ClimberSubsystem climberSubsystem) {
        timer = new Timer();
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        isItFinished = false;
        climberFinished = false;
        timer.restart();

    }

    @Override
    public void execute() {
        climberSubsystem.UPPERwentTo(RobotConstants.UPPERClimberGoal);
        if (!climberFinished && climberSubsystem.UPPERwentTo(RobotConstants.UPPERClimberGoal) || timer.get() > 3.0) {
            climberSubsystem.stop();
            climberFinished = true;
        }
        if (climberFinished) {
            isItFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= 3.0) {
            climberSubsystem.stop();
            return true;
        }
        return false;
    }
}