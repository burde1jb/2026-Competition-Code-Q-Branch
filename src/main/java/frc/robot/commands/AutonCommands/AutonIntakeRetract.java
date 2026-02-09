package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.FuelIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonIntakeRetract extends Command {
    FuelIntakeSubsystem intakeSubsystem;
    boolean isItFinished;
    boolean intakeFinished;
    Timer timer;

    public AutonIntakeRetract(FuelIntakeSubsystem intakeSubsystem) {
        timer = new Timer();
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        isItFinished = false;
        intakeFinished = false;
        timer.restart();

    }

    @Override
    public void execute() {
        intakeSubsystem.wentTo(RobotConstants.FuelWristRetractgoal);
    }

    @Override
    public boolean isFinished() {
        if (intakeSubsystem.wentTo(RobotConstants.FuelWristRetractgoal) || timer.get() >= 20.0) {
            intakeSubsystem.wristOff();
            return true;
        }
        return false;
    }
}