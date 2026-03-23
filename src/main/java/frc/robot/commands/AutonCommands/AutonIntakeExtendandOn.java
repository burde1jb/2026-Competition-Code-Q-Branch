package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.FuelIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonIntakeExtendandOn extends Command {
    FuelIntakeSubsystem intakeSubsystem;
    boolean isItFinished;
    boolean intakeFinished;
    Timer timer;

    public AutonIntakeExtendandOn(FuelIntakeSubsystem intakeSubsystem) {
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
        intakeSubsystem.wristOn(true);
        intakeSubsystem.FuelIntakeOn(RobotConstants.FuelIntakeOnspeedAuto);
        // wristSubsystem.wentTo(RobotConstants.FuelWristExtendgoal);
        // if (wristSubsystem.wentTo(RobotConstants.FuelWristExtendgoal) || timer.get() > 15.0) {
        //     wristSubsystem.wristOff();
        //     wristFinished = true;
        // }
        // if (wristFinished) {
        //     isItFinished = true;
        // }
    }
    
    @Override
    public boolean isFinished() {
        if (intakeSubsystem.wentTo(RobotConstants.FuelWristExtendgoal) || timer.get() >= 1.5) {
            intakeSubsystem.wristOff();
            return true;
        }
        else if (intakeFinished && timer.get() >= 10.0) {
            intakeSubsystem.FuelIntakeOff();
            return true;
        }
        return false;
    }
}