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
        intakeSubsystem.wentTo(RobotConstants.FuelWristExtendgoal);
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
    // @Override
    // public void end(boolean interrupted)
    // {
    //     intakeSubsystem.wristOff();
    // }
    @Override
    public boolean isFinished() {
        if (timer.get() >=2.0){
            intakeSubsystem.FuelIntakeOff();
            intakeSubsystem.wristOff();
            return true;
        }
        else if (intakeSubsystem.wentTo(RobotConstants.FuelWristExtendgoal) || timer.get() >= 1.0){
                intakeSubsystem.wristOff();
        }
        return false;
        // if (intakeSubsystem.wentTo(RobotConstants.FuelWristExtendgoal) || timer.get() >= 3.0) {
        //     intakeSubsystem.wristOff();
        //     intakeSubsystem.FuelIntakeOff();
            
        //     return true;
        // }
        // return false;
    }
}