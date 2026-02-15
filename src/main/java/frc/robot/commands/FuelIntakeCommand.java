package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.*;

public class FuelIntakeCommand extends Command {
    FuelIntakeSubsystem intakeSubsystem;
    XboxController controller2;

    public FuelIntakeCommand(
            FuelIntakeSubsystem intakeSubsystem,
            XboxController controller2) {
        this.intakeSubsystem = intakeSubsystem;
        this.controller2 = controller2;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (controller2.getLeftTriggerAxis() > 0.2) {
            intakeSubsystem.FuelIntakeOn(true);
        }
        else {
            intakeSubsystem.FuelIntakeOff();
        }

        if (controller2.getAButton()) {
            intakeSubsystem.goTo(RobotConstants.FuelWristExtendgoal);
            // intakeSubsystem.wristOn(true);
        } else if (controller2.getBButton()) {
            intakeSubsystem.goTo(RobotConstants.FuelWristRetractgoal);
            // intakeSubsystem.wristOn(false);
        } else {
            intakeSubsystem.wristOff();
        }
    }
}