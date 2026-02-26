package frc.robot.commands;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.*;

public class ConveyorCommand extends Command {
    ConveyorSubsystem ConveyorSubsystem;
    XboxController controller2;

    public ConveyorCommand(ConveyorSubsystem ConveyorSubsystem, XboxController controller2) {
        this.ConveyorSubsystem = ConveyorSubsystem;
        this.controller2 = controller2;
        addRequirements(ConveyorSubsystem);
    }

    @Override
    public void execute() {
        if (controller2.getLeftBumperButton()){
            ConveyorSubsystem.conveyorOn(RobotConstants.ConveyorMaxVelocity);
        }
        else if (controller2.getBackButton()){
            ConveyorSubsystem.conveyorOn(-RobotConstants.ConveyorMaxVelocity);
        }
        else {
            ConveyorSubsystem.conveyorOff();
        }
    }
}