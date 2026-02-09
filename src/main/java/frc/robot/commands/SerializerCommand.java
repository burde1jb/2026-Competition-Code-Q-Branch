package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class SerializerCommand extends Command {
    SerializerSubsystem SerializerSubsystem;
    XboxController controller2;

    public SerializerCommand(SerializerSubsystem SerializerSubsystem, XboxController controller2) {
        this.SerializerSubsystem = SerializerSubsystem;
        this.controller2 = controller2;
        addRequirements(SerializerSubsystem);
    }

    @Override
    public void execute() {
        if (controller2.getRightBumperButton()){
            SerializerSubsystem.serializerOn(true);
        }
        else if (controller2.getStartButton()) {
            SerializerSubsystem.serializerOn(false);
        }
        else {
            SerializerSubsystem.serializerOff();
        }
    }
}