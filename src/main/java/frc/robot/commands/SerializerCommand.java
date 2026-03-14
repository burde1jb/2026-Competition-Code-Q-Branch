package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AlphaBots.NT;
import frc.robot.subsystems.*;

public class SerializerCommand extends Command {
    SerializerSubsystem SerializerSubsystem;
    XboxController controller2;
     public Trigger RPMOK;// = new Trigger(()->shooterSubsystem.atSpeed());
    private final BooleanEntry RPMOKTOSerialize = NT.getBooleanEntry("SerializerCommand", "RPMOKTOSerialize", false);

    public SerializerCommand(SerializerSubsystem SerializerSubsystem, XboxController controller2,BooleanSupplier RpmOK) {
        this.SerializerSubsystem = SerializerSubsystem;
        this.controller2 = controller2;
        RPMOK = new Trigger(RpmOK);
        addRequirements(SerializerSubsystem);
    }

    @Override
    public void execute() {
        RPMOKTOSerialize.set(RPMOK.getAsBoolean());
        if (controller2.getRightBumperButton() ){ //& RPMOK.getAsBoolean()
            SerializerSubsystem.serializerOn(true);
        }
        else if (controller2.getRightTriggerAxis() > 0.2) {
            SerializerSubsystem.serializerOn(false);
        }
        else {
            SerializerSubsystem.serializerOff();
        }
    }
}