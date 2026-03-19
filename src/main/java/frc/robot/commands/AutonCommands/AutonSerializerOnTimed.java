package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SerializerSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonSerializerOnTimed extends Command {
    SerializerSubsystem serializerSubsystem;
    Timer timer;
    boolean isItFinished;

    public AutonSerializerOnTimed(SerializerSubsystem serializerSubsystem) {
        this.serializerSubsystem = serializerSubsystem;
        timer = new Timer();
        addRequirements(serializerSubsystem);
    }

    @Override
    public void initialize() {
        isItFinished = false;
        timer.reset();
        timer.restart();
    }

    @Override
    public void execute() {
        serializerSubsystem.serializerOn(true);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= 3.0) {
            serializerSubsystem.serializerOff();
            return true;
        }
        return false;
    }
}