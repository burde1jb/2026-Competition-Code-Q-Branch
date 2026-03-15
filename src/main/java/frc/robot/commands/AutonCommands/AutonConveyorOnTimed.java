package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonConveyorOnTimed extends Command {
    ConveyorSubsystem conveyorSubsystem;
    Timer timer;

    public AutonConveyorOnTimed(ConveyorSubsystem conveyorSubsystem) {
        this.conveyorSubsystem = conveyorSubsystem;
        timer = new Timer();
        addRequirements(conveyorSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        conveyorSubsystem.conveyorOn(RobotConstants.ConveyorMaxVelocity);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= 5.0) {
            conveyorSubsystem.conveyorOff();
            return true;
        }
        return false;
    }
}