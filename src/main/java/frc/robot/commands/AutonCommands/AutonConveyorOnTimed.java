package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonConveyorOnTimed extends Command {
    ConveyorSubsystem conveyorSubsystem;
    Timer timer;
    boolean isItFinished;

    public AutonConveyorOnTimed(ConveyorSubsystem conveyorSubsystem) {
        this.conveyorSubsystem = conveyorSubsystem;
        timer = new Timer();
        addRequirements(conveyorSubsystem);
    }

    @Override
    public void initialize() {
        isItFinished = false;
        // timer.reset();
        timer.restart();
    }

    @Override
    public void execute() {
        conveyorSubsystem.conveyorOn(RobotConstants.ConveyorMaxVelocity);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= RobotConstants.AutonShootTime) {
            conveyorSubsystem.conveyorOff();
            return true;
        }
        return false;
    }
}