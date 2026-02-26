package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.*;

public class ShooterCommand extends Command {
    FuelShooterSubsystem ShooterSubsystem;
    XboxController controller2;

    public ShooterCommand(FuelShooterSubsystem ShooterSubsystem, XboxController controller2) {
        this.ShooterSubsystem = ShooterSubsystem;
        this.controller2 = controller2;
        addRequirements(ShooterSubsystem);
    }

    @Override
    public void execute() {
        // if(controller2.getXButton()){
        //     ShooterSubsystem.shooterBangBang(-0.30);
        //     }
        //     else {
        //         ShooterSubsystem.shooterBangBang(0);
        //     }
        if(controller2.getYButton()){
            ShooterSubsystem.shooterOn(RobotConstants.FuelShooterMaxVelocity);
        }
        else if (controller2.getRightTriggerAxis() > 0.2) {
            // ShooterSubsystem.runFlywheelCommand();
            ShooterSubsystem.shooterSpeed(-0.10);
        }
        else {
            ShooterSubsystem.stop();
        }
        
       
    }
}