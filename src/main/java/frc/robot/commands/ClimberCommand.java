package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.*;

public class ClimberCommand extends Command {
    ClimberSubsystem climberSubsystem;
    XboxController controller2;

    public ClimberCommand(ClimberSubsystem climberSubsystem, XboxController controller2) {
        this.climberSubsystem = climberSubsystem;
        this.controller2 = controller2;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        if (controller2.getLeftY() > 0.2){
            climberSubsystem.UppergoTo(RobotConstants.UPPERClimberHome);
        }
        else if (controller2.getLeftY() < -0.2){
            climberSubsystem.UppergoTo(RobotConstants.UPPERClimberGoal);
        }
        // else if (controller2.getRightY() > 0.2){
        //     climberSubsystem.LowergoTo(RobotConstants.UPPERClimberGoal);
        // }
        // else if (controller2.getRightY() < -0.2){
        //     climberSubsystem.LowergoTo(RobotConstants.UPPERClimberHome);
        // }
        else{
           climberSubsystem.stop(); 
        }
    }
}