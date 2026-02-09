package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.RobotConstants;

public class LEDCommand extends Command {
    LEDSubsystem ledSubsystem;
    FuelShooterSubsystem shooterSubsystem;

    public LEDCommand(LEDSubsystem ledSubsystem, FuelShooterSubsystem shooterSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        if (shooterSubsystem.atSpeed()) {
            ledSubsystem.set(RobotConstants.LEDintakesensor);
        }
        else {
            ledSubsystem.set(RobotConstants.LEDdefault);
        }
    }
}