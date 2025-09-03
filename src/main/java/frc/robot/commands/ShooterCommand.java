package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double shooterRPM;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double shooterRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterRPM = shooterRPM;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterMotorRPM(shooterRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterMotorRPM(0);
    }
}
