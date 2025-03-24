package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Setpoints;

public class MoveToSetpointCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final double armSetpoint;
    private final double elevatorSetpoint;

    public MoveToSetpointCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, double armSetpoint, double elevatorSetpoint) {
        this.armSubsystem = armSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSetpoint = armSetpoint;
        this.elevatorSetpoint = elevatorSetpoint;
        addRequirements(armSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setMotorSpeed(armSetpoint);
        elevatorSubsystem.setMotorSpeed(elevatorSetpoint);
    }

    @Override
    public void execute() {
        // Add logic to move to setpoints using encoders
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMotor();
        elevatorSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        // Add logic to determine if the setpoints are reached
        return false;
    }
}

