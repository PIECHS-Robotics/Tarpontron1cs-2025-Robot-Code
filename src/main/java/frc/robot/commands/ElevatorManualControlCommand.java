// ElevatorManualControlCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorManualControlCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final DoubleSupplier joystickInput;

    public ElevatorManualControlCommand(ElevatorSubsystem subsystem, DoubleSupplier input) {
        this.elevatorSubsystem = subsystem;
        this.joystickInput = input;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double speed = joystickInput.getAsDouble();
        if (Math.abs(speed) > 0.1) {
            elevatorSubsystem.setMotorSpeed(speed * 0.5);  // Scale speed
        } else {
            elevatorSubsystem.stopMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotor();
    }
}

