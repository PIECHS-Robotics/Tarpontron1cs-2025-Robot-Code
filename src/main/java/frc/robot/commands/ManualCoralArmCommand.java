package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ArmSubsystem;

public class ManualCoralArmCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier joystickInput;

    public ManualCoralArmCommand(ArmSubsystem subsystem, DoubleSupplier joystickInput) {
        this.armSubsystem = subsystem;
        this.joystickInput = joystickInput;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double input = joystickInput.getAsDouble();
        if (Math.abs(input) > 0.1) {
            armSubsystem.setMotorSpeed(input * 0.3); //Scaled and deadband
        } else {
            armSubsystem.stopMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
