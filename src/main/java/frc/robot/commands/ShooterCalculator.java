package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.position_joint.PositionJoint;


public class ShooterCalculator extends Command{
    
    private final PositionJoint subsystem;
    private final DoubleSupplier distanceSupplier;

    public ShooterCalculator(PositionJoint subsystem, DoubleSupplier distanceSupplier) {
        this.subsystem = subsystem;
        this.distanceSupplier = distanceSupplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double distance = distanceSupplier.getAsDouble();
        double hoodAngle = calculateHoodAngle(distance);
        subsystem.setPosition(hoodAngle);
    }

    private double calculateHoodAngle(double distance) {
        // placeholder equation
        return 5.67 * Math.pow(1.0613, distance); 
    }

     @Override
     public boolean isFinished() {
         return false;
     }
}
