package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Controller extends XboxController{
    
    public Controller() {
        super(Constants.XBOX_CONTROLLER_PORT);
    }

    // TODO: maybe implement it in a way that each joystick/direction has its own exponent
    private double applyPreferences(double input) {
        input = Math.pow(Math.abs(input), Constants.EXPONENT) * Math.signum(input);
        if (Math.abs(input) < Constants.ANALOG_DEAD_ZONE) {
            input = 0;
        }
        return input;
    }

    
    @Override
    public double getLeftX() {
        return applyPreferences(super.getLeftX());
    }

    //flip input for Y-axis because up is negative natively
    @Override
    public double getLeftY() {
        return applyPreferences(-super.getLeftY());
    }

    @Override
    public double getRightX() {
        return applyPreferences(super.getRightX());
    }

    //flip input for Y-axis because up is negative natively
    @Override
    public double getRightY() {
        return applyPreferences(-super.getRightY());
    }
}
