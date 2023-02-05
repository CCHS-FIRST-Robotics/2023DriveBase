package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Controller extends XboxController{
    
    public Controller() {
        super(Constants.XBOX_CONTROLLER_PORT);
    }

    // TODO: maybe implement it in a way that each joystick/direction has its own exponent
    private double applyPreferences(double input, double exponent) {
        if (Math.abs(input) < Constants.ANALOG_DEAD_ZONE) {
            input = 0;
        }
        input = Math.pow(Math.abs(input), exponent) * Math.signum(input);
        return input;
    }

    
    @Override
    public double getLeftX() {
        return applyPreferences(super.getLeftX(), Constants.LEFT_X_EXPONENT);
    }

    //flip input for Y-axis because up is negative natively
    @Override
    public double getLeftY() {
        return applyPreferences(-super.getLeftY(), Constants.LEFT_Y_EXPONENT);
    }

    @Override
    public double getRightX() {
        return applyPreferences(super.getRightX(), Constants.RIGHT_X_EXPONENT);
    }

    //flip input for Y-axis because up is negative natively
    @Override
    public double getRightY() {
        return applyPreferences(-super.getRightY(), Constants.RIGHT_Y_EXPONENT);
    }
}
