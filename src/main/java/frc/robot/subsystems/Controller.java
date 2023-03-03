package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.LinearFilter;

public class Controller extends XboxController{
    
    LinearFilter leftXFilter, leftYFilter, rightXFilter, rightYFilter;

    /**
     * @param port
     * @param numSamples (int) - number of samples in the moving average (numSamples * .02 = time for moving average)
     */
    public Controller(int port, int numSamples) {
        super(port);
        leftXFilter = LinearFilter.movingAverage(numSamples);
        leftYFilter = LinearFilter.movingAverage(numSamples);
        rightXFilter = LinearFilter.movingAverage(numSamples);
        rightYFilter = LinearFilter.movingAverage(numSamples);
    }

    // TODO: maybe implement it in a way that each joystick/direction has its own exponent
    private double applyPreferences(double input, double exponent, LinearFilter filter) {
        if (Math.abs(input) < Constants.ANALOG_DEAD_ZONE) {
            input = 0;
        }
        input = Math.pow(Math.abs(input), exponent) * Math.signum(input);
        return filter.calculate(input);
    }
    
    @Override
    public double getLeftX() {
        return applyPreferences(super.getLeftX(), Constants.LEFT_X_EXPONENT, leftXFilter);
    }

    //flip input for Y-axis because up is negative natively
    @Override
    public double getLeftY() {
        return applyPreferences(-super.getLeftY(), Constants.LEFT_Y_EXPONENT, leftYFilter);
    }

    @Override
    public double getRightX() {
        return applyPreferences(super.getRightX(), Constants.RIGHT_X_EXPONENT, rightXFilter);
    }

    //flip input for Y-axis because up is negative natively
    @Override
    public double getRightY() {
        return applyPreferences(-super.getRightY(), Constants.RIGHT_Y_EXPONENT, rightYFilter);
    }
}
