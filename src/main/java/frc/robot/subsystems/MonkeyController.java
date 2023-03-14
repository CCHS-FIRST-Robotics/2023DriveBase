package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.LinearFilter;

public class MonkeyController extends GenericHID {
    
    LinearFilter leftXFilter, leftYFilter, rightXFilter, rightYFilter;

    /**
     * @param port
     * @param numSamples (int) - number of samples in the moving average (numSamples * .02 = time for moving average)
     */
    public MonkeyController(int port, int numSamples) {
        super(port);
        leftXFilter = LinearFilter.movingAverage(numSamples);
        leftYFilter = LinearFilter.movingAverage(numSamples);
        rightXFilter = LinearFilter.movingAverage(numSamples);
        rightYFilter = LinearFilter.movingAverage(numSamples);
    }

    private double applyPreferences(double input, double exponent, LinearFilter filter) {
        if (Math.abs(input) < Constants.ANALOG_DEAD_ZONE) {
            input = 0;
        }
        input = Math.pow(Math.abs(input), exponent) * Math.signum(input);
        if (filter == null) {
            return input;
        }
        return filter.calculate(input);
    }
    
    public double getPrimaryX() {
        return applyPreferences(getRawAxis(0), Constants.LEFT_X_EXPONENT, leftXFilter);
    }

    //flip input for Y-axis because up is negative natively
    public double getPrimaryY() {
        return applyPreferences(-getRawAxis(1), Constants.LEFT_Y_EXPONENT, leftYFilter);
    }

    public double getPrimaryTurn() {
        return applyPreferences(getRawAxis(3), 1, null);
    }

    public double getSecondaryX() {
        return applyPreferences(getRawAxis(4), Constants.RIGHT_X_EXPONENT, rightXFilter);
    }

    //flip input for Y-axis because up is negative natively
    public double getSecondaryY() {
        return applyPreferences(-getRawAxis(5), Constants.RIGHT_Y_EXPONENT, rightYFilter);
    }
    
    public double getSecondaryTurn() {
        return applyPreferences(getRawAxis(6), 1, null);
    }
}
