package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Controller extends XboxController{
    double leftAnalogX, leftAnalogY, rightAnalogX, rightAnalogY;
    
    public Controller() {
        super(Constants.XBOX_CONTROLLER_PORT);
    }

    public void applyPreferences() {
        double[] inputs = {leftAnalogX, leftAnalogY, rightAnalogX, rightAnalogY};
        
        // TODO: does this do what I want it to do or will it only update the array
        for (int i = 0; i < 4; i++) {
            inputs[i] = curveInput(inputs[i]);
            if (Math.abs(inputs[i]) < Constants.ANALOG_DEAD_ZONE) {
                inputs[i] = 0;
            }
        }
    }


    // TODO: maybe implement it in a way that each joystick/direction has its own exponent
    private double curveInput(double input) {
        return Math.pow(Math.abs(input), Constants.EXPONENT) * Math.signum(input);
    }

    public void updateValues() {
        leftAnalogX = getLeftX();
        leftAnalogY = getLeftY();
        rightAnalogX = getRightX();
        rightAnalogY = getRightY();
    }

    //flip input for Y-axis because up is negative natively
    public double getLeftY() {
        return -super.getLeftY();
    }

    public double getRightY() {
        return -super.getRightY();
    }
}
