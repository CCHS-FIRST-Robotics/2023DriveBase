// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;

/** Add your docs here. */
public class LinearProfile {


    
    public static double[] getSetPoints(double[] initialPosition, double[] finalPosition, double[] initialVelocity) {
        
        
        return new double[1];
    }

    private static double[] getAcceleratingSetPoints(double timeInterval) {

        int numberOfSteps = (int) (Math.ceil(timeInterval / Constants.PERIOD));

        double[] setPoints = new double[numberOfSteps];

        // loops through all the time increments
        for(int i = 0; i < numberOfSteps; i++) {
            double positionMag = Constants.ARM_MAX_ACCELERATION / 2 * Constants.PERIOD * Constants.PERIOD * (i+1) * (i+1);
        }

        return new double[1];
    }

    private static double magnitude(double[] array) {
        return Math.sqrt(array[0] * array[0] + array[1] * array[1]);
    }

}


