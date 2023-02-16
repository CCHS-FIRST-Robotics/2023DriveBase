package frc.robot.subsystems;
import frc.robot.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;

public class Test {
    // ProfiledPIDController shoulderPID, elbowPID;
    PIDController shoulderPID, elbowPID;
    
    public Test() {
        // shoulderPID = new ProfiledPIDController(Constants.SHOULDER_KP, Constants.SHOULDER_KI, Constants.SHOULDER_KD,
		// 	new TrapezoidProfile.Constraints(Constants.SHOULDER_MAX_VELOCITY, Constants.SHOULDER_MAX_ACCELERATION)
		// );
        // elbowPID = new ProfiledPIDController(Constants.ELBOW_KP, Constants.ELBOW_KI, Constants.ELBOW_KD,
		// 	new TrapezoidProfile.Constraints(Constants.ELBOW_MAX_VELOCITY, Constants.ELBOW_MAX_ACCELERATION)
		// );

        shoulderPID = new PIDController(Constants.SHOULDER_KP, Constants.SHOULDER_KI, Constants.SHOULDER_KD
		);
        elbowPID = new PIDController(Constants.ELBOW_KP, Constants.ELBOW_KI, Constants.ELBOW_KD
		);
    }

    public void run(double measurement, double setpoint) {
        System.out.println(
          "VOLTAGE: " + shoulderPID.calculate(measurement, setpoint) +
          " SETPOINT: " + shoulderPID.getSetpoint()//.position
        );
    }
}