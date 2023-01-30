package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.kauailabs.navx.frc.AHRS;


/* 
MANAGES SENSOR DATA
*/
public class Sensors {
	AHRS navx;

	float x, y, z, heading; // define navx outputs

	public void getValues() {
		x = navx.getDisplacementX();
		y = navx.getDisplacementY();
		z = navx.getDisplacementZ();

		heading = navx.getFusedHeading();
	}

	public void pushNetworkTables() {
		NetworkTable table   = NetworkTableInstance.getDefault().getTable("numbers");
		table.putNumber("NavX", x);
		table.putNumber("NavY", y);
		table.putNumber("NavZ", z);
	}

	public void pushShuffleboard() {
		SmartDashboard.putNumber("NavX", x);
        SmartDashboard.putNumber("NavY", y);
        SmartDashboard.putNumber("NavZ", z);
        SmartDashboard.putNumber("NavHead", heading);
	}
}