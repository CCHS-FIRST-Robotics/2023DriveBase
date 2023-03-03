package frc.robot;

// import java.lang.Math;
// import edu.wpi.first.wpilibj.SPI;

/**
 * Constants - set once, and use throught the code
 * Naming convention is all caps, spaces are underscores LIKE_THIS
 */
public class Constants {

	//height from floor to center of limelight lense in meters
	public static final double LIME_HEIGHT    = .42;
	//number of degrees from perfectly vertical
	public static final double LIME_ANGLE     = 0; 

	//height from floor to target in meters (added 2 inches)
	public static final double SHORT_TARGET_HEIGHT = .56 + 0.0508;
	public static final double TALL_TARGET_HEIGHT = 1.05  + 0.0508;

	public static final int SHORT_PIPE_NUM = 0;
	public static final int TALL_PIPE_NUM  = 1;

	// make sure its in the same units as the limelight vals
	public static final double TARGETS_DISTANCE = .42; // pipes are ~.42 meters away from each other in the lateral direction

	Constants() {
		
	}
}
