package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Controller extends XboxController{
    
    public Controller() {
        super(Constants.XBOX_CONTROLLER_PORT);
    }
}
