package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class Grabber {

DoubleSolenoid solenoidPCMClaw, solenoidPCMWrist;
Compressor pcmCompressor;

boolean clawForward, wristForward;

    /**
    * Constructor for Grabber Class
    * 
    * @param clawForwardChannelNum
    * @param clawReverseChannelNum
    * @param wristForwardChannelNum
    * @param wristReverseChannelNum
    */
    public Grabber(int clawForwardChannelNum, int clawReverseChannelNum, 
                    int wristForwardChannelNum, int wristReverseChannelNum) {
        // Each solenoid has two sets of positive and negative leads, one responsible for going forward the other for going back
        // Therefore, each object is passed the appropriate numbers that correlate with the values
        solenoidPCMClaw  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, clawForwardChannelNum,  clawReverseChannelNum);
        solenoidPCMWrist = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, wristForwardChannelNum, wristReverseChannelNum);

        pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);

        clawForward = true;
        wristForward = true;
        
        // This if statment to make sure the compressor runs, if it is not running
        if(!compIsEnabled())
            pcmCompressor.isEnabled();
        else
            pcmCompressor.enableDigital();
            //used in this case because a switch (modulated by the CTRE Pneumatics Control Module) regulates the compressor
    }

    /*
     * WRIST CONTROL
     */

    public void toggleWrist() {
        wristForward = !wristForward;
        solenoidPCMWrist.toggle();
    }

    public void wristForward(){
        wristForward = true;
        solenoidPCMWrist.set(kForward);
    }

    public void wristBack(){
        wristForward = false;
        solenoidPCMWrist.set(kReverse);
    }

    public void wristStop(){
        solenoidPCMWrist.set(kOff);
    }

    /*
     * CLAW CONTROL
     */

    public void toggleClaw() {
        clawForward = !clawForward;
        solenoidPCMClaw.toggle();
    }

    // closes the claw
    public void clawForward(){
        clawForward = true;
        solenoidPCMClaw.set(kForward);
    }

    // opens the claw
    public void clawBack(){
        clawForward = false;
        solenoidPCMClaw.set(kReverse);
    }

    public void clawStop(){
        solenoidPCMClaw.set(kOff);
    }

	// TODO: write method
	public boolean isWristActuated() {
		return wristForward;
	}

    public boolean isClawActuated() {
        return clawForward;
    }

    // just for you colin, this function (in case you didn't guess it) passes back whether the compressor is enabled
    // this is for conveincice, so we can call the function from anywhere in the program witout needing the compressor object
    // wow im so honored <3
    public boolean compIsEnabled() {
        return pcmCompressor.isEnabled();
    }



}


// Other variables we can set up if we want (they were set up in example code):
//  boolean enabled = pcmCompressor.enabled();
//  boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
//  double current = pcmCompressor.getCompressorCurrent();