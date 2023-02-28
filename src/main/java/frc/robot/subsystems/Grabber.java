package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class Grabber {
DoubleSolenoid solenoidPCMClaw, solenoidPCMWrist;
Compressor pcmCompressor;

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
        
        // This if statment to make sure the compressor runs, if it is not running
        if(!compIsEnabled())
            pcmCompressor.isEnabled();
        else
            pcmCompressor.enableDigital();
            //used in this case because a switch (modulated by the CTRE Pneumatics Control Module) regulates the compressor
    }

    // For the next six functions set() is used to accordingly make the pnuematic go forward, go backward, or stop
    public void wristForward(){
        solenoidPCMWrist.set(kForward);
    }

    public void wristBack(){
        solenoidPCMWrist.set(kReverse);
    }

    public void wristStop(){
        solenoidPCMWrist.set(kOff);
    }

    public void clawForward(){
        solenoidPCMClaw.set(kForward);
    }

    public void clawBack(){
        solenoidPCMClaw.set(kReverse);
    }

    public void clawStop(){
        solenoidPCMClaw.set(kOff);
    }

    // just for you colin, this function (in case you didn't guess it) passes back whether the compressor is enabled
    // this is for conveincice, so we can call the function from anywhere in the program witout needing the compressor object
    public boolean compIsEnabled() {
        return pcmCompressor.isEnabled();
    }



}


// Other variables we can set up if we want (they were set up in example code):
//  boolean enabled = pcmCompressor.enabled();
//  boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
//  double current = pcmCompressor.getCompressorCurrent();