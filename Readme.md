
# 2023 Drive Base
[![CI](https://github.com/CCHS-FIRST-Robotics/2023DriveBase/actions/workflows/main.yml/badge.svg)](https://github.com/CCHS-FIRST-Robotics/2023DriveBase/actions/workflows/main.yml)
may be important https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/MecanumDriveWheelPositions.html
[DRIVE subsystem tutorial (follow)](https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-drive-subsystem.html) 

[Project Board](https://docs.google.com/document/d/15ajK8g77htJsg22ZYEKL1gVa0hAAOZJdVJMOak42eIU/edit?usp=sharing)

## Contributing

### The enviroment

FIRST robotics projects are made in [Visual Studio Code](https://code.visualstudio.com), with the wpilib extension. Look up "wpilib" in the extensions tab of vs-code once it is installed. Use [wpilib commands to build and deploy the code](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/vscode-basics.html). 

Download the repository with `git clone https://github.com/CCHS-FIRST-Robotics/2023DriveBase.git`
 
### How to add your amazing changes!

1. Run `doxygen` to update the docs (how to install doxygen is in the "Adding documentation" section).
2. Create a branch:
`git branch *name*` (replace "\*name\*" with what changes you intend to make).
3. Track all of your new changes:
`git add .`
4. Commit the changes: `git commit -m "*changes*"` (replace "\*changes\*" with what changes you made)
5. Push the commit to the repository: `git push` 


[New to github?](https://docs.github.com/en/get-started/quickstart/set-up-git)

## Documentation

### Adding documentation

The documentation pages and visuals are created by doxygen. The doxygen configuration is almost completely default, its file is "Doxygen" [learn more about doxygen](https://www.doxygen.nl 'Doxygen')

When updating code, comments, or anything that should be in documentation, run 
`doxygen` 
before commiting. Install Doxygen on mac with [Brew](https://brew.sh): `brew install doxygen`

If you want to change the doxygen configuration, use the doxygen gui, [doxywizard](https://www.doxygen.nl/download.html). Make sure you save the outputted doxygen config in the Doxygen file. 

### How the drive base works

The "heart" of the robot is the roborio, which uses wpilib for instructions. There are a couple [base robot templates](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/creating-robot-program.html), but we are using the `TimedRobot`.

If you want to understand wpilib, start [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html). After that, you will need the [Java library documentation](https://github.wpilib.org/allwpilib/docs/release/java/). 

Then, learn about our [drivebase code](https://cchs-first-robotics.github.io/2023DriveBase/html/annotated.html). 

### How the arm works

The arm is controlled very simply the same way we control the drive base. The most difficult part to understand is the kinematic models that help us determine how we should move the motors in order to move the arm at a certain (x, y) linear veloicty -- as opposed to controlling the angular veocity -- or finding the angular position the encoders need to be for a given cartesian (x, y) position. 

If you would like to better understand these models, here are some diagrams/math which show the derivation:
![alt text](https://github.com/CCHS-FIRST-Robotics/2023DriveBase/blob/main/images/2R-positional-forward-inverse-kinematics.jpg)
![alt text](https://github.com/CCHS-FIRST-Robotics/2023DriveBase/blob/main/images/2R-speed-inverse-kinematics.jpg)

### Motors and components

Motors/motor controllers and other components of the robot will need other java libraries, and have other docs aswell.

We are using [Rev Robotics SparkMax](https://codedocs.revrobotics.com/java/index.html) and [ctre Phoenix Talon and Victor](https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/package-summary.html) motor controllers (links are to their libraries).

The robot also uses a [NavX Gyro](https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/package-summary.html). The gyro can get lots of information about the robot's orientation, position, etc.

When adding a new component, look up instructions on how to install it, and then update this doc with its library's documenation.

## Relevant Articles

[Kinematics](http://motion.cs.illinois.edu/RoboticSystems/Kinematics.html#:~:text=Kinematics%20is%20the%20study%20of,and%20classical%20topic%20in%20robotics.)