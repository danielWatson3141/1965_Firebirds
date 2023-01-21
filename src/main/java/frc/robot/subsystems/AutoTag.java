package frc.robot.subsystems;

import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SixWheelDrivetrain;

public class AutoTag {

    double leftTagAngle = vision.___;
    double rightTagAngle = vision.___;
    double tagDistance = vision.___;
    //above is info needed from vision
private void tagAngle() {
    if (leftTagAngle > rightTagAngle) {
        //return rightTagAngle
    }
    else {
        //return leftTagAngle
    } 
}

double horozontalDistance = (tagDistance * Math.cos(tagAngle()));
double verticleDistance = (horozontalDistance * Math.tan(tagAngle()));

private void directions() {
    if (leftTagAngle > rightTagAngle) {
        //turn left
    }
    else {
        //turn right
    }  
}
}
