package frc.robot.subsystems;

import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SixWheelDrivetrain;
import edu.wpi.first.math.geometry.Transform3d;

public class AutoTag {

    Transform3d myPosition;
    
    //already givem
    double zeX = myPosition.getX();
    double zeY = myPosition.getX();
    //calculated using triganometry
    double h = (Math.sqrt((zeX * zeX) + (zeY * zeY)));
    double angleA = Math.asin(zeY / h);

public void directions() {
    if (zeX > 0) {
        //turn angleA, turn right, move left motors
        //move x
        //Turn -90°, left
        //move zeY
    }
    else if (zeX < 0) {
        //turn -angleA, turn left, move right motors
        //move x
        //Turn 90°, right
        //move zeY
    }  
    else {
    //don't move
    }
}
}
