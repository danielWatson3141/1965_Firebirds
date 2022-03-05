package frc.robot;

import java.text.SimpleDateFormat;
import java.util.Date;

public class Logging {

    public static void log(String where, String message){    
        String timeStamp = new SimpleDateFormat("HH.mm.ss").format(new Date());
        System.out.println(timeStamp+"/"+where+ ":" +message);
    }
}
