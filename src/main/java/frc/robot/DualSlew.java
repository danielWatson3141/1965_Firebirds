package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DualSlew extends SlewRateLimiter {

    enum Dir {
        STOP, POS, NEG,
    }

    // Initialize m_dir to Day.ZERO and m_mag to 0.0
    private Dir m_dir = Dir.STOP;
    private double m_mag = 0.0;

    DualSlew(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        super(positiveRateLimit, negativeRateLimit, initialValue);
        m_dir = initialValue == 0 ? Dir.ZERO : initialValue > 0 ? Dir.POS : Dir.NEG;
        m_mag = Math.abs(initialValue);
    }
    
    // Calculate method from SlewRateLimiter
    @Override
    public double calculate(double input) {
        // Determine the direction based on the input
        Dir i_dir = (input == 0) ? Dir.ZERO : (input > 0) ? Dir.POS : Dir.NEG;
            
        // Calculate the absolute value of the input
        double i_mag = Math.abs(input);
    
        // Handle different cases based on the current and input directions
        if (m_dir == Dir.STOP) {
            // If the current direction is ZERO/STOP, update m_mag and m_dir from input
            m_mag = super.calculate(i_mag);
            m_dir = i_dir;
        } else if (m_dir == i_dir) {
            // If the current direction is the same as the input, update m_mag
            m_mag = super.calculate(i_mag);
        } else {
            // If directions are different, reset m_mag and update m_dir accordingly
            m_mag = super.calculate(0.0);
            // If the speed ever stops, reset to ZERO/STOP
            if (m_mag == 0) {
                m_dir = Dir.STOP;
            }
        }

        // Return the result based on the direction
        return (m_dir == Dir.NEG) ? (-1 * m_mag) : m_mag;
    }

}
