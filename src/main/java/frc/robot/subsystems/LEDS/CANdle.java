//New LED code file. NEEDS DEBUGGING!

package.frc.robot.subsystems.LEDS;

import com.ctre.phoenix.led.CANdle;
import frc.robot.Constants;

public class CANdle extends SubsystemBase {
    
    //Create a CANdle with a GRB LED strip attached
    public CANdle m_CANdle60 = new CANdle(60);
    m_CANdle60.configLEDType(GRB);

}