
package frc.robot.subsystems.LEDS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

    private final CANdle m_candle = new CANdle(60);

    private final static int kLED_COLUMNS = 4;
    private final static int kLED_ROWS = 2;
    private final static int kSTRIP_START = kLED_COLUMNS * kLED_ROWS;
    private final static int kSTRIP_LENGTH = 60;
    private final static int kLED_TOTAL = kLED_COLUMNS * kLED_ROWS + kSTRIP_LENGTH;

    private boolean m_isPanelDisabled = false;

    /** Creates a new LedPannel. */
    public LEDs() {
        CANdleConfiguration configALL = new CANdleConfiguration();
        configALL.disableWhenLOS = false;
        configALL.stripType = LEDStripType.GRB;
        configALL.brightnessScalar = 0.1; // dim the LEDs to half brightness
        // configALL.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configALL, 100);
        m_Timer.start();
    }

    private static Timer m_Timer = new Timer();

    @Override
    public void periodic() {
        // limit to 10x a second
        /*if (m_Timer.advanceIfElapsed(0.1)) {
            // Only run if not disabled
            if (DriverStation.isDisabled()) {
                // Turn all lights red
                // m_candle.setLEDs(255, 0, 0);
            }

            if (RobotContainer.m_Tracking.isTrackingTarget()) {
                this.showTrackingStatusGreen();
            } else {
                this.showTrackingStatusRed();
            }

            if (RobotContainer.m_GridSelector.isCube()) {
                this.showCube();
            } else {
                this.showCone();
            }
        }*/

    }

    /*
    public void showCone() {
        m_candle.setLEDs(255, 255, 0, 0, 0, 33 * 8);
    }

    public void showCube() {
        m_candle.setLEDs(138, 43, 226, 0, 0, 33 * 8);
    }*/

    public void setColor(int red, int green, int blue, int white, int start, int finish) {
      m_candle.setLEDs(red, green, blue, white, start, finish);
  }

    public void showVisionStatusGreen() {
        m_candle.setLEDs(0, 255, 0, 0, kSTRIP_START, kSTRIP_LENGTH);
    }

    public void showVisionStatusRed() {
        m_candle.setLEDs(255, 0, 0, 0, kSTRIP_START, kSTRIP_LENGTH);
    }

    public void showVisionStatusYellow() {
        m_candle.setLEDs(255, 255, 0, 0, kSTRIP_START, kSTRIP_LENGTH);
    }

    public void showTrackingStatusGreen() {
        m_candle.setLEDs(0, 255, 0, 0, 100, 80);
    }

    public void showTrackingStatusRed() {
        m_candle.setLEDs(255, 0, 0, 0, 100, 80);
    }

    public void showTrackingStatusYellow() {
        m_candle.setLEDs(255, 255, 0, 0, 100, 80);
    }

}




// package frc.robot.subsystems.LEDS;

// import java.util.function.Function;

// import com.ctre.phoenix.led.CANdle;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.util.Color;
// import com.ctre.phoenix.led.CANdle;
// import frc.robot.Constants;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdleConfiguration;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;

// //import frc.robot.subsystems.Subsystem;

// public class LEDs extends SubsystemBase {
//   private static LEDs m_instance;

//   private CANdle m_candle;

//   private final static int kLED_COLUMNS = 33;
//   private final static int kLED_ROWS = 8;
//   private final static int kSTRIP_START = kLED_COLUMNS * kLED_ROWS;
//   private final static int kSTRIP_LENGTH = 48;
//   private final static int kLED_TOTAL = kLED_COLUMNS * kLED_ROWS + kSTRIP_LENGTH;

//   private int m_ledTotalLength = Constants.LEDs.k_totalLength;

//   // Main sections
//   private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_ledStripColor = LEDModes
//       .setColor(Color.kRed);

//   public static LEDs getInstance() {
//     if (m_instance == null) {
//       m_instance = new LEDs();
//     }
//     return m_instance;
//   }

//   private LEDs() {
//     super("LEDs");
//     m_candle = new CANdle(60);
//     // m_CANdle61 = new CANdle(61);
//     //m_led = new AddressableLED(Constants.LEDs.k_PWMId);
//     CANdleConfiguration configALL = new CANdleConfiguration();
//     configALL.disableWhenLOS = false;
//     configALL.stripType = LEDStripType.GRB;
//     configALL.brightnessScalar = 0.1; // dim the LEDs to half brightness
//     // configALL.vBatOutputMode = VBatOutputMode.Modulated;
//     m_candle.configAllSettings(configALL, 100);
//     m_Timer.start();
//   }

//   private static Timer m_Timer = new Timer();

//   @Override
//   public void periodic() {
//     setColorMode();

//     //m_led.setData(m_buffer);
//   }

//   public void setColor(int red, int green, int blue, int white, int start, int finish) {
//     m_candle.setLEDs(red, green, blue, white, start, finish);
//     // m_CANdle61.setLEDs((int) color.red*255,(int) color.green*255,(int) color.blue*255);
//     //m_candle.configBrightnessScalar(1.0);
//     // m_CANdle61.configBrightnessScalar(1.0);
//   }

//   public void defaultLEDS() {
//     breathe();
//   }

//   public void chase() {
//     m_ledStripColor = LEDModes.redChase;
//   }

//   public void breathe() {
//     m_ledStripColor = LEDModes.redBreathe;
//   }

//   public void rainbow() {
//     m_ledStripColor = LEDModes.rainbow;
//   }

//   public void setColorMode() {
//     //m_buffer = m_ledStripColor.apply(0).apply(Constants.LEDs.k_totalLength).apply(m_buffer);
//   }
// }