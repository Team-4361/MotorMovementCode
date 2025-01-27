package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;

    public class Piston extends TimedRobot {
      //Declares what is plugged into the Drive Station
    private final Joystick m_stick = new Joystick(0);

// Solenoid corresponds to a single solenoid.
private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

// DoubleSolenoid corresponds to a double solenoid.
private final DoubleSolenoid m_doubleSolenoid =
new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);

//Button configurations
private static final int kSolenoidButton = 1;
private static final int kDoubleSolenoidForward = 2;
private static final int kDoubleSolenoidReverse = 3;

@Override
public void teleopPeriodic() {
/*
* The output of GetRawButton is true/false depending on whether
* the button is pressed; Set takes a boolean for whether
* to use the default (false) channel or the other (true).
*/
m_solenoid.set(m_stick.getRawButton(kSolenoidButton));

/*
 * If B is pressed, extend the piston
 * If X is pressed, retract the piston
 */
if (m_stick.getRawButton(kDoubleSolenoidForward)) {
  m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
} else if (m_stick.getRawButton(kDoubleSolenoidReverse)) {
  m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
}

}
}