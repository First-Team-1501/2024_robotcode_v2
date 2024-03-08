package frc.robot.subsystems.thumbwheel;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Thumbwheel extends SubsystemBase {
    ArrayList<DigitalInput> binaryControls;

    // TODO: configure
    final private int ioLineStart = 1;

    public Thumbwheel() {

        binaryControls = new ArrayList<DigitalInput>();
        for (int i = ioLineStart; i < ioLineStart + 4; i++) {
            binaryControls.add(new DigitalInput(i));

            ShuffleBoardInit();

        }

    }

    public void periodic() {
        //SmartDashboard.putNumber("thumb", getValue());
    }

    public int getValue() {
        // *shrugs*
        // listen, nobody said the code had to make any sense...
        String s = "";
        for (int i = binaryControls.size() - 1; i >= 0; i--) {
            s += binaryControls.get(i).get() ? "0" : "1";
        }

        return (Integer.parseInt(s, 2));
    }

    public void ShuffleBoardInit() {
        
    }
}