package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj.GenericHID;
import java.util.ArrayList;
import java.util.List;

public class ButtonBox {
  List<ButtonBoxPanel> panels = new ArrayList<>();
  GenericHID wrapped;

  public ButtonBox(int index) {
    wrapped = new GenericHID(index);
  }

  boolean getBitFromFloat(float f, int ind) {
    int val = 0;
    if (f < 0) {
      val = (int) f * 128;
    } else {
      val = (int) f * 127;
    }
    val += 128;
    return (val & ((int) Math.pow(2, ind))) == 1;
  }

  private boolean getButton(int ind) {
    int digital_outputs = 21;
    int pov_bits = 3;
    int analog_bits = 8;
    int curr = digital_outputs;
    if (ind <= curr) {
      return wrapped.getRawButton(ind);
    }
    curr += pov_bits;
    if (ind <= curr) {
      int pov_bit = ind - curr + pov_bits - 1;
      int pov_as_bits = wrapped.getPOV() / 45;
      return (pov_as_bits & ((int) Math.pow(2, pov_bit))) == 1;
    }
    curr += analog_bits;
    if (ind <= curr) {
      int realind = ind - curr + analog_bits - 1;
      float realJStk = (float) wrapped.getRawAxis(3);
      return getBitFromFloat(realJStk, realind);
    }

    curr += analog_bits;
    if (ind <= curr) {
      int realind = ind - curr + analog_bits - 1;
      float realJStk = (float) wrapped.getRawAxis(4);
      return getBitFromFloat(realJStk, realind);
    }

    curr += analog_bits;
    if (ind <= curr) {
      int realind = ind - curr + analog_bits - 1;
      float realJStk = (float) wrapped.getRawAxis(5);
      return getBitFromFloat(realJStk, realind);
    }
    return false;
  }

  void registerPanel(ButtonBoxPanel panel) {
    panels.add(panel);
    panel.panel_index = panels.size() - 1;
  }

  public boolean getButtonOnPanel(int pIND, int bIND) {
    int zeroIndex = 1;
    for (int i = 0; i < pIND; i++) {
      zeroIndex += panels.get(i).getButtons();
    }
    return getButton(zeroIndex + bIND);
  }
}
