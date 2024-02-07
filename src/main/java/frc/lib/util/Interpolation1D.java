package frc.lib.util;

import java.util.ArrayList;
import java.util.Optional;

public class Interpolation1D {
  private final ArrayList<Double> values = new ArrayList<Double>();
  private final ArrayList<Double> targets = new ArrayList<Double>();

  public Interpolation1D(double[]... args) {
    for (var arg : args) {
      values.add(arg[0]);
      targets.add(arg[1]);
    }
  }

  public Optional<Double> getTarget(double value) {
    for (int i = 0; i < values.size() - 1; i++) {
      var currentValue = values.get(i);
      var currentTarget = targets.get(i);
      var nextValue = values.get(i + 1);
      var nextTarget = targets.get(i + 1);
      if (value >= currentValue && value <= nextValue) {
        var slope = (nextTarget - currentTarget) / (nextValue - currentValue);

        return Optional.of(slope * (value - currentValue) + currentTarget);
      }
    }

    return Optional.empty();
  }

}
