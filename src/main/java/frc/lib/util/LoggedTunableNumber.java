// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private double previousValue = 0;

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param key          name
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String key, double defaultValue) {
    initDefault(defaultValue);
    this.key = key;
    SmartDashboard.putNumber(getName(), defaultValue);
  }

  private String getName() {
    return tableKey + "/" + key;
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return SmartDashboard.getNumber(getName(), defaultValue);
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared
   *           between multiple
   *           objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was
   *         called, false
   *         otherwise.
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (previousValue != currentValue) {
      previousValue = currentValue;
      return true;
    }
    return false;
  }
}