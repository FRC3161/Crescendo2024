package frc.lib.util;

import java.util.List;

/**
 * Class representing a data point with multiple independent variables and one
 * dependent variable.
 * 
 * @author gorosgobe
 */
public class MultiplePoint {

  /** List representing the independent variables */
  private final List<Double> Xs;
  /** Double representing the dependent variable */
  private final double y;

  /**
   * Constructor for a multiple Point
   * 
   * @param xs a list of the xs of the multiple point
   * @param y  the dependent variable of the multiple point
   */
  public MultiplePoint(List<Double> xs, double y) {
    Xs = xs;
    this.y = y;
  }

  /**
   * Adds the given <em>x</em> to the list of independent variables
   * 
   * @param x the x to add to the list of independent variables.
   */
  public void addX(double x) {
    Xs.add(x);
  }

  /**
   * Gets the list of the independent variables
   * 
   * @return a list with the independent variables.
   */
  public List<Double> getXs() {
    return Xs;
  }

  /**
   * Gets the dependent variable, <em>y</em>
   * 
   * @return the depedent variable, <em>y</em>.
   */
  public double getY() {
    return y;
  }
}
