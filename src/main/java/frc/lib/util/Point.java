package frc.lib.util;

/**
 * Utility class representing a point.
 * 
 * @author gorosgobe
 */
public class Point {

  /** Double representing the x of the point */
  private final double x;
  /** Double representing the y of the point */
  private final double y;

  /**
   * Constructor for a point
   * 
   * @param x x of the point
   * @param y y of the point
   */
  public Point(double x, double y) {
    this.x = x;
    this.y = y;
  }

  /**
   * Gets the x of the point
   * 
   * @return the x of the point
   */
  public double getX() {
    return x;
  }

  /**
   * Gets the y of the point
   * 
   * @return the y of the point
   */
  public double getY() {
    return y;
  }

  /**
   * Overridden toString function for Point
   * 
   * @return the String for the point, of the format <em>(x, y)</em>
   */
  @Override
  public String toString() {
    return "(" + x + ", " + y + ")";
  }

  /**
   * Overridden equals function for Point
   * 
   * @param o the object to compare the Point to
   * @return is the Point equal to the object supplied?
   */
  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Point)) {
      return false;
    }

    Point other = (Point) o;

    return other.getX() == this.getX() && other.getY() == this.getY();
  }

  /**
   * Overridden hashcode method for Point
   * 
   * @return the hashcode for the Point
   */
  @Override
  public int hashCode() {
    int result;
    long temp;
    temp = Double.doubleToLongBits(x);
    result = (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(y);
    result = 31 * result + (int) (temp ^ (temp >>> 32));
    return result;
  }
}
