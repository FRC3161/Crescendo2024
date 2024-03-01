package frc.lib.util;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Class representing Statistic Utils used for regression.
 * 
 * @author gorosgobe
 */
public class StatisticUtils {

  /** Default EPSILON used for double comparison */
  public static final double EPSILON = 0.00001;

  /**
   * Looks for equality between two doubles within the epsilon parameter.
   * 
   * @param d1 the first double
   * @param d2 the second double
   * @return are both doubles equal within the default epsilon?
   */
  public static boolean isApproxEqual(double d1, double d2) {
    return Math.abs(d1 - d2) <= EPSILON;
  }

  /**
   * Looks for equality between two doubles within the epsilon parameter.
   * 
   * @param d1      the first double
   * @param d2      the second double
   * @param epsilon the epsilon to use in the comparison
   * @return are both doubles equal within the epsilon supplied?
   */
  public static boolean isApproxEqual(double d1, double d2, double epsilon) {
    return Math.abs(d1 - d2) <= epsilon;
  }

  /**
   * Gets the mean of the supplied list of doubles
   * 
   * @param list the doubles to compute the mean of
   * @return the mean of the supplied list of doubles
   */
  public static double mean(List<Double> list) {

    double sum = list.stream().reduce(0.0, (a, b) -> a + b);
    return sum / list.size();

  }

  /**
   * Computes the mean of the supplied array of doubles
   * 
   * @param doubles an array of doubles to compute the mean of
   * @return the mean of the supplied array of doubles
   */
  public static double mean(Double[] doubles) {
    return mean(Arrays.asList(doubles));
  }

  /**
   * Computes the variance of a list of doubles
   * 
   * @param list the list of double to compute the variance of
   * @return the variance of the list of doubles
   */
  public static double variance(List<Double> list) {

    double mean = mean(list);

    return list.stream()
        .map(i -> (i - mean) * (i - mean))
        .reduce(0.0, (a, b) -> a + b);
  }

  /**
   * Computes the variance of an array of doubles
   * 
   * @param doubles the array of doubles to compute the variance of
   * @return the variance of the array of doubles
   */
  public static double variance(Double[] doubles) {
    return variance(Arrays.asList(doubles));
  }

  /**
   * Gets the covariance of the supplied lists of doubles
   * 
   * @param xs the xs of the points
   * @param ys the ys of the points
   * @return the covariance of the points formed by both lists
   */
  public static double covariance(List<Double> xs, List<Double> ys) {

    double meanX = mean(xs);
    double meanY = mean(ys);

    double covariance = 0.0;

    for (int i = 0; i < xs.size(); i++) {
      covariance += (xs.get(i) - meanX) * (ys.get(i) - meanY);
    }

    return covariance;

  }

  /**
   * Gets the covariance of the list of points supplied
   * 
   * @param points the points to compute the covariance of
   * @return the covariance of the list of points supplied
   */
  public static double covariance(List<Point> points) {
    return covariance(getListOfX(points), getListOfY(points));
  }

  /**
   * Gets a list of the Xs of the points supplied
   * 
   * @param list A list of the points to get the Xs of
   * @return a list of the Xs of the points supplied
   */
  public static List<Double> getListOfX(List<Point> list) {
    return list.stream()
        .map(i -> i.getX())
        .collect(Collectors.toList());
  }

  /**
   * Gets a list of the Ys of the points supplied
   * 
   * @param list A list of the points to get the Ys of
   * @return a list of the Ys of the points supplied
   */
  public static List<Double> getListOfY(List<Point> list) {
    return list.stream()
        .map(i -> i.getY())
        .collect(Collectors.toList());
  }

}
