package ca.mcgill.ecse211.lab4;

public class UltrasonicLocalizer {
  private int distance;
  
  public enum Method { RISING_EDGE, FALLING_EDGE };
  private Method m;
  
  public UltrasonicLocalizer(Method m) {
    this.m = m;
  }
  
  public void processUSData(int distance) {
    if (m == Method.RISING_EDGE) {
      risingEdge(distance);
    } else {
      fallingEdge(distance);
    }
    
    System.out.println(distance);
  }
  
  private void risingEdge(int distance) {
    
  }
  
  private void fallingEdge(int distance) {
    
  }

  public int readUSDistance() {
    return this.distance;
  }
}
