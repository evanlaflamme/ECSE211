package ca.mcgill.ecse211.lab1;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
