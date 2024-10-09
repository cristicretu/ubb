package repository;

public class capacityExceededException extends Exception {
  public capacityExceededException(String message) {
    super(message);
  }

  public capacityExceededException() {
    super("Capacity exceeded");
  }
}
