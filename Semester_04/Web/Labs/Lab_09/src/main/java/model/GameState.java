package model;

public class GameState {
  private Long id;
  private Long userId;
  private int score;

  private Position[] obstacles;

  private Position apple;

  private Position[] snake;

  public Long getId() {
    return id;
  }

  public void setId(Long id) {
    this.id = id;
  }

  public Long getUserId() {
    return userId;
  }

  public void setUserId(Long userId) {
    this.userId = userId;
  }

  public int getScore() {
    return score;
  }

  public void setScore(int score) {
    this.score = score;
  }

  public Position[] getObstacles() {
    return obstacles;
  }

  public void setObstacles(Position[] obstacles) {
    this.obstacles = obstacles;
  }

  public Position getApple() {
    return apple;
  }

  public void setApple(Position apple) {
    this.apple = apple;
  }

  public Position[] getSnake() {
    return snake;
  }

  public void setSnake(Position[] snake) {
    this.snake = snake;
  }
}
