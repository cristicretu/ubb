package model;

import java.time.LocalDateTime;

public class Move {
  private Long id;
  private Long gameStateId;
  private Long userId;
  private String direction;
  private int fromX;
  private int fromY;
  private int toX;
  private int toY;
  private boolean ateApple;
  private int scoreAfterMove;
  private LocalDateTime moveTime;

  public Move() {
  }

  public Move(Long gameStateId, Long userId, String direction, int fromX, int fromY,
      int toX, int toY, boolean ateApple, int scoreAfterMove) {
    this.gameStateId = gameStateId;
    this.userId = userId;
    this.direction = direction;
    this.fromX = fromX;
    this.fromY = fromY;
    this.toX = toX;
    this.toY = toY;
    this.ateApple = ateApple;
    this.scoreAfterMove = scoreAfterMove;
    this.moveTime = LocalDateTime.now();
  }

  public Long getId() {
    return id;
  }

  public void setId(Long id) {
    this.id = id;
  }

  public Long getGameStateId() {
    return gameStateId;
  }

  public void setGameStateId(Long gameStateId) {
    this.gameStateId = gameStateId;
  }

  public Long getUserId() {
    return userId;
  }

  public void setUserId(Long userId) {
    this.userId = userId;
  }

  public String getDirection() {
    return direction;
  }

  public void setDirection(String direction) {
    this.direction = direction;
  }

  public int getFromX() {
    return fromX;
  }

  public void setFromX(int fromX) {
    this.fromX = fromX;
  }

  public int getFromY() {
    return fromY;
  }

  public void setFromY(int fromY) {
    this.fromY = fromY;
  }

  public int getToX() {
    return toX;
  }

  public void setToX(int toX) {
    this.toX = toX;
  }

  public int getToY() {
    return toY;
  }

  public void setToY(int toY) {
    this.toY = toY;
  }

  public boolean isAteApple() {
    return ateApple;
  }

  public void setAteApple(boolean ateApple) {
    this.ateApple = ateApple;
  }

  public int getScoreAfterMove() {
    return scoreAfterMove;
  }

  public void setScoreAfterMove(int scoreAfterMove) {
    this.scoreAfterMove = scoreAfterMove;
  }

  public LocalDateTime getMoveTime() {
    return moveTime;
  }

  public void setMoveTime(LocalDateTime moveTime) {
    this.moveTime = moveTime;
  }
}