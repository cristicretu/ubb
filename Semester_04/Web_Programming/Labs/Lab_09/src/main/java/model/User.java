package model;

import java.time.LocalDateTime;

public class User {
  private Long id;
  private String username;
  private String hashedPassword;
  private int highScore;
  private LocalDateTime createdAt;
  private LocalDateTime lastLogin;

  // Default constructor
  public User() {
    this.createdAt = LocalDateTime.now();
  }

  // Constructor with basic fields
  public User(String username, String hashedPassword) {
    this();
    this.username = username;
    this.hashedPassword = hashedPassword;
    this.highScore = 0;
  }

  // Getters and Setters
  public Long getId() {
    return id;
  }

  public void setId(Long id) {
    this.id = id;
  }

  public String getUsername() {
    return username;
  }

  public void setUsername(String username) {
    this.username = username;
  }

  public String getHashedPassword() {
    return hashedPassword;
  }

  public void setHashedPassword(String hashedPassword) {
    this.hashedPassword = hashedPassword;
  }

  public int getHighScore() {
    return highScore;
  }

  public void setHighScore(int highScore) {
    this.highScore = highScore;
  }

  public LocalDateTime getCreatedAt() {
    return createdAt;
  }

  public void setCreatedAt(LocalDateTime createdAt) {
    this.createdAt = createdAt;
  }

  public LocalDateTime getLastLogin() {
    return lastLogin;
  }

  public void setLastLogin(LocalDateTime lastLogin) {
    this.lastLogin = lastLogin;
  }

  @Override
  public String toString() {
    return "User{" +
        "id=" + id +
        ", username='" + username + '\'' +
        ", highScore=" + highScore +
        ", createdAt=" + createdAt +
        ", lastLogin=" + lastLogin +
        '}';
  }
}