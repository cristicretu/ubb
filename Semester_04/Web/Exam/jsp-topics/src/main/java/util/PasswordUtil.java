package util;

import org.mindrot.jbcrypt.BCrypt;

public class PasswordUtil {

  // Number of rounds for BCrypt hashing (12 is a good balance of security and
  // performance)
  private static final int ROUNDS = 12;

  /**
   * Hash a plain text password using BCrypt
   * 
   * @param plainTextPassword The plain text password to hash
   * @return The hashed password
   */
  public static String hashPassword(String plainTextPassword) {
    return BCrypt.hashpw(plainTextPassword, BCrypt.gensalt(ROUNDS));
  }

  /**
   * Verify a plain text password against a hashed password
   * 
   * @param plainTextPassword The plain text password to verify
   * @param hashedPassword    The hashed password to verify against
   * @return true if the password matches, false otherwise
   */
  public static boolean verifyPassword(String plainTextPassword, String hashedPassword) {
    return BCrypt.checkpw(plainTextPassword, hashedPassword);
  }

  /**
   * Check if a password meets basic security requirements
   * 
   * @param password The password to validate
   * @return true if password is valid, false otherwise
   */
  public static boolean isValidPassword(String password) {
    if (password == null || password.length() < 6) {
      return false;
    }

    // Check for at least one letter and one number
    boolean hasLetter = false;
    boolean hasDigit = false;

    for (char c : password.toCharArray()) {
      if (Character.isLetter(c)) {
        hasLetter = true;
      }
      if (Character.isDigit(c)) {
        hasDigit = true;
      }
    }

    return hasLetter && hasDigit;
  }

  /**
   * Get password validation error message
   * 
   * @param password The password to validate
   * @return Error message or null if password is valid
   */
  public static String getPasswordValidationError(String password) {
    if (password == null || password.trim().isEmpty()) {
      return "Password is required";
    }

    if (password.length() < 6) {
      return "Password must be at least 6 characters long";
    }

    boolean hasLetter = false;
    boolean hasDigit = false;

    for (char c : password.toCharArray()) {
      if (Character.isLetter(c)) {
        hasLetter = true;
      }
      if (Character.isDigit(c)) {
        hasDigit = true;
      }
    }

    if (!hasLetter) {
      return "Password must contain at least one letter";
    }

    if (!hasDigit) {
      return "Password must contain at least one number";
    }

    return null; // Password is valid
  }
}