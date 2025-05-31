import model.User;
import repository.UserRepository;
import util.PasswordUtil;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;
import java.io.IOException;
import java.sql.SQLException;
import java.time.LocalDateTime;
import java.util.Optional;

@WebServlet("/user")
public class UserServlet extends HttpServlet {
  private UserRepository userRepository;

  @Override
  public void init() throws ServletException {
    userRepository = new UserRepository();
  }

  @Override
  protected void doGet(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    String action = request.getParameter("action");

    try {
      switch (action != null ? action : "") {
        case "logout":
          logout(request, response);
          break;
        default:
          response.sendError(HttpServletResponse.SC_BAD_REQUEST, "Invalid action");
          break;
      }
    } catch (Exception e) {
      throw new ServletException("Error processing request", e);
    }
  }

  @Override
  protected void doPost(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    String action = request.getParameter("action");

    try {
      switch (action != null ? action : "") {
        case "register":
          registerUser(request, response);
          break;
        case "login":
          loginUser(request, response);
          break;
        case "updateScore":
          updateScore(request, response);
          break;
        default:
          response.sendError(HttpServletResponse.SC_BAD_REQUEST, "Invalid action");
          break;
      }
    } catch (SQLException e) {
      throw new ServletException("Database error", e);
    }
  }

  private void registerUser(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException, ServletException {

    String username = request.getParameter("username");
    String password = request.getParameter("password");

    // Basic validation
    if (username == null || username.trim().isEmpty()) {
      request.setAttribute("error", "Username is required");
      request.setAttribute("username", username);
      request.getRequestDispatcher("/register/index.jsp").forward(request, response);
      return;
    }

    if (password == null || password.trim().isEmpty()) {
      request.setAttribute("error", "Password is required");
      request.setAttribute("username", username);
      request.getRequestDispatcher("/register/index.jsp").forward(request, response);
      return;
    }

    // Validate password strength
    String passwordError = PasswordUtil.getPasswordValidationError(password);
    if (passwordError != null) {
      request.setAttribute("error", passwordError);
      request.setAttribute("username", username);
      request.getRequestDispatcher("/register/index.jsp").forward(request, response);
      return;
    }

    // Check if user already exists
    if (userRepository.existsByUsername(username.trim())) {
      request.setAttribute("error", "Username already exists");
      request.setAttribute("username", username);
      request.getRequestDispatcher("/register/index.jsp").forward(request, response);
      return;
    }

    // Hash the password
    String hashedPassword = PasswordUtil.hashPassword(password);

    // Create new user
    User user = new User(username.trim(), hashedPassword);
    User savedUser = userRepository.save(user);

    // Auto-login after registration
    HttpSession session = request.getSession();
    session.setAttribute("user", savedUser);

    // Redirect to home page
    response.sendRedirect("/");
  }

  private void loginUser(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException, ServletException {

    String username = request.getParameter("username");
    String password = request.getParameter("password");

    if (username == null || username.trim().isEmpty()) {
      request.setAttribute("error", "Username is required");
      request.setAttribute("username", username);
      request.getRequestDispatcher("/login/index.jsp").forward(request, response);
      return;
    }

    if (password == null || password.trim().isEmpty()) {
      request.setAttribute("error", "Password is required");
      request.setAttribute("username", username);
      request.getRequestDispatcher("/login/index.jsp").forward(request, response);
      return;
    }

    Optional<User> userOpt = userRepository.findByUsername(username);

    if (userOpt.isPresent() && PasswordUtil.verifyPassword(password, userOpt.get().getHashedPassword())) {
      User user = userOpt.get();

      // Update last login
      user.setLastLogin(LocalDateTime.now());
      userRepository.update(user);

      // Create session
      HttpSession session = request.getSession();
      session.setAttribute("user", user);

      // Redirect to home page
      response.sendRedirect("/");
    } else {
      request.setAttribute("error", "Invalid username or password");
      request.setAttribute("username", username);
      request.getRequestDispatcher("/login/index.jsp").forward(request, response);
    }
  }

  private void updateScore(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      response.setStatus(HttpServletResponse.SC_UNAUTHORIZED);
      response.setContentType("application/json");
      response.getWriter().write("{\"success\": false, \"message\": \"User not logged in\"}");
      return;
    }

    User user = (User) session.getAttribute("user");
    String scoreStr = request.getParameter("score");

    try {
      int newScore = Integer.parseInt(scoreStr);

      if (newScore > user.getHighScore()) {
        user.setHighScore(newScore);
        userRepository.update(user);
        session.setAttribute("user", user); // Update session

        response.setContentType("application/json");
        response.getWriter()
            .write("{\"success\": true, \"message\": \"New high score!\", \"score\": " + newScore + "}");
      } else {
        response.setContentType("application/json");
        response.getWriter().write("{\"success\": true, \"message\": \"Score recorded\", \"score\": " + newScore + "}");
      }
    } catch (NumberFormatException e) {
      response.setStatus(HttpServletResponse.SC_BAD_REQUEST);
      response.setContentType("application/json");
      response.getWriter().write("{\"success\": false, \"message\": \"Invalid score format\"}");
    }
  }

  private void logout(HttpServletRequest request, HttpServletResponse response)
      throws IOException {

    HttpSession session = request.getSession(false);
    if (session != null) {
      session.invalidate();
    }

    response.sendRedirect("/");
  }
}