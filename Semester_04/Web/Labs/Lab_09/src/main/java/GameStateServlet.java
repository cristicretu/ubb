import model.GameState;
import model.Position;
import model.User;
import model.Move;
import repository.GameStateRepository;
import repository.MoveRepository;
import repository.UserRepository;
import com.google.gson.Gson;
import com.google.gson.JsonSyntaxException;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;
import java.io.IOException;
import java.io.BufferedReader;
import java.sql.SQLException;
import java.util.List;
import java.util.Optional;

@WebServlet("/gamestate")
public class GameStateServlet extends HttpServlet {
  private GameStateRepository gameStateRepository;
  private MoveRepository moveRepository;
  private UserRepository userRepository;
  private Gson gson;

  @Override
  public void init() throws ServletException {
    gameStateRepository = new GameStateRepository();
    moveRepository = new MoveRepository();
    userRepository = new UserRepository();
    gson = new Gson();
  }

  @Override
  protected void doGet(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    String action = request.getParameter("action");

    try {
      switch (action != null ? action : "") {
        case "get":
          getGameState(request, response);
          break;
        case "getLatest":
          getLatestGameState(request, response);
          break;
        case "getAll":
          getAllGameStates(request, response);
          break;
        default:
          response.sendError(HttpServletResponse.SC_BAD_REQUEST, "Invalid action");
          break;
      }
    } catch (SQLException e) {
      throw new ServletException("Database error", e);
    }
  }

  @Override
  protected void doPost(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    String action = request.getParameter("action");

    try {
      switch (action != null ? action : "") {
        case "save":
          saveGameState(request, response);
          break;
        case "update":
          updateGameState(request, response);
          break;
        case "start":
          startNewGame(request, response);
          break;
        case "move":
          moveSnake(request, response);
          break;
        case "reset":
          resetGame(request, response);
          break;
        case "end":
          endGame(request, response);
          break;
        case "autoMove":
          autoMoveSnake(request, response);
          break;
        default:
          response.sendError(HttpServletResponse.SC_BAD_REQUEST, "Invalid action");
          break;
      }
    } catch (SQLException e) {
      throw new ServletException("Database error", e);
    }
  }

  @Override
  protected void doDelete(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    String action = request.getParameter("action");

    try {
      switch (action != null ? action : "") {
        case "delete":
          deleteGameState(request, response);
          break;
        case "deleteAll":
          deleteAllGameStates(request, response);
          break;
        default:
          response.sendError(HttpServletResponse.SC_BAD_REQUEST, "Invalid action");
          break;
      }
    } catch (SQLException e) {
      throw new ServletException("Database error", e);
    }
  }

  private void saveGameState(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      sendErrorResponse(response, HttpServletResponse.SC_UNAUTHORIZED, "User not logged in");
      return;
    }

    User user = (User) session.getAttribute("user");

    try {
      // Read JSON from request body
      StringBuilder jsonBuffer = new StringBuilder();
      String line;
      try (BufferedReader reader = request.getReader()) {
        while ((line = reader.readLine()) != null) {
          jsonBuffer.append(line);
        }
      }

      GameState gameState = gson.fromJson(jsonBuffer.toString(), GameState.class);

      if (gameState == null) {
        sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Invalid game state data");
        return;
      }

      // Set the user ID from the session
      gameState.setUserId(user.getId());

      GameState savedGameState = gameStateRepository.save(gameState);
      sendSuccessResponse(response, savedGameState, "Game state saved successfully");

    } catch (JsonSyntaxException e) {
      sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Invalid JSON format");
    }
  }

  private void updateGameState(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      sendErrorResponse(response, HttpServletResponse.SC_UNAUTHORIZED, "User not logged in");
      return;
    }

    User user = (User) session.getAttribute("user");

    try {
      // Read JSON from request body
      StringBuilder jsonBuffer = new StringBuilder();
      String line;
      try (BufferedReader reader = request.getReader()) {
        while ((line = reader.readLine()) != null) {
          jsonBuffer.append(line);
        }
      }

      GameState gameState = gson.fromJson(jsonBuffer.toString(), GameState.class);

      if (gameState == null || gameState.getId() == null) {
        sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Invalid game state data or missing ID");
        return;
      }

      // Verify the game state belongs to the current user
      Optional<GameState> existingGameState = gameStateRepository.findById(gameState.getId());
      if (existingGameState.isEmpty() || !existingGameState.get().getUserId().equals(user.getId())) {
        sendErrorResponse(response, HttpServletResponse.SC_FORBIDDEN, "Game state not found or access denied");
        return;
      }

      // Set the user ID from the session
      gameState.setUserId(user.getId());

      GameState updatedGameState = gameStateRepository.update(gameState);
      sendSuccessResponse(response, updatedGameState, "Game state updated successfully");

    } catch (JsonSyntaxException e) {
      sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Invalid JSON format");
    }
  }

  private void getGameState(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      sendErrorResponse(response, HttpServletResponse.SC_UNAUTHORIZED, "User not logged in");
      return;
    }

    User user = (User) session.getAttribute("user");
    String idStr = request.getParameter("id");

    if (idStr == null) {
      sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Game state ID is required");
      return;
    }

    try {
      Long id = Long.parseLong(idStr);
      Optional<GameState> gameState = gameStateRepository.findById(id);

      if (gameState.isEmpty() || !gameState.get().getUserId().equals(user.getId())) {
        sendErrorResponse(response, HttpServletResponse.SC_NOT_FOUND, "Game state not found");
        return;
      }

      sendSuccessResponse(response, gameState.get(), "Game state retrieved successfully");

    } catch (NumberFormatException e) {
      sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Invalid game state ID format");
    }
  }

  private void getLatestGameState(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      sendErrorResponse(response, HttpServletResponse.SC_UNAUTHORIZED, "User not logged in");
      return;
    }

    User user = (User) session.getAttribute("user");
    Optional<GameState> gameState = gameStateRepository.findLatestByUserId(user.getId());

    if (gameState.isEmpty()) {
      sendErrorResponse(response, HttpServletResponse.SC_NOT_FOUND, "No game state found");
      return;
    }

    sendSuccessResponse(response, gameState.get(), "Latest game state retrieved successfully");
  }

  private void getAllGameStates(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      sendErrorResponse(response, HttpServletResponse.SC_UNAUTHORIZED, "User not logged in");
      return;
    }

    User user = (User) session.getAttribute("user");
    List<GameState> gameStates = gameStateRepository.findByUserId(user.getId());

    sendSuccessResponse(response, gameStates, "Game states retrieved successfully");
  }

  private void deleteGameState(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      sendErrorResponse(response, HttpServletResponse.SC_UNAUTHORIZED, "User not logged in");
      return;
    }

    User user = (User) session.getAttribute("user");
    String idStr = request.getParameter("id");

    if (idStr == null) {
      sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Game state ID is required");
      return;
    }

    try {
      Long id = Long.parseLong(idStr);

      // Verify the game state belongs to the current user
      Optional<GameState> existingGameState = gameStateRepository.findById(id);
      if (existingGameState.isEmpty() || !existingGameState.get().getUserId().equals(user.getId())) {
        sendErrorResponse(response, HttpServletResponse.SC_FORBIDDEN, "Game state not found or access denied");
        return;
      }

      boolean deleted = gameStateRepository.deleteById(id);

      if (deleted) {
        sendSuccessResponse(response, null, "Game state deleted successfully");
      } else {
        sendErrorResponse(response, HttpServletResponse.SC_INTERNAL_SERVER_ERROR, "Failed to delete game state");
      }

    } catch (NumberFormatException e) {
      sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Invalid game state ID format");
    }
  }

  private void deleteAllGameStates(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      sendErrorResponse(response, HttpServletResponse.SC_UNAUTHORIZED, "User not logged in");
      return;
    }

    User user = (User) session.getAttribute("user");
    boolean deleted = gameStateRepository.deleteByUserId(user.getId());

    if (deleted) {
      sendSuccessResponse(response, null, "All game states deleted successfully");
    } else {
      sendSuccessResponse(response, null, "No game states to delete");
    }
  }

  private void startNewGame(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      response.sendRedirect("/login/index.jsp");
      return;
    }

    User user = (User) session.getAttribute("user");

    // Create initial game state
    GameState gameState = new GameState();
    gameState.setUserId(user.getId());
    gameState.setScore(0);

    // Initialize snake in the center
    Position[] snake = new Position[3];
    snake[0] = new Position(10, 10); // Head
    snake[1] = new Position(9, 10);
    snake[2] = new Position(8, 10); // Tail
    gameState.setSnake(snake);

    // Set initial direction
    gameState.setCurrentDirection("RIGHT");

    // Place random apple
    gameState.setApple(generateRandomApple(snake, null));

    // Create some obstacles
    gameState.setObstacles(generateObstacles());

    // Store start time in session
    session.setAttribute("gameStartTime", System.currentTimeMillis());

    gameStateRepository.save(gameState);
    response.sendRedirect("/");
  }

  private void moveSnake(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      response.sendRedirect("/login/index.jsp");
      return;
    }

    User user = (User) session.getAttribute("user");
    String direction = request.getParameter("direction");

    if (direction == null) {
      response.sendRedirect("/");
      return;
    }

    // Get current game state
    Optional<GameState> gameOpt = gameStateRepository.findLatestByUserId(user.getId());
    if (gameOpt.isEmpty()) {
      response.sendRedirect("/");
      return;
    }

    GameState gameState = gameOpt.get();
    Position[] currentSnake = gameState.getSnake();

    if (currentSnake == null || currentSnake.length == 0) {
      response.sendRedirect("/");
      return;
    }

    // Calculate new head position
    Position currentHead = currentSnake[0];
    Position newHead = calculateNewHead(currentHead, direction);

    // Check for collisions
    if (isCollision(newHead, currentSnake, gameState.getObstacles())) {
      // Game over - redirect to show final state
      session.setAttribute("gameMessage", "Game Over! You hit something!");
      response.sendRedirect("/");
      return;
    }

    // Check if apple is eaten
    boolean ateApple = newHead.getX() == gameState.getApple().getX() &&
        newHead.getY() == gameState.getApple().getY();

    // Create new snake
    Position[] newSnake;
    if (ateApple) {
      // Grow snake
      newSnake = new Position[currentSnake.length + 1];
      newSnake[0] = newHead;
      System.arraycopy(currentSnake, 0, newSnake, 1, currentSnake.length);

      // Increase score
      gameState.setScore(gameState.getScore() + 10);

      // Generate new apple
      gameState.setApple(generateRandomApple(newSnake, gameState.getObstacles()));
    } else {
      // Move snake (same length)
      newSnake = new Position[currentSnake.length];
      newSnake[0] = newHead;
      System.arraycopy(currentSnake, 0, newSnake, 1, currentSnake.length - 1);
    }

    gameState.setSnake(newSnake);

    // Update current direction
    gameState.setCurrentDirection(direction);

    // Create and save the move record
    Move move = new Move(
        gameState.getId(),
        user.getId(),
        direction,
        currentHead.getX(),
        currentHead.getY(),
        newHead.getX(),
        newHead.getY(),
        ateApple,
        gameState.getScore());
    moveRepository.save(move);

    // Save the updated game state to database
    gameStateRepository.update(gameState);

    response.sendRedirect("/");
  }

  private void resetGame(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      response.sendRedirect("/login/index.jsp");
      return;
    }

    User user = (User) session.getAttribute("user");

    // Delete current game
    gameStateRepository.deleteByUserId(user.getId());

    // Clear any game messages
    session.removeAttribute("gameMessage");

    response.sendRedirect("/");
  }

  private void endGame(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      response.sendRedirect("/login/index.jsp");
      return;
    }

    User user = (User) session.getAttribute("user");

    // Get current game to check final score
    Optional<GameState> gameOpt = gameStateRepository.findLatestByUserId(user.getId());
    if (gameOpt.isPresent()) {
      GameState gameState = gameOpt.get();

      // Update high score if necessary
      if (gameState.getScore() > user.getHighScore()) {
        user.setHighScore(gameState.getScore());
        userRepository.update(user); // Save to database
        session.setAttribute("user", user);
        session.setAttribute("gameMessage", "New High Score: " + gameState.getScore() + "!");
      } else {
        session.setAttribute("gameMessage", "Game ended. Final score: " + gameState.getScore());
      }
    }

    // Delete current game
    gameStateRepository.deleteByUserId(user.getId());

    response.sendRedirect("/");
  }

  private Position calculateNewHead(Position currentHead, String direction) {
    int newX = currentHead.getX();
    int newY = currentHead.getY();

    switch (direction.toUpperCase()) {
      case "UP":
        newY--;
        break;
      case "DOWN":
        newY++;
        break;
      case "LEFT":
        newX--;
        break;
      case "RIGHT":
        newX++;
        break;
    }

    return new Position(newX, newY);
  }

  private boolean isCollision(Position newHead, Position[] snake, Position[] obstacles) {
    final int GRID_SIZE = 20;

    // Check wall collision
    if (newHead.getX() < 0 || newHead.getX() >= GRID_SIZE ||
        newHead.getY() < 0 || newHead.getY() >= GRID_SIZE) {
      return true;
    }

    // Check self collision
    for (Position segment : snake) {
      if (newHead.getX() == segment.getX() && newHead.getY() == segment.getY()) {
        return true;
      }
    }

    // Check obstacle collision
    if (obstacles != null) {
      for (Position obstacle : obstacles) {
        if (newHead.getX() == obstacle.getX() && newHead.getY() == obstacle.getY()) {
          return true;
        }
      }
    }

    return false;
  }

  private Position generateRandomApple(Position[] snake, Position[] obstacles) {
    final int GRID_SIZE = 20;
    java.util.Random random = new java.util.Random();

    Position apple;
    boolean validPosition;

    do {
      apple = new Position(random.nextInt(GRID_SIZE), random.nextInt(GRID_SIZE));
      validPosition = true;

      // Check if apple position conflicts with snake
      for (Position segment : snake) {
        if (apple.getX() == segment.getX() && apple.getY() == segment.getY()) {
          validPosition = false;
          break;
        }
      }

      // Check if apple position conflicts with obstacles
      if (validPosition && obstacles != null) {
        for (Position obstacle : obstacles) {
          if (apple.getX() == obstacle.getX() && apple.getY() == obstacle.getY()) {
            validPosition = false;
            break;
          }
        }
      }
    } while (!validPosition);

    return apple;
  }

  private Position[] generateObstacles() {
    final int GRID_SIZE = 20;
    java.util.Random random = new java.util.Random();

    // Create some random obstacles
    Position[] obstacles = new Position[8];
    obstacles[0] = new Position(5, 5);
    obstacles[1] = new Position(15, 5);
    obstacles[2] = new Position(5, 15);
    obstacles[3] = new Position(15, 15);
    obstacles[4] = new Position(10, 3);
    obstacles[5] = new Position(3, 10);
    obstacles[6] = new Position(17, 10);
    obstacles[7] = new Position(10, 17);

    return obstacles;
  }

  private void autoMoveSnake(HttpServletRequest request, HttpServletResponse response)
      throws SQLException, IOException {

    HttpSession session = request.getSession(false);
    if (session == null || session.getAttribute("user") == null) {
      sendErrorResponse(response, HttpServletResponse.SC_UNAUTHORIZED, "User not logged in");
      return;
    }

    User user = (User) session.getAttribute("user");

    // Get current game state
    Optional<GameState> gameOpt = gameStateRepository.findLatestByUserId(user.getId());
    if (gameOpt.isEmpty()) {
      sendErrorResponse(response, HttpServletResponse.SC_NOT_FOUND, "No active game");
      return;
    }

    GameState gameState = gameOpt.get();
    String currentDirection = gameState.getCurrentDirection();

    if (currentDirection == null) {
      currentDirection = "RIGHT"; // Default direction
      gameState.setCurrentDirection(currentDirection);
    }

    Position[] currentSnake = gameState.getSnake();

    if (currentSnake == null || currentSnake.length == 0) {
      sendErrorResponse(response, HttpServletResponse.SC_BAD_REQUEST, "Invalid game state");
      return;
    }

    // Calculate new head position using current direction
    Position currentHead = currentSnake[0];
    Position newHead = calculateNewHead(currentHead, currentDirection);

    // Check for collisions
    if (isCollision(newHead, currentSnake, gameState.getObstacles())) {
      // Game over
      sendErrorResponse(response, HttpServletResponse.SC_OK, "Game Over");
      return;
    }

    // Check if apple is eaten
    boolean ateApple = newHead.getX() == gameState.getApple().getX() &&
        newHead.getY() == gameState.getApple().getY();

    // Create new snake
    Position[] newSnake;
    if (ateApple) {
      // Grow snake
      newSnake = new Position[currentSnake.length + 1];
      newSnake[0] = newHead;
      System.arraycopy(currentSnake, 0, newSnake, 1, currentSnake.length);

      // Increase score
      gameState.setScore(gameState.getScore() + 10);

      // Generate new apple
      gameState.setApple(generateRandomApple(newSnake, gameState.getObstacles()));
    } else {
      // Move snake (same length)
      newSnake = new Position[currentSnake.length];
      newSnake[0] = newHead;
      System.arraycopy(currentSnake, 0, newSnake, 1, currentSnake.length - 1);
    }

    gameState.setSnake(newSnake);

    // Create and save the move record
    Move move = new Move(
        gameState.getId(),
        user.getId(),
        currentDirection,
        currentHead.getX(),
        currentHead.getY(),
        newHead.getX(),
        newHead.getY(),
        ateApple,
        gameState.getScore());
    moveRepository.save(move);

    // Save the updated game state to database
    gameStateRepository.update(gameState);

    // Return JSON response for AJAX
    sendSuccessResponse(response, gameState, "Move completed");
  }

  private void sendSuccessResponse(HttpServletResponse response, Object data, String message) throws IOException {
    response.setContentType("application/json");
    response.setCharacterEncoding("UTF-8");

    String jsonResponse;
    if (data != null) {
      jsonResponse = String.format("{\"success\": true, \"message\": \"%s\", \"data\": %s}",
          message, gson.toJson(data));
    } else {
      jsonResponse = String.format("{\"success\": true, \"message\": \"%s\"}", message);
    }

    response.getWriter().write(jsonResponse);
  }

  private void sendErrorResponse(HttpServletResponse response, int statusCode, String message) throws IOException {
    response.setStatus(statusCode);
    response.setContentType("application/json");
    response.setCharacterEncoding("UTF-8");

    String jsonResponse = String.format("{\"success\": false, \"message\": \"%s\"}", message);
    response.getWriter().write(jsonResponse);
  }
}