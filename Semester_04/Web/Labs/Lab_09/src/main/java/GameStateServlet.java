import model.GameState;
import model.Position;
import model.User;
import repository.GameStateRepository;
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
  private Gson gson;

  @Override
  public void init() throws ServletException {
    gameStateRepository = new GameStateRepository();
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