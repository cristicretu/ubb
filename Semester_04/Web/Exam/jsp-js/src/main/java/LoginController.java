import java.io.IOException;
import java.io.PrintWriter;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;

import com.google.gson.JsonObject;

@WebServlet("/login")
public class LoginController extends HttpServlet {
  private static final long serialVersionUID = 1L;

  @Override
  protected void doGet(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    request.getRequestDispatcher("login.jsp").forward(request, response);
  }

  @Override
  protected void doPost(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    response.setContentType("application/json");
    response.setCharacterEncoding("UTF-8");
    PrintWriter out = response.getWriter();

    String username = request.getParameter("name");
    HttpSession session = request.getSession();

    JsonObject jsonResponse = new JsonObject();

    if (username == null || username.trim().isEmpty()) {
      response.setStatus(HttpServletResponse.SC_BAD_REQUEST);
      jsonResponse.addProperty("success", false);
      jsonResponse.addProperty("error", "Please enter a valid username");
      out.print(jsonResponse.toString());
    } else {
      session.setAttribute("currentUser", username.trim());
      response.setStatus(HttpServletResponse.SC_OK);
      jsonResponse.addProperty("success", true);
      jsonResponse.addProperty("message", "Login successful");
      jsonResponse.addProperty("user", username.trim());
      out.print(jsonResponse.toString());
    }
  }
}

@WebServlet("/logout")
class LogoutController extends HttpServlet {
  private static final long serialVersionUID = 1L;

  @Override
  protected void doGet(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    response.setContentType("application/json");
    response.setCharacterEncoding("UTF-8");
    PrintWriter out = response.getWriter();

    HttpSession session = request.getSession();
    session.invalidate();

    JsonObject jsonResponse = new JsonObject();
    jsonResponse.addProperty("success", true);
    jsonResponse.addProperty("message", "Logout successful");

    response.setStatus(HttpServletResponse.SC_OK);
    out.print(jsonResponse.toString());
  }
}