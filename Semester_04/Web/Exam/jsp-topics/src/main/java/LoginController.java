import java.io.IOException;
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

    String username = request.getParameter("name");
    HttpSession session = request.getSession();
    String errorMessage = "";
    if (username == null || username.trim().isEmpty()) {
      errorMessage = "Please enter a valid username";
      request.setAttribute("error_message", errorMessage);
      request.getRequestDispatcher("login.jsp").forward(request, response);
      return;
    } else {
      session.setAttribute("currentUser", username.trim());
      response.sendRedirect("main");
    }
  }
}

@WebServlet("/logout")
class LogoutController extends HttpServlet {
  private static final long serialVersionUID = 1L;

  @Override
  protected void doGet(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    HttpSession session = request.getSession();
    session.invalidate();

    response.sendRedirect("login.jsp");
  }
}