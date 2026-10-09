import java.io.IOException;
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

    // Forward to login JSP
    request.getRequestDispatcher("login.jsp").forward(request, response);
  }

  @Override
  protected void doPost(HttpServletRequest request, HttpServletResponse response)
      throws ServletException, IOException {

    String username = request.getParameter("username");
    String errorMessage = "";

    if (username == null || username.trim().isEmpty()) {
      errorMessage = "Please enter a username";
      request.setAttribute("error_message", errorMessage);
      request.getRequestDispatcher("login.jsp").forward(request, response);
    } else {
      // Store username in session
      HttpSession session = request.getSession();
      session.setAttribute("currentUser", username.trim());

      // Redirect to main page
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