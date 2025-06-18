using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;

namespace ProjectManagement.Controllers
{
    public class LoginController : Controller
    {
        private readonly ApplicationDbContext _context;

        public LoginController(ApplicationDbContext context)
        {
            _context = context;
        }

        [HttpGet]
        public IActionResult Index()
        {
            return View();
        }

        [HttpPost]
        public IActionResult Index(string username, string password)
        {
            if (!string.IsNullOrEmpty(username) && !string.IsNullOrEmpty(password))
            {
                var user = _context.Users.FirstOrDefault(u => u.Username== username);

                if (user != null) {
                    if (user.Password == password) {
                        HttpContext.Session.SetString("username", username);
                        HttpContext.Session.SetString("id", user.Id.ToString());
                        return RedirectToAction("Index", "Home");
                    } else {
                        ViewBag.ErrorMessage = "Invalid password";
                    }
                } else {
                    ViewBag.ErrorMessage = "User not found";
                }
            } else {
                ViewBag.ErrorMessage = "Please enter a name and password";
            }
            
            return View();
        }
    }
} 