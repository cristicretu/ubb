using Microsoft.AspNetCore.Mvc;

namespace ProjectManagement.Controllers
{
    public class LoginController : Controller
    {
        [HttpGet]
        public IActionResult Index()
        {
            return View();
        }

        [HttpPost]
        public IActionResult Index(string name, string date, string city)
        {
            if (!string.IsNullOrEmpty(name) && !string.IsNullOrEmpty(date) && !string.IsNullOrEmpty(city))
            {
                HttpContext.Session.SetString("Username", name);
                HttpContext.Session.SetString("date", date);
                HttpContext.Session.SetString("city", city);
                return RedirectToAction("Index", "Home");
            }
            
            return View();
        }

        [HttpPost]
        public IActionResult Logout()
        {
            HttpContext.Session.Clear();
            return RedirectToAction("Index", "Login");
        }
    }
} 