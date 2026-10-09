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
        public IActionResult Index(string name)
        {
            if (!string.IsNullOrEmpty(name))
            {
                HttpContext.Session.SetString("name", name);
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