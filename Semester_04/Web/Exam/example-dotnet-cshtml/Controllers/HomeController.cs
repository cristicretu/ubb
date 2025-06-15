using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;
using ProjectManagement.Models;

namespace ProjectManagement.Controllers
{
    public class HomeController : Controller
    {
        private readonly ApplicationDbContext _context;

        public HomeController(ApplicationDbContext context)
        {
            _context = context;
        }

        public IActionResult Index()
        {
            var projects = _context.Projects.ToList();
            return View(projects);
        }

        public IActionResult Error()
        {
            return View();
        }

    
        public IActionResult Projects()
        {
            var projects = _context.Projects.ToList();
            return View(projects);
        }

        public IActionResult GetUserID(string username)
        {
            var user = _context.Users.FirstOrDefault(u => u.Username == username);
            ViewBag.UserID = user.Id;
            return View();
        }
    }
} 