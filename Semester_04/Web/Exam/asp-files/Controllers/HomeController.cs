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

        [HttpGet]
        public IActionResult Index()
        {
            var username = HttpContext.Session.GetString("username");
             if (string.IsNullOrEmpty(username))
            {
                return Redirect("/Login/Index");
            }
            var userId = HttpContext.Session.GetString("id"); 
            int id = int.Parse(userId);

            var files = getPaginatedFiles(0, id);
            ViewBag.Files = files;
            ViewBag.offset = 0;
          
            return View();
        }

        public List<ProjectManagement.Models.File> getPaginatedFiles(int offset, int userId) {
            var files = _context.Files.Where(f => f.UserId == userId)
            .Skip(offset).Take(5).ToList();

            return files;
        }


        [HttpPost]
        public IActionResult Index(string action, int offset)
        {
            var sessionName = HttpContext.Session.GetString("username");
            var userId = HttpContext.Session.GetString("id"); 
            int id = int.Parse(userId);

            if (string.IsNullOrEmpty(sessionName))
            {
                return Redirect("/Login/Index");
            }
             
             int newOffset = offset;

            if (action == "back") {
                newOffset -= 5;
            } else if (action == "next") {
                newOffset += 5;
            }

            var files = getPaginatedFiles(newOffset, id);
            ViewBag.Files = files;
            ViewBag.offset = newOffset;

            return View();
        }

        public IActionResult Error()
        {
            return View();
        }

    }
} 