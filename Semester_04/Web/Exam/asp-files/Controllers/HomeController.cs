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
            var name = HttpContext.Session.GetString("name");
            if (string.IsNullOrEmpty(name))
            {
                return Redirect("/Login/Index");
            }

          
            return View();

            // // Get current user
            // var currentUser = _context.SoftwareDevelopers.FirstOrDefault(u => u.Name == username);
            // int? currentUserID = currentUser?.Id;

            // // Get all projects
            // var allProjects = _context.Projects.ToList();

            // // Get projects managed by current user
            // var yourProjects = currentUserID.HasValue 
            //     ? _context.Projects.Where(p => p.ProjectManagerID == currentUserID.Value).ToList() 
            //     : new List<Project>();

            // // Get projects where current user is a member
            // var memberProjects = allProjects.Where(p => 
            //     !string.IsNullOrEmpty(p.Members) && p.Members.Contains(username)).ToList();

            // // Get all developers
            // var allDevelopers = _context.SoftwareDevelopers.ToList();

            // // Pass data to view
            // ViewBag.Username = username;
            // ViewBag.UserID = currentUserID;
            // ViewBag.AllProjects = allProjects;
            // ViewBag.YourProjects = yourProjects;
            // ViewBag.MemberProjects = memberProjects;
            // ViewBag.AllDevelopers = allDevelopers;

            // return View(allProjects);
        }


        [HttpPost]
        public IActionResult Index(string action, string name, string channel_name)
        {
            var sessionName = HttpContext.Session.GetString("name");
            if (string.IsNullOrEmpty(sessionName))
            {
                return Redirect("/Login/Index");
            }

            return View();

            // return View(allProjects);
        }

        public IActionResult Error()
        {
            return View();
        }

    }
} 