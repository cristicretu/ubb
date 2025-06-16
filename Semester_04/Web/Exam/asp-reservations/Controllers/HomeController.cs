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
            return View();
            // var username = HttpContext.Session.GetString("Username");
            // if (string.IsNullOrEmpty(username))
            // {
            //     return Redirect("/Login/Index");
            // }

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
        public IActionResult Index(string action, string project_name, string project_manager_name)
        {
            return View();
            // var username = HttpContext.Session.GetString("Username");
            // if (string.IsNullOrEmpty(username))
            // {
            //     return Redirect("/Login/Index");
            // }

            // string successMessage = "";
            // string errorMessage = "";

            // if (action == "assign_project")
            // {
            //     if (!string.IsNullOrEmpty(project_name) && !string.IsNullOrEmpty(project_manager_name))
            //     {
            //         var projectManager = _context.SoftwareDevelopers.FirstOrDefault(s => s.Name == project_manager_name);
            //         if (projectManager != null)
            //         {
            //             var newProject = new Project
            //             {
            //                 Name = project_name,
            //                 ProjectManagerID = projectManager.Id,
            //                 Description = "",
            //                 Members = ""
            //             };

            //             _context.Projects.Add(newProject);
            //             var result = _context.SaveChanges();
                        
            //             if (result > 0)
            //             {
            //                 successMessage = $"Project '{project_name}' assigned to '{project_manager_name}' successfully!";
            //             }
            //             else
            //             {
            //                 errorMessage = "Failed to assign project.";
            //             }
            //         }
            //         else
            //         {
            //             errorMessage = $"Developer '{project_manager_name}' not found!";
            //         }
            //     }
            //     else
            //     {
            //         errorMessage = "Please fill in all fields.";
            //     }
            // }

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
            // ViewBag.SuccessMessage = successMessage;
            // ViewBag.ErrorMessage = errorMessage;

            // return View(allProjects);
        }

        public IActionResult Error()
        {
            return View();
        }

        public IActionResult Flights()
        {
            var projects = _context.Flights.ToList();
            return View(projects);
        }

    }
} 