using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;
using ProjectManagement.Models;

namespace ProjectManagement.Controllers.Api
{
    [ApiController]
    [Route("api/[controller]")]
    public class ProjectsController : ControllerBase
    {
        private readonly ApplicationDbContext _context;

        public ProjectsController(ApplicationDbContext context)
        {
            _context = context;
        }

        [HttpGet]
        public IActionResult GetProjects()
        {
            var username = HttpContext.Session.GetString("Username");
            if (string.IsNullOrEmpty(username))
            {
                return Unauthorized(new { message = "User not authenticated" });
            }

            var currentUser = _context.SoftwareDevelopers.FirstOrDefault(u => u.Name == username);
            int? currentUserID = currentUser?.Id;

            var allProjects = _context.Projects.ToList();

            var yourProjects = currentUserID.HasValue 
                ? _context.Projects.Where(p => p.ProjectManagerID == currentUserID.Value).ToList() 
                : new List<Project>();

            var memberProjects = allProjects.Where(p => 
                !string.IsNullOrEmpty(p.Members) && p.Members.Contains(username)).ToList();

            return Ok(new
            {
                username = username,
                userId = currentUserID,
                allProjects = allProjects,
                yourProjects = yourProjects,
                memberProjects = memberProjects
            });
        }

        [HttpPost]
        public IActionResult CreateProject([FromBody] CreateProjectRequest request)
        {
            var username = HttpContext.Session.GetString("Username");
            if (string.IsNullOrEmpty(username))
            {
                return Unauthorized(new { message = "User not authenticated" });
            }

            if (string.IsNullOrEmpty(request.ProjectName) || string.IsNullOrEmpty(request.ProjectManagerName))
            {
                return BadRequest(new { message = "Please fill in all fields." });
            }

            var projectManager = _context.SoftwareDevelopers.FirstOrDefault(s => s.Name == request.ProjectManagerName);
            if (projectManager == null)
            {
                return BadRequest(new { message = $"Developer '{request.ProjectManagerName}' not found!" });
            }

            var newProject = new Project
            {
                Name = request.ProjectName,
                ProjectManagerID = projectManager.Id,
                Description = "",
                Members = ""
            };

            _context.Projects.Add(newProject);
            var result = _context.SaveChanges();
            
            if (result > 0)
            {
                return Ok(new { 
                    message = $"Project '{request.ProjectName}' assigned to '{request.ProjectManagerName}' successfully!",
                    project = newProject
                });
            }
            else
            {
                return BadRequest(new { message = "Failed to assign project." });
            }
        }
    }

    public class CreateProjectRequest
    {
        public string ProjectName { get; set; } = "";
        public string ProjectManagerName { get; set; } = "";
    }
} 