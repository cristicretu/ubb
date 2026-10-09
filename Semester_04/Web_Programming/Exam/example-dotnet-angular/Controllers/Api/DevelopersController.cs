using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;
using ProjectManagement.Models;

namespace ProjectManagement.Controllers.Api
{
    [ApiController]
    [Route("api/[controller]")]
    public class DevelopersController : ControllerBase
    {
        private readonly ApplicationDbContext _context;

        public DevelopersController(ApplicationDbContext context)
        {
            _context = context;
        }

        [HttpGet]
        public IActionResult GetDevelopers([FromQuery] string? skill = null)
        {
            var username = HttpContext.Session.GetString("Username");
            if (string.IsNullOrEmpty(username))
            {
                return Unauthorized(new { message = "User not authenticated" });
            }

            var developersQuery = _context.SoftwareDevelopers.AsQueryable();

            if (!string.IsNullOrEmpty(skill))
            {
                developersQuery = developersQuery.Where(d => 
                    !string.IsNullOrEmpty(d.Skills) && 
                    d.Skills.ToLower().Contains(skill.ToLower()));
            }

            var allDevelopers = developersQuery
                .Select(d => new
                {
                    id = d.Id,
                    name = d.Name,
                    skills = d.Skills
                })
                .ToList();

            return Ok(new
            {
                developers = allDevelopers,
                filteredBySkill = !string.IsNullOrEmpty(skill) ? skill : null
            });
        }

        [HttpGet("{id}")]
        public IActionResult GetDeveloper(int id)
        {
            var username = HttpContext.Session.GetString("Username");
            if (string.IsNullOrEmpty(username))
            {
                return Unauthorized(new { message = "User not authenticated" });
            }

            var developer = _context.SoftwareDevelopers
                .Where(d => d.Id == id)
                .Select(d => new
                {
                    id = d.Id,
                    name = d.Name,
                    skills = d.Skills
                })
                .FirstOrDefault();
                
            if (developer == null)
            {
                return NotFound(new { message = "Developer not found" });
            }

            return Ok(developer);
        }
    }
} 