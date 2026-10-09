using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;
using ProjectManagement.Models;

namespace ProjectManagement.Controllers.Api
{
    [ApiController]
    [Route("api/[controller]")]
    public class AuthController : ControllerBase
    {
        private readonly ApplicationDbContext _context;

        public AuthController(ApplicationDbContext context)
        {
            _context = context;
        }

        [HttpPost("login")]
        public IActionResult Login([FromBody] LoginRequest request)
        {
            if (string.IsNullOrEmpty(request.Username))
            {
                return BadRequest(new { message = "Username is required" });
            }

            var user = _context.SoftwareDevelopers.FirstOrDefault(u => u.Name == request.Username);
            if (user == null)
            {
                return BadRequest(new { message = "User not found" });
            }

            HttpContext.Session.SetString("Username", request.Username);
            HttpContext.Session.SetInt32("UserId", user.Id);

            // Debug logging
            Console.WriteLine($"Session set for user: {request.Username}, Session ID: {HttpContext.Session.Id}");

            return Ok(new { 
                message = "Login successful", 
                username = request.Username,
                userId = user.Id 
            });
        }

        [HttpPost("logout")]
        public IActionResult Logout()
        {
            HttpContext.Session.Clear();
            return Ok(new { message = "Logout successful" });
        }

        [HttpGet("status")]
        public IActionResult Status()
        {
            var username = HttpContext.Session.GetString("Username");
            
            // Debug logging
            Console.WriteLine($"Auth status check - Session ID: {HttpContext.Session.Id}, Username: {username ?? "null"}");
            
            if (string.IsNullOrEmpty(username))
            {
                return Ok(new { isAuthenticated = false, debug = $"SessionId: {HttpContext.Session.Id}" });
            }

            var user = _context.SoftwareDevelopers.FirstOrDefault(u => u.Name == username);
            return Ok(new { 
                isAuthenticated = true, 
                username = username,
                userId = user?.Id 
            });
        }
    }

    public class LoginRequest
    {
        public string Username { get; set; } = "";
    }
} 