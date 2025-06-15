using Microsoft.AspNetCore.Mvc;

namespace ProjectManagement.Controllers
{
    public class StaticController : Controller
    {
        public IActionResult Index()
        {
            return PhysicalFile(Path.Combine(Directory.GetCurrentDirectory(), "wwwroot", "index.html"), "text/html");
        }
    }
} 