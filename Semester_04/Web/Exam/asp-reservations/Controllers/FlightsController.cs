using Microsoft.AspNetCore.Mvc;

namespace ProjectManagement.Controllers
{
    public class FlightsController: Controller
    {
        [HttpGet]
        public IActionResult Index()
        {
            return View();
        }

        [HttpPost]
        public IActionResult Index(string name, string date, string city)
        {
            
            return View();
        }
    }
} 