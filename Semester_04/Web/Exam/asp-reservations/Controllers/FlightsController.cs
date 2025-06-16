using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;

namespace ProjectManagement.Controllers
{
    public class FlightsController: Controller
    {
        private readonly ApplicationDbContext _context;

        public FlightsController(ApplicationDbContext context)
        {
            _context = context;
        }
        [HttpGet]
        public IActionResult Index()
        {
            var date = HttpContext.Session.GetString("date");
            var city = HttpContext.Session.GetString("city");
            var flights = _context.Flights.Where(p => p.Date == date && p.DestinationCity == city && p.AvailableSeats > 0).ToList();

            ViewBag.Flights = flights;
            ViewBag.Date = date;
            ViewBag.City = city;
            return View();
        }


    }
} 