using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;
using ProjectManagement.Models;

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

        [HttpPost]
        public IActionResult Reserve(int flightId)
        {
            var flight = _context.Flights.Find(flightId);
            if (flight == null)
            {
                return NotFound();
            }

            var reservation = new Reservations
            {
                IdReservedResource = flightId,
                Type = "flight",
                Person = HttpContext.Session.GetString("name")
            };
            _context.Reservations.Add(reservation);

            var operation = "flight:" + flightId + "." + HttpContext.Session.GetString("name") + ";";
            var oldOperation = HttpContext.Session.GetString("operation");
            if (oldOperation != null)
            {
                operation = oldOperation + operation;
            } else {
                operation = operation;
            }
            HttpContext.Session.SetString("operation", operation);

            flight.AvailableSeats--;
            _context.SaveChanges();

            return RedirectToAction("Index");
        }
    }
} 