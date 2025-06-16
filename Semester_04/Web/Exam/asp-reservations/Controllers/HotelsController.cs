using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;
using ProjectManagement.Models;

namespace ProjectManagement.Controllers
{
    public class HotelsController: Controller
    {
        private readonly ApplicationDbContext _context;

        public HotelsController(ApplicationDbContext context)
        {
            _context = context;
        }
        [HttpGet]
        public IActionResult Index()
        {
            var date = HttpContext.Session.GetString("date");
            var city = HttpContext.Session.GetString("city");
            var hotels = _context.Hotels.Where(p => p.Date == date && p.City == city && p.AvailableRooms > 0).ToList();

            ViewBag.Hotels = hotels;
            ViewBag.Date = date;
            ViewBag.City = city;
            return View();
        }

        [HttpPost]
        public IActionResult Reserve(int hotelId)
        {
            var hotel = _context.Hotels.Find(hotelId);
            if (hotel == null)
            {
                return NotFound();
            }

            var reservation = new Reservations
            {
                IdReservedResource = hotelId,
                Type = "hotel",
                Person = HttpContext.Session.GetString("name")
            };
            _context.Reservations.Add(reservation);

            hotel.AvailableRooms--;
            _context.SaveChanges();

            return RedirectToAction("Index");
        }
    }
} 