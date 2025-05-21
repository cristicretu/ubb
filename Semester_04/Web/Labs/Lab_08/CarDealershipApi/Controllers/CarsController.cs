using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;
using CarDealershipApi.Data;
using CarDealershipApi.Models;
using System;
using System.Linq;
using System.Threading.Tasks;

namespace CarDealershipApi.Controllers
{
    [Route("api/cars")]
    [ApiController]
    public class CarsController : ControllerBase
    {
        private readonly CarDealershipContext _context;

        public CarsController(CarDealershipContext context)
        {
            _context = context;
        }

        // GET: api/cars
        // GET: api/cars?category_id=1
        [HttpGet]
        public async Task<ActionResult<dynamic>> GetCars([FromQuery] int? category_id)
        {
            try
            {
                IQueryable<Car> query = _context.Cars;
                
                if (category_id.HasValue)
                {
                    query = query.Where(c => c.CategoryId == category_id.Value);
                }
                
                var cars = await query
                    .Select(c => new 
                    {
                        id = c.Id,
                        model = c.Model ?? string.Empty,
                        engine_power = c.EnginePower ?? string.Empty,
                        fuel_type = c.FuelType ?? string.Empty,
                        price = c.Price,
                        color = c.Color ?? string.Empty,
                        year = c.Year,
                        history = c.History ?? string.Empty,
                        category_id = c.CategoryId,
                        features = c.Features ?? string.Empty
                    })
                    .ToListAsync();
                
                var response = new
                {
                    records = cars
                };
                
                return response;
            }
            catch (Exception ex)
            {
                return StatusCode(500, new { error = ex.Message, stack = ex.StackTrace });
            }
        }

        // GET: api/cars/read 
        [HttpGet("read")]
        public async Task<ActionResult<dynamic>> ReadCars([FromQuery] int? category_id)
        {
            return await GetCars(category_id);
        }

        // GET: api/cars/5
        [HttpGet("{id:int}")]
        public async Task<ActionResult<dynamic>> GetCar(int id)
        {
            try
            {
                var car = await _context.Cars
                    .Where(c => c.Id == id)
                    .Select(c => new 
                    {
                        id = c.Id,
                        model = c.Model ?? string.Empty,
                        engine_power = c.EnginePower ?? string.Empty,
                        fuel_type = c.FuelType ?? string.Empty,
                        price = c.Price,
                        color = c.Color ?? string.Empty,
                        year = c.Year,
                        history = c.History ?? string.Empty,
                        category_id = c.CategoryId,
                        features = c.Features ?? string.Empty
                    })
                    .FirstOrDefaultAsync();

                if (car == null)
                {
                    return NotFound();
                }
                
                return car;
            }
            catch (Exception ex)
            {
                return StatusCode(500, new { error = ex.Message, stack = ex.StackTrace });
            }
        }

        // GET: api/cars/read_one?id=5 
        [HttpGet("read_one")]
        public async Task<ActionResult<dynamic>> ReadOneCar([FromQuery] int id)
        {
            try
            {
                var car = await _context.Cars
                    .Where(c => c.Id == id)
                    .Select(c => new 
                    {
                        id = c.Id,
                        model = c.Model ?? string.Empty,
                        engine_power = c.EnginePower ?? string.Empty,
                        fuel_type = c.FuelType ?? string.Empty,
                        price = c.Price,
                        color = c.Color ?? string.Empty,
                        year = c.Year,
                        history = c.History ?? string.Empty,
                        category_id = c.CategoryId,
                        features = c.Features ?? string.Empty
                    })
                    .FirstOrDefaultAsync();

                if (car == null)
                {
                    return NotFound(new { 
                        success = false, 
                        message = "Car not found", 
                        requestedId = id 
                    });
                }
                
                return new { 
                    success = true, 
                    record = car 
                };
            }
            catch (Exception ex)
            {
                return StatusCode(500, new { 
                    success = false, 
                    error = ex.Message, 
                    stack = ex.StackTrace 
                });
            }
        }

        // POST: api/cars
        [HttpPost]
        public async Task<ActionResult<dynamic>> CreateCar([FromBody] Car car)
        {
            if (!ModelState.IsValid)
            {
                return BadRequest(ModelState);
            }

            car.CreatedAt = DateTime.UtcNow;
            _context.Cars.Add(car);
            await _context.SaveChangesAsync();

            return new
            {
                message = "Car was created.",
                id = car.Id
            };
        }

        // POST: api/cars/create (PHP-style endpoint)
        [HttpPost("create")]
        public async Task<ActionResult<dynamic>> CreateCarPhpStyle([FromBody] Car car)
        {
            return await CreateCar(car);
        }

        // PUT: api/cars/5
        [HttpPut("{id}")]
        public async Task<ActionResult<dynamic>> UpdateCar(int id, [FromBody] Car car)
        {
            if (id != car.Id)
            {
                return BadRequest();
            }

            if (!ModelState.IsValid)
            {
                return BadRequest(ModelState);
            }

            var existingCar = await _context.Cars.FindAsync(id);
            if (existingCar == null)
            {
                return NotFound();
            }

            existingCar.Model = car.Model ?? string.Empty;
            existingCar.EnginePower = car.EnginePower ?? string.Empty;
            existingCar.FuelType = car.FuelType ?? string.Empty;
            existingCar.Price = car.Price;
            existingCar.Color = car.Color ?? string.Empty;
            existingCar.Year = car.Year;
            existingCar.History = car.History;
            existingCar.CategoryId = car.CategoryId;
            existingCar.Features = car.Features;

            try
            {
                await _context.SaveChangesAsync();
                return new { message = "Car was updated." };
            }
            catch (DbUpdateConcurrencyException)
            {
                if (!CarExists(id))
                {
                    return NotFound();
                }
                else
                {
                    throw;
                }
            }
        }

        // POST: api/cars/edit 
        [HttpPost("edit")]
        public async Task<ActionResult<dynamic>> EditCarPhpStyle([FromBody] Car car)
        {
            return await UpdateCar(car.Id, car);
        }

        // DELETE: api/cars/5
        [HttpDelete("{id}")]
        public async Task<ActionResult<dynamic>> DeleteCar(int id)
        {
            var car = await _context.Cars.FindAsync(id);
            if (car == null)
            {
                return NotFound();
            }

            _context.Cars.Remove(car);
            await _context.SaveChangesAsync();

            return new { message = "Car was deleted." };
        }

        // POST: api/cars/delete 
        [HttpPost("delete")]
        public async Task<ActionResult<dynamic>> DeleteCarPhpStyle([FromBody] dynamic data)
        {
            int id;
            try
            {
                id = Convert.ToInt32(data.id);
            }
            catch
            {
                return BadRequest(new { message = "Invalid ID format" });
            }
            
            return await DeleteCar(id);
        }

        private bool CarExists(int id)
        {
            return _context.Cars.Any(e => e.Id == id);
        }
    }
} 