using Microsoft.AspNetCore.Mvc;
using Microsoft.EntityFrameworkCore;
using CarDealershipApi.Data;
using CarDealershipApi.Models;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace CarDealershipApi.Controllers
{
    [Route("api/categories")]
    [ApiController]
    public class CategoriesController : ControllerBase
    {
        private readonly CarDealershipContext _context;

        public CategoriesController(CarDealershipContext context)
        {
            _context = context;
        }

        // GET: api/categories
        [HttpGet]
        public async Task<ActionResult<dynamic>> GetCategories()
        {
            var categories = await _context.Categories.ToListAsync();
            
            // Format response to match PHP API structure
            var response = new
            {
                records = categories.Select(c => new
                {
                    id = c.Id,
                    name = c.Name,
                    description = c.Description
                }).ToList()
            };
            
            return response;
        }
        
        // GET: api/categories/read - PHP style endpoint
        [HttpGet("read")]
        public async Task<ActionResult<dynamic>> ReadCategories()
        {
            return await GetCategories();
        }
    }
} 