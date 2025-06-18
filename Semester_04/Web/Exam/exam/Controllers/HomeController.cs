using Microsoft.AspNetCore.Mvc;
using ProjectManagement.Data;
using ProjectManagement.Models;
using System.Linq; 



namespace ProjectManagement.Controllers
{
    public class HomeController : Controller
    {
        private readonly ApplicationDbContext _context;

        public HomeController(ApplicationDbContext context)
        {
            _context = context;
        }

        [HttpGet]
        public IActionResult Index()
        {
            var name = HttpContext.Session.GetString("name");
            if (string.IsNullOrEmpty(name))
            {
                return Redirect("/Login/Index");
            }
            
            var products = _context.Products.ToList();
            ViewBag.products = products;

            return View();
        }


        [HttpPost]
        [ActionName("Index")]
        public IActionResult IndexPost(string action, string productId, string messi)
        {
            var sessionName = HttpContext.Session.GetString("name");
            if (string.IsNullOrEmpty(sessionName))
            {
                return Redirect("/Login/Index");
            }

            if (action == "add_to_cart" && !string.IsNullOrEmpty(productId))
            {
                var cartString = HttpContext.Session.GetString("cart");
                
                List<string> cart;
                if (string.IsNullOrEmpty(cartString))
                {
                    cart = new List<string>();
                }
                else
                {
                    cart = cartString.Split(',', System.StringSplitOptions.RemoveEmptyEntries).ToList();
                }

                // Add product ID to cart
                cart.Add(productId);

                // Save updated cart back to session
                var updatedCartString = string.Join(",", cart);
                HttpContext.Session.SetString("cart", updatedCartString);

                ViewBag.SuccessMessage = "Product added to cart!";
            }
            else
            {
                ViewBag.ErrorMessage = "Failed to add product to cart.";
            }

            // Reload products for the view
            var products = _context.Products.ToList();
            ViewBag.products = products;

            return View("Index");
        }


        public IActionResult Error()
        {
            return View();
        }

    }
} 



