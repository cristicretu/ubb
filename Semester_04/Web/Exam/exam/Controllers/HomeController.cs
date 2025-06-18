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
                var cartt = HttpContext.Session.GetString("cart");
                
                List<string> cart;
                if (string.IsNullOrEmpty(cartt))
                {
                    cart = new List<string>();
                }
                else
                {
                    cart = cartt.Split(',', System.StringSplitOptions.RemoveEmptyEntries).ToList();
                }
                cart.Add(productId);

                var updatedCartString = string.Join(",", cart);
                HttpContext.Session.SetString("cart", updatedCartString);

                ViewBag.SuccessMessage = "Product added to cart!";
            } else if (action == "confirm_order") {

                var cartt = HttpContext.Session.GetString("cart");

                List<string> cart;
                if (string.IsNullOrEmpty(cartt))
                {
                    cart = new List<string>();
                }
                else
                {
                    cart = cartt.Split(',', System.StringSplitOptions.RemoveEmptyEntries).ToList();
                }

                var uzarid = _context.Users.Where(u => u.Username == sessionName).Select(u => u.Id).SingleOrDefault();        


                var productIds = cart.Select(idStr => int.Parse(idStr)).ToList();
                var productsInCart = _context.Products.Where(p => productIds.Contains(p.Id)).ToList();

                double price = productsInCart.Sum(p => p.price);

                var categoryPrefixes = productsInCart
                    .Select(p => p.name?.Split('-')[0])
                    .Where(c => !string.IsNullOrEmpty(c))
                    .ToList();

                bool twoOfSameCategories = categoryPrefixes
                    .GroupBy(c => c)
                    .Any(g => g.Count() >= 2);

                if (cart.Count >= 3)
                {
                    price -= 0.10 * price; 
                }

                if (twoOfSameCategories)
                {
                    price -= 0.05 * price; 
                }

                var order = new Orders
                {
                    userId = uzarid,
                    totalPrice = price
                };

                _context.Orders.Add(order);
                _context.SaveChanges();

                foreach (var product in productsInCart)
                {
                    var orderItem = new OrderItem
                    {
                        orderId = order.Id,
                        productId = product.Id
                    };
                    _context.OrderItems.Add(orderItem);
                }

                _context.SaveChanges();

                HttpContext.Session.Remove("cart");
                                
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



