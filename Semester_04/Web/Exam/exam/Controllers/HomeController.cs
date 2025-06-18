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

              var uzarid = _context.Users.Where(u => u.Username == name).Select(u => u.Id).SingleOrDefault();        
            
            var products = _context.Products.ToList();
            ViewBag.products = products;

            var myorders = _context.Orders.Where(o => o.userId == uzarid).ToList();
            ViewBag.myorders = myorders;

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
                var productToAdd = _context.Products.FirstOrDefault(p => p.Id == int.Parse(productId));
                if (productToAdd == null)
                {
                    ViewBag.ErrorMessage = "Product not found!";
                }
                else
                {
                    var categoryToAdd = productToAdd.name?.Split('-')[0];
                    
                    var uzarid = _context.Users.Where(u => u.Username == sessionName).Select(u => u.Id).SingleOrDefault();        
                    
                    var lastThreeOrders = _context.Orders
                        .Where(o => o.userId == uzarid)
                        .OrderByDescending(o => o.Id)
                        .Take(3)
                        .ToList();
                    
                    if (lastThreeOrders.Count == 3)
                    {
                        bool categoryInAllThreeOrders = true;
                        
                        foreach (var order in lastThreeOrders)
                        {
                            var productsInOrder = _context.OrderItems
                                .Where(oi => oi.orderId == order.Id)
                                .Join(_context.Products, oi => oi.productId, p => p.Id, (oi, p) => p)
                                .ToList();
                            
                            var categoriesInOrder = productsInOrder
                                .Select(p => p.name?.Split('-')[0])
                                .Where(c => !string.IsNullOrEmpty(c))
                                .Distinct()
                                .ToList();
                            
                            if (!categoriesInOrder.Contains(categoryToAdd))
                            {
                                categoryInAllThreeOrders = false;
                                break;
                            }
                        }
                        
                        if (categoryInAllThreeOrders)
                        {
                            ViewBag.ErrorMessage = $"Cannot add product from category '{categoryToAdd}' - this category exists in your last 3 orders!";
                        }
                        else
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
                        }
                    }
                    else
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
                    }
                }
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

                // var uzarid = _context.Users.Where(u => u.Username == sessionName).Select(u => u.Id).SingleOrDefault();        
                var uzarid = 1;        

                // if (uzarid == 0)
                // {
                //     ViewBag.ErrorMessage = "User not found!";
                //     var products = _context.Products.ToList();
                //     ViewBag.products = products;
                //     return View("Index");
                // }

                var productIds = cart.Select(idStr => int.Parse(idStr)).ToList();
                var productCOS = _context.Products.Where(p => productIds.Contains(p.Id)).ToList();


                double price = productCOS.Sum(p => p.price);

                var categoriilemasi = productCOS
                    .Select(p => p.name?.Split('-')[0])
                    .Where(c => !string.IsNullOrEmpty(c))
                    .ToList();

                bool douamama = categoriilemasi
                    .GroupBy(c => c)
                    .Any(g => g.Count() >= 2);

                if (cart.Count >= 3)
                {
                    price -= 0.10 * price; 
                }

                if (douamama)
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

                // foreach (var product in productCOS)
                // {
                //     var orderItem = new OrderItem
                //     {
                //         orderId = order.Id,
                //         productId = product.Id
                //     };
                //     _context.OrderItems.Add(orderItem);
                // }
                
                _context.SaveChanges(); 

                HttpContext.Session.Remove("cart");
                ViewBag.SuccessMessage = "Order confirmed successfully!";
                                
             }
            else
            {
                ViewBag.ErrorMessage = "Failed to add product to cart.";
            }

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



