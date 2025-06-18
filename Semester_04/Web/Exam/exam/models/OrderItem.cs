using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class OrderItem 
    {
        [Key]
        public int Id { get; set; }

        public int orderId { get; set; }

        public int productId { get; set; }
    }
} 