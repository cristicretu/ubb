using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Orders 
    {
        [Key]
        public int Id { get; set; }

        public int userId { get; set; }

        public double totalPrice { get; set; }
    }
} 