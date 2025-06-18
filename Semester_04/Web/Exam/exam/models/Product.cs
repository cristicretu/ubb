using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Product 
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string name { get; set; }

        [MaxLength(100)]
        public double price { get; set; }
    }
} 