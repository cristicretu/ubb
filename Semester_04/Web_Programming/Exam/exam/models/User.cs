using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class User 
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? Username { get; set; }
    }
} 