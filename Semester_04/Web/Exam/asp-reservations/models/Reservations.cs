using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Reservations
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? Person { get; set; }

        public int? Type { get; set; }

        public int? IdReservedResource { get; set; }
    }
} 