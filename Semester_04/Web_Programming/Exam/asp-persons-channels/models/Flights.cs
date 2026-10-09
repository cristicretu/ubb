using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Flights
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? Date { get; set; }

        [MaxLength(100)]
        public string? DestinationCity { get; set; }

        [MaxLength(100)]
        public int? AvailableSeats { get; set; }
    }
} 