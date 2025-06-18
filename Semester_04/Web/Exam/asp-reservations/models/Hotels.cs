using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Hotels
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? HotelName { get; set; }

        [MaxLength(100)]
        public string? Date { get; set; }

        [MaxLength(100)]
        public string? City { get; set; }

        [MaxLength(100)]
        public int? AvailableRooms { get; set; }
    }
} 