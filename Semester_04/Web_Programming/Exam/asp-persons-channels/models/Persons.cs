using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Persons
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? Name { get; set; }

        public int Age { get; set; }

        [MaxLength(100)]
        public string Gender { get; set; }

        public ICollection<Channels> Channels { get; set; }
    }
} 