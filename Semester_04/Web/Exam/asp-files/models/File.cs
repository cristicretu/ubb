using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class File 
    {
        [Key]
        public int Id { get; set; }

        public int UserId { get; set; }

        [MaxLength(100)]
        public string? Filename { get; set; }

        [MaxLength(100)]
        public string? Filepath { get; set; }

        public int Size { get; set; }
    }
} 