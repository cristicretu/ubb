using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Courses 
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? ProfessorId { get; set; }

        [MaxLength(100)]
        public string? CourseName { get; set; }

        [MaxLength(100)]
        public string? Participants { get; set; }

        [MaxLength(100)]
        public string? Grades { get; set; }

        [ForeignKey("ProfessorId")]
        public Persons? Professor { get; set; }
    }
} 