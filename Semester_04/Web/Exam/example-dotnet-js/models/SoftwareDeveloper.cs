using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class SoftwareDeveloper
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? Name { get; set; }

        public int Age { get; set; } = 0;

        [MaxLength(100)]
        public string? Skills { get; set; }

        // Navigation property for projects managed by this developer
        public virtual ICollection<Project> ManagedProjects { get; set; } = new List<Project>();
    }
} 