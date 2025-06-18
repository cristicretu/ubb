using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Project
    {
        [Key]
        public int Id { get; set; }

        [ForeignKey("ProjectManager")]
        public int? ProjectManagerID { get; set; }

        [MaxLength(100)]
        public string? Name { get; set; }

        [MaxLength(100)]
        public string? Description { get; set; }

        [MaxLength(100)]
        public string? Members { get; set; }

        // Navigation property for the project manager
        public virtual SoftwareDeveloper? ProjectManager { get; set; }
    }
} 