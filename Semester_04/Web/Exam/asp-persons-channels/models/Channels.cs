using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Channels
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? Name { get; set; }

        [MaxLength(100)]
        public string? Description { get; set; }

        [MaxLength(500)]
        public string? Subscribers { get; set; }

        public int SubscriberCount 
        { 
            get 
            {
                if (string.IsNullOrEmpty(Subscribers)) return 0;
                return Subscribers.Split(';', StringSplitOptions.RemoveEmptyEntries).Length;
            }
        }

        public int? OwnerId { get; set; }

        public Persons? Owner { get; set; }
    }
} 