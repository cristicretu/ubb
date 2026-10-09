using System;
using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace CarDealershipApi.Models
{
    [Table("cars")]
    public class Car
    {
        [Key]
        [Column("id")]
        public int Id { get; set; }
        
        [Required]
        [StringLength(100)]
        [Column("model")]
        public string Model { get; set; } = string.Empty;
        
        [Required]
        [StringLength(50)]
        [Column("engine_power")]
        public string EnginePower { get; set; } = string.Empty;
        
        [Required]
        [StringLength(50)]
        [Column("fuel_type")]
        public string FuelType { get; set; } = string.Empty;
        
        [Required]
        [Column("price", TypeName = "decimal(10,2)")]
        public decimal Price { get; set; }
        
        [Required]
        [StringLength(50)]
        [Column("color")]
        public string Color { get; set; } = string.Empty;
        
        [Required]
        [Column("year")]
        public int Year { get; set; }
        
        [Column("history")]
        public string? History { get; set; }
        
        [Column("features")]
        public string? Features { get; set; }
        
        [Required]
        [Column("category_id")]
        public int CategoryId { get; set; }
        
        [ForeignKey("CategoryId")]
        public Category? Category { get; set; }
        
        [Required]
        [Column("created_at")]
        public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    }
} 