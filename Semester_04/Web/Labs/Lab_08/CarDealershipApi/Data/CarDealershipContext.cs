using Microsoft.EntityFrameworkCore;
using CarDealershipApi.Models;
using Microsoft.AspNetCore.Identity.EntityFrameworkCore;

namespace CarDealershipApi.Data
{
    public class CarDealershipContext : IdentityDbContext<ApplicationUser>
    {
        public CarDealershipContext(DbContextOptions<CarDealershipContext> options)
            : base(options)
        {
        }

        public DbSet<Car> Cars { get; set; }
        public DbSet<Category> Categories { get; set; }

        protected override void OnModelCreating(ModelBuilder modelBuilder)
        {
            // Apply Identity configuration first
            base.OnModelCreating(modelBuilder);

            // Configure existing Car and Category entities
            modelBuilder.Entity<Car>()
                .HasOne(c => c.Category)
                .WithMany(c => c.Cars)
                .HasForeignKey(c => c.CategoryId);
                
            // Preserve existing table names
            modelBuilder.Entity<Car>().ToTable("cars");
            modelBuilder.Entity<Category>().ToTable("categories");
        }
    }
} 