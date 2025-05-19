using Microsoft.EntityFrameworkCore;
using CarDealershipApi.Models;

namespace CarDealershipApi.Data
{
    public class CarDealershipContext : DbContext
    {
        public CarDealershipContext(DbContextOptions<CarDealershipContext> options)
            : base(options)
        {
        }

        public DbSet<Car> Cars { get; set; }
        public DbSet<Category> Categories { get; set; }

        protected override void OnModelCreating(ModelBuilder modelBuilder)
        {
            base.OnModelCreating(modelBuilder);

            modelBuilder.Entity<Car>()
                .HasOne(c => c.Category)
                .WithMany(c => c.Cars)
                .HasForeignKey(c => c.CategoryId);
        }
    }
} 