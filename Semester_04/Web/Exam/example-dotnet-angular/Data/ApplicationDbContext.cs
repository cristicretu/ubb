using Microsoft.EntityFrameworkCore;
using ProjectManagement.Models;

namespace ProjectManagement.Data
{
    public class ApplicationDbContext : DbContext
    {
        public ApplicationDbContext(DbContextOptions<ApplicationDbContext> options) : base(options)
        {
        }

        public DbSet<SoftwareDeveloper> SoftwareDevelopers { get; set; }
        public DbSet<Project> Projects { get; set; }

        protected override void OnModelCreating(ModelBuilder modelBuilder)
        {
            base.OnModelCreating(modelBuilder);

            modelBuilder.Entity<SoftwareDeveloper>().ToTable("SoftwareDeveloper");
            modelBuilder.Entity<Project>().ToTable("Project");

            modelBuilder.Entity<Project>()
                .HasOne(p => p.ProjectManager)
                .WithMany(s => s.ManagedProjects)
                .HasForeignKey(p => p.ProjectManagerID)
                .OnDelete(DeleteBehavior.SetNull);
        }
    }
} 