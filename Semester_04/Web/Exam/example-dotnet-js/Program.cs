using Microsoft.EntityFrameworkCore;
using ProjectManagement.Data;

var builder = WebApplication.CreateBuilder(args);

// Add services
builder.Services.AddControllers(); // For API controllers
builder.Services.AddControllersWithViews(); // For MVC controllers (if still needed)

builder.Services.AddDbContext<ApplicationDbContext>(options =>
    options.UseSqlite(builder.Configuration.GetConnectionString("DefaultConnection")));

builder.Services.AddSession(options =>
{
    options.IdleTimeout = TimeSpan.FromMinutes(10000);
    options.Cookie.HttpOnly = false; // Allow JavaScript access for CORS
    options.Cookie.IsEssential = true;
    options.Cookie.SameSite = SameSiteMode.None; // Required for cross-origin
    options.Cookie.SecurePolicy = CookieSecurePolicy.SameAsRequest; // Allow HTTP in development
    options.Cookie.Name = "ProjectManagement.Session"; // Explicit cookie name
});

// Add CORS for frontend
builder.Services.AddCors(options =>
{
    options.AddPolicy("AllowFrontend", policy =>
    {
        policy.WithOrigins(
                "http://localhost:3000", 
                "http://127.0.0.1:3000",
                "http://localhost:5173", 
                "http://127.0.0.1:5173",
                "http://localhost:5500",
                "http://127.0.0.1:5500",
                "http://localhost:5000",
                "http://127.0.0.1:5000",
                "http://localhost:5001",
                "http://127.0.0.1:5001")
              .AllowAnyHeader()
              .AllowAnyMethod()
              .AllowCredentials();
    });
});

var app = builder.Build();

if (!app.Environment.IsDevelopment())
{
    app.UseExceptionHandler("/Home/Error");
    app.UseHsts();
    app.UseHttpsRedirection();
}
else
{
    app.UseDeveloperExceptionPage();
}

app.UseStaticFiles();
app.UseRouting();

// Enable CORS
app.UseCors("AllowFrontend");

// Add session middleware
app.UseSession();
app.UseAuthorization();

// Map API routes
app.MapControllers();

// Map MVC routes - serve static frontend by default
app.MapControllerRoute(
    name: "default",
    pattern: "{controller=Static}/{action=Index}/{id?}");

app.Run(); 