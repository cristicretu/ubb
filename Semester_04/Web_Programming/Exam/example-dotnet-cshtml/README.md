# .NET MVC Project Management System

This is a .NET Core MVC application that uses the same SQLite database as your PHP application.

## Database

The application connects to the same SQLite database file (`tezt`) that your PHP application uses.

## Models

The following models have been created to match your database schema:

- **SoftwareDeveloper**: Developers with skills
- **Project**: Projects managed by developers
- **Category**: Car categories
- **Car**: Cars with categories

## Database Configuration

- **Connection String**: `Data Source=tezt`
- **Entity Framework**: Configured to use SQLite
- **DbContext**: `ApplicationDbContext` with proper table mappings

## How to Run

1. Make sure you have .NET 8.0 SDK installed
2. Navigate to the project directory
3. Restore packages:
   ```bash
   dotnet restore
   ```
4. Run the application:
   ```bash
   dotnet run
   ```

The application will start and you can access it at `https://localhost:5001` or `http://localhost:5000`.

## Project Structure

```
├── Controllers/
│   └── HomeController.cs          # Home controller
├── Data/
│   └── ApplicationDbContext.cs    # Entity Framework DbContext
├── Models/
│   ├── SoftwareDeveloper.cs       # Developer model
│   ├── Project.cs                 # Project model
│   ├── Category.cs                # Category model
│   └── Car.cs                     # Car model
├── Views/
│   ├── Home/
│   │   └── Index.cshtml           # Home page
│   └── Shared/
│       └── _Layout.cshtml         # Layout template
├── wwwroot/                       # Static files
├── Program.cs                     # Application entry point
├── ProjectManagement.csproj       # Project file
└── appsettings.json              # Configuration
```

## Next Steps

The models and database configuration are ready. You can now:

1. Create API controllers for your endpoints
2. Build your frontend views
3. Add authentication if needed
4. Implement business logic

The application is configured to use the same database as your PHP app, so all your existing data will be available. 