# Project Management System - REST API + JavaScript Frontend

This project has been refactored to separate the frontend and backend:
- **.NET Core REST API**: Backend that provides JSON endpoints
- **JavaScript Frontend**: Single-page application that consumes the API

## Architecture

- **Backend**: .NET Core Web API with Entity Framework and SQLite
- **Frontend**: Vanilla JavaScript SPA with modern UI using Tailwind CSS
- **Database**: SQLite database shared with the original PHP application

## API Endpoints

### Authentication
- `POST /api/auth/login` - Login with username
- `POST /api/auth/logout` - Logout current user
- `GET /api/auth/status` - Check authentication status

### Projects
- `GET /api/projects` - Get all projects (categorized by user relationship)
- `POST /api/projects` - Create a new project

### Developers
- `GET /api/developers` - Get all developers
- `GET /api/developers?skill=Java` - Filter developers by skill
- `GET /api/developers/{id}` - Get specific developer

## Database

The application connects to the same SQLite database file (`tezt`) that your PHP application uses.

## Models

The following models have been created to match your database schema:

- **SoftwareDeveloper**: Developers with skills
- **Project**: Projects managed by developers

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

The frontend will be served from the root URL and will automatically consume the REST API endpoints.

## Project Structure

```
├── Controllers/
│   ├── Api/
│   │   ├── AuthController.cs      # Authentication API endpoints
│   │   ├── ProjectsController.cs  # Projects API endpoints
│   │   └── DevelopersController.cs # Developers API endpoints
│   ├── HomeController.cs          # Legacy MVC controller
│   ├── LoginController.cs         # Legacy MVC controller
│   └── StaticController.cs        # Serves the frontend HTML
├── Data/
│   └── ApplicationDbContext.cs    # Entity Framework DbContext
├── Models/
│   ├── SoftwareDeveloper.cs       # Developer model
│   └── Project.cs                 # Project model
├── Views/                         # Legacy MVC views (can be removed)
├── wwwroot/
│   ├── index.html                 # Main frontend HTML
│   └── js/
│       └── app.js                 # JavaScript application logic
├── Program.cs                     # Application entry point
├── ProjectManagement.csproj       # Project file
└── appsettings.json              # Configuration
```

## Features

✅ **User Authentication**: Session-based login/logout  
✅ **Project Management**: View all projects, managed projects, and member projects  
✅ **Project Assignment**: Create new projects and assign managers  
✅ **Developer Directory**: Browse and filter developers by skills  
✅ **Real-time UI**: Loading indicators and error handling  
✅ **Responsive Design**: Modern UI with Tailwind CSS  

## API Configuration

- **CORS**: Configured to allow frontend origins
- **Sessions**: Enabled for authentication state
- **JSON**: All API responses in JSON format
- **Error Handling**: Proper HTTP status codes and error messages

## Migration Notes

This refactored version maintains all the original functionality while providing:
- **Better separation of concerns**: Frontend and backend are decoupled
- **API-first approach**: Backend can be consumed by multiple frontends
- **Modern UI**: Single-page application with better user experience
- **Scalability**: Easier to scale and maintain

The legacy MVC controllers (`HomeController`, `LoginController`) are still present but not used by the new frontend. They can be removed once migration is complete. 