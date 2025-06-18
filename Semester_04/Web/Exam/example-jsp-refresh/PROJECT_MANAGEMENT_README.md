# Project Management Web Application

A JSP/Servlet-based web application that replicates PHP project management functionality with server-side state management.

## Features

- **Session-based Authentication**: Simple username-based login system
- **Project Management**: Create and assign projects to developers
- **Developer Listing**: View all developers with skill filtering
- **Project Views**: 
  - All projects
  - Your managed projects
  - Projects you're a member of
- **SQLite Database**: Uses the existing `tezt` SQLite database

## Application Structure

### Servlets
- `LoginController` (`/login`) - Handles login/logout functionality
- `MainController` (`/main`) - Main application logic for project management
- `ProjectServlet` (`/project`) - REST API for project operations
- `SoftwareDeveloperServlet` (`/softwareDeveloper`) - REST API for developer operations

### JSP Pages
- `index.jsp` - Landing page that redirects based on login status
- `login.jsp` - Login form
- `main.jsp` - Main application interface

## Usage

### Starting the Application
1. Ensure the SQLite database file `tezt` is in the project root
2. Run the application using Maven: `mvn tomcat7:run`
3. Navigate to `http://localhost:8080`

### Login
- Enter any username to login (no password required)
- The system will create a session and redirect to the main page

### Project Management
1. **View Projects**: 
   - All projects are displayed in the "All Projects" section
   - Your managed projects appear in "Your Managed Projects"
   - Projects where you're a member appear in "Projects You're Member Of"

2. **Assign Project**:
   - Use the "Assign Project" form
   - Enter a project name and project manager name
   - If the project exists, it will be reassigned
   - If the project doesn't exist, it will be created

3. **View Developers**:
   - Click "Show All Developers" to see all developers
   - Use the skill filter to find developers with specific skills
   - Enter skills like "Java", "Python", etc. and click "Filter"

### API Endpoints

#### Project API (`/project`)
- `GET ?action=readAll` - Get all projects
- `GET ?action=readAllByProjectManagerID&projectManagerId=X` - Get projects by manager
- `POST ?action=create` - Create new project
- `POST ?action=assignProject` - Assign project to manager
- `PUT` - Update project
- `DELETE ?id=X` - Delete project

#### Developer API (`/softwareDeveloper`)
- `GET ?action=readAll` - Get all developers
- `GET ?action=findOne&name=X` - Find developer by name
- `POST` - Create new developer
- `PUT` - Update developer
- `DELETE ?id=X` - Delete developer

## Database Schema

The application expects the following tables in the SQLite database:

### Project Table
```sql
CREATE TABLE Project (
    id INTEGER PRIMARY KEY,
    ProjectManagerID INTEGER,
    name TEXT,
    description TEXT,
    members TEXT
);
```

### SoftwareDeveloper Table
```sql
CREATE TABLE SoftwareDeveloper (
    id INTEGER PRIMARY KEY,
    name TEXT,
    age INTEGER,
    skills TEXT
);
```

## Key Features

1. **Server-side State Management**: All application state is maintained on the server using HTTP sessions
2. **SQLite Integration**: Direct database integration using JDBC
3. **Responsive UI**: Uses Tailwind CSS for modern, responsive design
4. **Real-time Filtering**: Client-side JavaScript for developer skill filtering
5. **Session Security**: Automatic redirection to login for unauthenticated users

## Differences from PHP Version

- Uses Java servlets instead of PHP for server-side logic
- JSP for templating instead of PHP mixed with HTML
- SQLite JDBC driver instead of PDO
- Session management using HttpSession instead of PHP sessions
- JSON responses for API endpoints using Gson library

## Getting Started

1. Make sure you have Java 11+ and Maven installed
2. Clone/download the project
3. Ensure the SQLite database `tezt` is in the root directory
4. Run `mvn clean compile`
5. Start the server with `mvn tomcat7:run`
6. Open `http://localhost:8080` in your browser
7. Login with any username to start using the application 