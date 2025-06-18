# Project Management System - Angular + Java Backend

This project has been transformed from a server-side JSP + Servlets application to a modern Angular frontend with the existing Java backend serving as a REST API.

## Architecture Overview

### Backend (Java + Servlets)
- **Technologies**: Java 11, Servlets, SQLite, Maven
- **Port**: 8080
- **Database**: SQLite (`tezt` file)
- **API Endpoints**:
  - `/login` - User authentication
  - `/logout` - User logout  
  - `/project` - Project CRUD operations
  - `/softwareDeveloper` - Developer CRUD operations
  - `/main` - Product and cart operations

### Frontend (Angular)
- **Technologies**: Angular 20, TypeScript, TailwindCSS
- **Port**: 4200
- **Features**:
  - Modern responsive UI
  - Real-time form validation
  - CRUD operations for Projects, Developers, and Products
  - Shopping cart functionality
  - User authentication with session management

## Setup Instructions

### Prerequisites
- Java 11 or higher
- Maven 3.6+
- Node.js 20+ and npm
- A Java servlet container (Tomcat recommended)

### Backend Setup (Java)

1. **Navigate to the project root directory**
   ```bash
   cd /path/to/jsp-angular
   ```

2. **Compile the Java backend**
   ```bash
   mvn clean compile
   ```

3. **Run with Tomcat (using Maven plugin)**
   ```bash
   mvn tomcat7:run
   ```
   
   Or alternatively, build and deploy to your Tomcat server:
   ```bash
   mvn clean package
   # Deploy target/ROOT.war to your Tomcat webapps directory
   ```

4. **Verify backend is running**
   - Backend should be accessible at: http://localhost:8080
   - CORS is configured to allow requests from http://localhost:4200

### Frontend Setup (Angular)

1. **Navigate to the frontend directory**
   ```bash
   cd frontend
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

3. **Start the development server**
   ```bash
   npm start
   # or
   ng serve
   ```

4. **Access the application**
   - Frontend will be available at: http://localhost:4200
   - The app will automatically redirect to the login page

## Features & Usage

### Authentication
- **Login**: Enter any username (no password required for demo)
- **Session Management**: User session is maintained in localStorage
- **Logout**: Available from the dashboard header

### Dashboard Navigation
The dashboard provides access to three main sections:

#### 1. Projects Management
- **Create Project**: Add new projects with name, manager ID, description, and team members
- **View Projects**: See all projects in a responsive table
- **Edit Project**: Modify existing project details
- **Delete Project**: Remove projects with confirmation
- **Filter Projects**: View projects by project manager ID

#### 2. Software Developers Management
- **Add Developer**: Register new developers with name, age, and skills
- **View Developers**: Browse all developers with their information
- **Edit Developer**: Update developer profiles
- **Delete Developer**: Remove developers from the system
- **Search Developer**: Find developers by name

#### 3. Products & Shopping Cart
- **Add Products**: Create new products with name and description
- **Search Products**: Find products by name (partial matching)
- **Shopping Cart**: Add products to cart with quantity
- **Order Management**: Finalize orders and clear cart

### API Integration
The Angular frontend communicates with the Java backend through:
- **HTTP Services**: Dedicated services for each entity (AuthService, ProjectService, etc.)
- **FormData**: Used for API compatibility with existing servlet endpoints
- **CORS Support**: Configured in the backend for cross-origin requests
- **Error Handling**: Comprehensive error management with user-friendly messages

## Technical Details

### Backend Changes Made
1. **CORS Filter**: Added `CORSFilter.java` to handle cross-origin requests
2. **SQL Fix**: Corrected INSERT syntax in `SoftwareDeveloperServlet.java`
3. **JSON Responses**: Existing servlets already return JSON for API consumption

### Frontend Architecture
- **Components**: Modular Angular components for each feature
- **Services**: HTTP services for API communication
- **Models**: TypeScript interfaces matching backend data structures
- **Routing**: Angular router for SPA navigation
- **Styling**: TailwindCSS for modern, responsive design

### Database Schema
The existing SQLite database contains tables for:
- `SoftwareDeveloper` (id, name, age, skills)
- `Project` (id, ProjectManagerID, name, description, members)
- `Products` (id, name, description) 
- `Orders` (user, productId, quantity)

## Development Workflow

### Running Both Applications
1. **Start Backend** (Terminal 1):
   ```bash
   mvn tomcat7:run
   ```

2. **Start Frontend** (Terminal 2):
   ```bash
   cd frontend
   npm start
   ```

3. **Access Application**: Navigate to http://localhost:4200

### Making Changes
- **Backend Changes**: Restart Tomcat after Java code modifications
- **Frontend Changes**: Angular dev server automatically reloads on file changes
- **Database Changes**: SQLite database file (`tezt`) persists data between restarts

## Troubleshooting

### Common Issues

1. **CORS Errors**: Ensure backend is running on port 8080 and CORS filter is active
2. **Port Conflicts**: Make sure ports 4200 and 8080 are available
3. **Node Version**: Use Node.js 20+ for Angular compatibility
4. **Build Errors**: Run `npm install` in frontend directory if dependencies are missing

### API Testing
You can test the backend API directly:
- Login: `POST http://localhost:8080/login` with FormData `name=testuser`
- Projects: `GET http://localhost:8080/project?action=readAll`
- Developers: `GET http://localhost:8080/softwareDeveloper?action=readAll`

## Future Enhancements

### Potential Improvements
- **Authentication**: Implement proper authentication with JWT tokens
- **Database**: Migrate from SQLite to PostgreSQL/MySQL for production
- **Validation**: Add server-side validation for API endpoints
- **Testing**: Add unit and integration tests for both frontend and backend
- **Deployment**: Containerize with Docker for easier deployment
- **Real-time**: Add WebSocket support for real-time updates

### Migration Benefits
- **Modern UI/UX**: Responsive, accessible interface
- **Better Performance**: Client-side rendering and caching
- **Maintainability**: Separation of concerns between frontend and backend
- **Scalability**: Frontend and backend can be scaled independently
- **Developer Experience**: Hot reload, modern tooling, TypeScript

## Support

For issues or questions:
1. Check the browser console for frontend errors
2. Check Tomcat logs for backend errors  
3. Verify both applications are running on correct ports
4. Ensure CORS headers are present in API responses

The application successfully bridges modern frontend development with existing Java backend infrastructure while maintaining all original functionality. 