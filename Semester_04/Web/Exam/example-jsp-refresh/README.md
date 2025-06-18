# JSP & Servlets Game Application

A modern web application built with JSP, Servlets, and Tailwind CSS for creating interactive games.

## 🏗️ Architecture

This project follows a clean separation of concerns:

- **Servlets** - Act as controllers that interact with the database and repositories
- **JSP Pages** - Serve as the UI layer with modern Tailwind CSS styling
- **Repository Pattern** - Handle data access operations
- **Database** - Store user data and game scores

## 🚀 Getting Started

### Prerequisites

- Java 11 or higher
- Maven 3.6+
- A servlet container (Tomcat, Jetty, etc.)

### Setup

1. **Clone the repository**
   ```bash
   git clone <your-repo-url>
   cd Lab_09
   ```

2. **Build the Java application**
   ```bash
   mvn clean compile
   ```

3. **Deploy to your servlet container**
   ```bash
   mvn package
   # Deploy the generated WAR file to your servlet container
   ```

That's it! No Node.js setup needed - Tailwind CSS is loaded via CDN.

## 🎨 Frontend Development

### Tailwind CSS

The project uses Tailwind CSS via CDN for styling. Simply use Tailwind classes directly in your JSP files:

```html
<!-- Buttons -->
<button class="bg-blue-500 hover:bg-blue-600 text-white font-bold py-2 px-4 rounded">Primary Button</button>
<button class="bg-green-500 hover:bg-green-600 text-white font-bold py-2 px-4 rounded">Secondary Button</button>

<!-- Cards -->
<div class="bg-white rounded-lg shadow-md p-6 border border-gray-200">
  <h2>Card Title</h2>
  <p>Card content...</p>
</div>

<!-- Game Container -->
<div class="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 p-4">
  <!-- Full-height gradient background -->
</div>
```

### Development Workflow

1. **Make changes to JSP files** in `src/main/webapp/`
2. **Use Tailwind classes** directly in your JSP files
3. **Test and deploy** your application

## 📁 Project Structure

```
Lab_09/
├── src/main/
│   ├── java/                 # Java servlets and classes
│   │   └── HelloServlet.java
│   └── webapp/               # Web resources
│       ├── WEB-INF/
│       │   └── web.xml
│       └── index.jsp         # Main page
├── target/                   # Maven build output
├── pom.xml                   # Maven configuration
├── .gitignore               # Git ignore rules
└── README.md                # This file
```

## 🎮 Features to Implement

- [ ] User authentication system
- [ ] Snake game with JavaScript
- [ ] Score tracking and leaderboards
- [ ] User profiles
- [ ] Game statistics

## 🛠️ Development Tips

1. **Servlet Development**: Create servlets in `src/main/java/` and register them in `web.xml`
2. **JSP Development**: Create JSP files in `src/main/webapp/` and use Tailwind classes for styling
3. **Database Integration**: Implement repository pattern for data access
4. **Styling**: Use Tailwind utility classes directly - no build process needed!

## 📝 Notes

- Tailwind CSS is loaded via CDN - no build process required
- Use Tailwind utility classes for consistent styling
- Follow the MVC pattern: Servlets (Controllers) → JSP (Views) → Repository (Model)

## 🤝 Contributing

1. Make your changes
2. Test the application
3. Update documentation if needed
4. Submit a pull request 