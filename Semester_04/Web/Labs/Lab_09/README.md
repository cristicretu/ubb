# JSP/Servlet Hello World Boilerplate

A minimal JSP/Servlet starter template for web development.

## What's Included

- **index.jsp** - Basic JSP page
- **HelloServlet.java** - Simple servlet example
- **web.xml** - Deployment descriptor
- **pom.xml** - Maven configuration

## Quick Start

1. **Compile and run:**
   ```bash
   mvn clean compile
   mvn tomcat7:run
   ```

2. **Access the app:**
   - Open: `http://localhost:8080/hello-world`

## Project Structure
```
├── web.xml                    # Servlet configuration
├── pom.xml                    # Maven dependencies
├── index.jsp                  # JSP page
├── WEB-INF/
│   └── classes/
│       └── HelloServlet.java  # Basic servlet
└── README.md
```

## Next Steps

Now you can implement:
- Login system with sessions
- Snake game logic
- Database integration
- Additional servlets and JSP pages

## Build Commands

- **Compile:** `mvn clean compile`
- **Run:** `mvn tomcat7:run`
- **Package:** `mvn clean package` (creates WAR file)

That's it! You have a working JSP/Servlet foundation to build upon. 