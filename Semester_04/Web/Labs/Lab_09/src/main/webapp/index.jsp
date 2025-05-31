<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ page import="model.User" %>
<%@ page import="model.GameState" %>
<%@ page import="model.Position" %>
<%@ page import="repository.GameStateRepository" %>
<%@ page import="java.util.Optional" %>
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Game Hub - JSP & Servlets</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        .game-cell {
            width: 20px;
            height: 20px;
            border: 1px solid #e5e7eb;
        }
        .snake { background-color: #10b981; }
        .apple { background-color: #ef4444; }
        .obstacle { background-color: #374151; }
        .empty { background-color: #f9fafb; }
    </style>
</head>
<body class="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 p-4">
    <%
        User currentUser = (User) session.getAttribute("user");
        boolean isLoggedIn = currentUser != null;
        
        GameState currentGame = null;
        if (isLoggedIn) {
            try {
                GameStateRepository gameRepo = new GameStateRepository();
                Optional<GameState> gameOpt = gameRepo.findLatestByUserId(currentUser.getId());
                currentGame = gameOpt.orElse(null);
            } catch (Exception e) {
                // Handle database error
            }
        }
    %>
    
    <div class="max-w-4xl mx-auto">
        <!-- User Header (if logged in) -->
        <% if (isLoggedIn) { %>
            <div class="bg-white rounded-lg shadow-md p-4 mb-6 border border-gray-200">
                <div class="flex justify-between items-center">
                    <div class="flex items-center space-x-3">
                        <div class="w-10 h-10 bg-blue-500 rounded-full flex items-center justify-center text-white font-bold">
                            <%= currentUser.getUsername().substring(0, 1).toUpperCase() %>
                        </div>
                        <div>
                            <h3 class="font-semibold text-gray-800">Welcome back, <%= currentUser.getUsername() %>!</h3>
                            <p class="text-sm text-gray-600">High Score: <span class="font-bold text-green-600"><%= currentUser.getHighScore() %></span></p>
                        </div>
                    </div>
                    <div>
                        <a href="/user?action=logout" class="bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded transition-colors duration-200">Logout</a>
                    </div>
                </div>
            </div>
        <% } %>

        <!-- Header -->
        <header class="text-center mb-8">
            <h1 class="text-4xl font-bold text-gray-800 mb-2">🐍 Snake Game</h1>
        </header>

        <% if (isLoggedIn) { %>
            <!-- Game Messages -->
            <%
                String gameMessage = (String) session.getAttribute("gameMessage");
                if (gameMessage != null) {
                    session.removeAttribute("gameMessage");
            %>
                <div class="bg-yellow-100 border border-yellow-400 text-yellow-700 px-4 py-3 rounded mb-6">
                    <%= gameMessage %>
                </div>
            <% } %>
            
            <!-- Game Section -->
            <div class="bg-white rounded-lg shadow-md p-6 mb-6">
                <% if (currentGame != null) { %>
                    <!-- Current Game Display -->
                    <div class="mb-4">
                        <div class="flex justify-between items-center mb-2">
                            <h3 class="text-lg font-semibold">Current Game - Score: <%= currentGame.getScore() %></h3>
                        </div>
                        
                        <!-- Game Grid -->
                        <div class="inline-block border-2 border-gray-400 bg-gray-100 p-2">
                            <%
                                final int GRID_SIZE = 25;
                                Position[] snake = currentGame.getSnake();
                                Position[] obstacles = currentGame.getObstacles();
                                Position apple = currentGame.getApple();
                                
                                String[][] grid = new String[GRID_SIZE][GRID_SIZE];
                                
                                for (int i = 0; i < GRID_SIZE; i++) {
                                    for (int j = 0; j < GRID_SIZE; j++) {
                                        grid[i][j] = "empty";
                                    }
                                }
                                
                                if (obstacles != null) {
                                    for (Position obs : obstacles) {
                                        if (obs.getX() >= 0 && obs.getX() < GRID_SIZE && obs.getY() >= 0 && obs.getY() < GRID_SIZE) {
                                            grid[obs.getY()][obs.getX()] = "obstacle";
                                        }
                                    }
                                }
                                
                                if (apple != null && apple.getX() >= 0 && apple.getX() < GRID_SIZE && apple.getY() >= 0 && apple.getY() < GRID_SIZE) {
                                    grid[apple.getY()][apple.getX()] = "apple";
                                }
                                
                                if (snake != null) {
                                    for (Position segment : snake) {
                                        if (segment.getX() >= 0 && segment.getX() < GRID_SIZE && segment.getY() >= 0 && segment.getY() < GRID_SIZE) {
                                            grid[segment.getY()][segment.getX()] = "snake";
                                        }
                                    }
                                }
                                
                                for (int y = 0; y < GRID_SIZE; y++) {
                                    out.println("<div style='display: flex;'>");
                                    for (int x = 0; x < GRID_SIZE; x++) {
                                        out.println("<div class='game-cell " + grid[y][x] + "'></div>");
                                    }
                                    out.println("</div>");
                                }
                            %>
                        </div>
                        
                        <div class="mt-4 flex space-x-2 justify-center">
                            <form method="post" action="/gamestate" style="display: inline;">
                                <input type="hidden" name="action" value="reset">
                                <button type="submit" class="bg-yellow-500 hover:bg-yellow-600 text-white px-4 py-2 rounded">Reset Game</button>
                            </form>
                            <form method="post" action="/gamestate" style="display: inline;">
                                <input type="hidden" name="action" value="end">
                                <button type="submit" class="bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded">End Game</button>
                            </form>
                        </div>
                    </div>
                <% } else { %>
                    <div class="text-center">
                        <h3 class="text-lg font-semibold mb-4">No active game</h3>
                        <form method="post" action="/gamestate">
                            <input type="hidden" name="action" value="start">
                            <button type="submit" class="bg-green-500 hover:bg-green-600 text-white font-bold py-3 px-6 rounded-lg">Start New Game</button>
                        </form>
                    </div>
                <% } %>
            </div>
            
        <% } else { %>
            <div class="flex justify-center space-x-4">
                <a href="/login/index.jsp" class="bg-blue-500 hover:bg-blue-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200 block text-center">🔑 Login</a>
                <a href="/register/index.jsp" class="bg-green-500 hover:bg-green-600 text-white font-bold py-2 px-4 rounded w-full transition-colors duration-200 block text-center">📝 Register</a>
            </div>
        <% } %>
    </div>

    <% if (isLoggedIn && currentGame != null) { %>
    <script>
        let gameInterval;
        let gameActive = true;

        // function startAutoMovement() {
        //     if (gameInterval) clearInterval(gameInterval);
            
        //     gameInterval = setInterval(function() {
        //         if (gameActive) {
        //             fetch('/gamestate', {
        //                 method: 'POST',
        //                 headers: {
        //                     'Content-Type': 'application/x-www-form-urlencoded',
        //                 },
        //                 body: 'action=autoMove'
        //             })
        //             .then(response => response.json())
        //             .then(data => {
        //                 if (data.success) {
        //                     window.location.reload();
        //                 } else {
        //                     gameActive = false;
        //                     clearInterval(gameInterval);
        //                     if (data.message === "Game Over") {
        //                         alert("Game Over!");
        //                         window.location.reload();
        //                     }
        //                 }
        //             })
        //             .catch(error => {
        //                 console.error('Error:', error);
        //                 gameActive = false;
        //                 clearInterval(gameInterval);
        //             });
        //         }
        //     }, 500);
        // }

        document.addEventListener('keydown', function(event) {
            if (!gameActive) return;
            
            let direction = null;
            switch(event.key) {
                case 'ArrowUp':
                    direction = 'UP';
                    break;
                case 'ArrowDown':
                    direction = 'DOWN';
                    break;
                case 'ArrowLeft':
                    direction = 'LEFT';
                    break;
                case 'ArrowRight':
                    direction = 'RIGHT';
                    break;
                default:
                    return; 
            }
            
            if (direction) {
                event.preventDefault(); 
                
                fetch('/gamestate', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/x-www-form-urlencoded',
                    },
                    body: 'action=move&direction=' + direction
                })
                .then(response => {
                    if (response.ok) {
                        window.location.reload();
                    }
                })
                .catch(error => {
                    console.error('Error:', error);
                });
            }
        });

        startAutoMovement();

        window.addEventListener('beforeunload', function() {
            gameActive = false;
            if (gameInterval) clearInterval(gameInterval);
        });
    </script>
    <% } %>
</body>
</html> 