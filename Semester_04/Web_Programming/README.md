# Web Programming

Semester 4 · Year 2 · HTML, CSS, JavaScript, jQuery, PHP, Angular, ASP.NET Core, JSP/Servlets

Static pages first (HTML, CSS, DOM, jQuery), then server-side apps: a car dealership CRUD in PHP, the same app with an Angular frontend and an ASP.NET Core API, and a JSP/Servlets game. The exam is a small two-table web app written in PHP, JSP or ASP.NET under time pressure, so `Exam/` holds many practice solutions in every stack.

## Contents

### Labs

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | "Porsche History" static page in HTML and CSS, with an audio element |
| [Labs/Lab_02](Labs/Lab_02) | CSS-only bar chart with tooltips and a two-level menu; `lab_stuff/` is the given layout exercise |
| [Labs/Lab_03](Labs/Lab_03) | Clone of a corporate site's navigation bar and layout with flexbox and grid |
| [Labs/Lab_04](Labs/Lab_04) | Vanilla JS: table of Porsche models with add form, input validation and sorting by column (Tailwind via CDN) |
| [Labs/Lab_05](Labs/Lab_05) | jQuery: payment modal with show/hide, fade effects and form validation |
| [Labs/Lab_06](Labs/Lab_06) | PHP + MySQL car dealership: cars by category, add/edit/delete, JSON API loaded with `fetch` |
| [Labs/Lab_07](Labs/Lab_07) | Same PHP API with an Angular 17 frontend (`angular-frontend/`) |
| [Labs/Lab_08](Labs/Lab_08) | Backend rewritten in ASP.NET Core (`CarDealershipApi/`: EF Core on MySQL, Identity login/register) with the Angular frontend and an auth guard |
| [Labs/Lab_09](Labs/Lab_09) | JSP/Servlets snake game with register/login, saved game state and moves in SQLite |

### Exam practice

Every folder ships its SQLite database as a file named `tezt`. The `example-*` folders solve the same task (`task.md`: software developers and projects, assign a developer to a project) in different stacks; the rest are other past exam problems.

| Folder | Stack | Problem |
| --- | --- | --- |
| [example-php-refresh](Exam/example-php-refresh) | PHP, full page reloads | Developers and projects |
| [example-php-js-fetch](Exam/example-php-js-fetch) | PHP API + JS `fetch` | Developers and projects |
| [example-php-angular](Exam/example-php-angular) | PHP API + Angular | Developers and projects |
| [example-dotnet-cshtml](Exam/example-dotnet-cshtml) | ASP.NET MVC, Razor views | Developers and projects |
| [example-dotnet-js](Exam/example-dotnet-js) | ASP.NET API + JS | Developers and projects |
| [example-dotnet-angular](Exam/example-dotnet-angular) | ASP.NET API + Angular build in `wwwroot` | Developers and projects |
| [example-jsp-refresh](Exam/example-jsp-refresh) | JSP/Servlets | Developers and projects |
| [jsp-js](Exam/jsp-js), [jsp-angular](Exam/jsp-angular) | JSP + JS / Angular | Products and orders |
| [jsp-products](Exam/jsp-products) | JSP | Products and orders |
| [jsp-movies](Exam/jsp-movies) | JSP | Movies and documents by author, document with most authors |
| [jsp-topics](Exam/jsp-topics) | JSP | Topics and posts |
| [php-topics-notifications](Exam/php-topics-notifications) | PHP | Topics and posts |
| [php-hotel-reservations](Exam/php-hotel-reservations) | PHP | Hotel rooms available between two dates, reservations, guests per day |
| [php-user-property](Exam/php-user-property) | PHP | Users and properties, properties with several owners |
| [php-websites](Exam/php-websites) | PHP | Websites and documents, search by keywords |
| [asp-files](Exam/asp-files) | ASP.NET MVC | A user's files with pagination, most frequent file |
| [asp-persons-channels](Exam/asp-persons-channels) | ASP.NET MVC | Persons subscribing to channels |
| [asp-reservations](Exam/asp-reservations) | ASP.NET MVC | Flight and hotel reservations |
| [exam](Exam/exam) | ASP.NET MVC | Products, orders and a session cart |

## How to run

- Static labs (01 to 05): open `index.html` in a browser.
- PHP: `php -S localhost:8000` in the folder. Lab_06/07 need MySQL with `database.sql` imported (db `car_dealership`, user `root`, empty password).
- Angular: `cd angular-frontend && npm install && npm start`. It proxies `/api` to `http://localhost:8000`.
- ASP.NET (exam folders): `dotnet run` next to `ProjectManagement.csproj` (.NET 8).
- JSP: `mvn package tomcat7:run`, then open `http://localhost:8080`.

## Notes

- Some PHP exam configs hardcode the database path, e.g. `/Users/huge/fun/ubb/Semester_04/Web/Exam/php-websites/tezt` in `config/database.php`. Point it to the `tezt` file in your copy.
- Lab_08's `.csproj` is gitignored. Create one with `dotnet new webapi -n CarDealershipApi` and add Pomelo MySQL, EF Core and Identity packages before `dotnet run`.
- `Lab_08/api/` is a leftover copy of the PHP endpoints and is missing its `config/` and `includes/`.
- `task.md` is the same template in every exam folder, so read the code (or the database) to see the actual problem. Several exam READMEs are copied from another project and do not describe the folder.
