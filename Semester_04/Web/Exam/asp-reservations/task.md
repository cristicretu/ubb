1. Write a web application in PHP/JSP/ASP which uses the following 2 database tables:
- table SoftwareDeveloper: id (int), name (string), age (int), skills (string)
- table Project: id (int), ProjectManagerID (int), name (string), description
  (string),   members (string)

The 'members' column from the Project table contains a list of persons
who are part of this project. The 'ProjectManagerID' is a foreign key and
references an entry in the SoftwareDeveloper table. The user should be able to specify
his/her name in a text field when starting to use the application. The user is a
software developer who can be a member of several projects. The user should be able
to see all the projects (display all the data of a project) in the database.
The user should be able to view all the projects (their names) he/she is member of. 
The current user should be able to assign another developer to a list of
projects (in one single HTTP request sent to the server - you should not send separate
HTTP requests to the server for each project). If the developer who should be
added to a project does not already exist in the SoftwareDeveloper table, nothing
happens (the non-existing developer is not added to any project). If the developer
has to be added to a non-existing project, this project is automatically added to
the Project table by the server-side (you are free to specify only the 'name' of the
project and leave the other fields of the project record empty).
There should also be a button that displays all the Software Developers from
the database and a javascript code should filter only those that have a
specific skill (e.g. 'Java'), skill that is chosen by the user using a text
input.


Grading scale:
- 1 point by default (oficiu) 
- configure web environment, create DB, display all projects in the database  : 2
- display all projects (only the project's name) the user is member of	      : 2.5
- assign other developer to a list of projects				      : 3
(nothing happens if the developer is not already in the SoftwareDeveloper table; 
add a new project if the project the developer should be member of doesn't exist 
in the Project table)
- display all software developers (server-side) and filter only the developers
that have a specific skill (client-side, javascript)			      : 1.5

You are not allowed to use any other DB tables except the ones specified above.
Also, all the data should be stored at the server-side in databases, not text files
or something similar.
------------------------------------------------------------------------------------
The server-side technology (PHP or JSP or ASP.NET) is not at your choice, it is 
fixed by the exam subject. However, you can change this technology to another 
one of your preference, but in this case the final grade that you receive will 
be cut at 6 (i.e. the maximum grade that you can get for this practical exam is 6).
