# Database Management Systems

Semester 4 · Year 2 · C# (.NET Framework, WinForms), SQL Server

How a DBMS works from the application side: ADO.NET master-detail forms on top of SQL Server, then transactions with rollback and partial recovery.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | WinForms master-detail app on the `formula1` database: pick a team, then add/update/delete its drivers with `SqlDataAdapter` and a `DataGridView` |
| [Labs/Lab_02](Labs/Lab_02) | Same master-detail form, but generic: table names, columns and SQL queries come from `App.config` (configured for `iPhoneModels` / `iPhoneInventory`) |
| [Labs/Lab_03](Labs/Lab_03) | T-SQL transactions: validation functions plus `AddRaceDriverResult` (all-or-nothing rollback) and `AddRaceDriverResultRecoverable` (keeps the parts that succeeded), with test calls and a log table |

## How to run

Lab_01 and Lab_02 target .NET Framework 4.7.2, so they need Windows:

1. Open `DBMS.sln` in Visual Studio.
2. Create the database (`formula1` for Lab_01, `iPhoneInventory` for Lab_02) in SQL Server LocalDB.
3. Fix the connection string (see Notes) and press F5.

Lab_03 is a SQL script. Run it in SSMS or Azure Data Studio against the F1 database from [Databases](../../Semester_03/Databases).

## Notes

- The connection strings point to a machine-specific LocalDB named pipe (`np:\\.\pipe\LOCALDB#C4384798\tsql\query` in `Lab_01/Form1.cs`, `LOCALDB#4B9EA026` in `Lab_02/App.config`). Replace them with `Data Source=(localdb)\MSSQLLocalDB` or your own server.
- The table creation scripts for these databases are not in this folder.
