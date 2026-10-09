# Databases

Semester 3 · Year 2 · SQL Server (T-SQL)

Relational database design and SQL. The lab work builds one Formula 1 database (teams, drivers, races, circuits, results, sponsors, engines) and extends it each lab with queries, versioning procedures, performance tests and indexes.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | Schema of the F1 database (tables, keys, foreign keys) |
| [Labs/Lab_03](Labs/Lab_03) | Insert/update/delete statements on the F1 database |
| [Labs/Lab_08](Labs/Lab_08) | Inserts, updates, deletes and query homework: joins, `UNION`, `INTERSECT`, `EXCEPT`, `GROUP BY`/`HAVING` |
| [Labs/Lab_11](Labs/Lab_11) | Full script: versioning procedures (`versionUp_N`/`versionDown_N`, `UpdateDatabaseVersion`), views, and the `Tests`/`TestRuns` tables for timing inserts and views |
| [Labs/Lab_12](Labs/Lab_12) | Index lab: tables `Ta`, `Tb`, `Tc` with nonclustered indexes and queries to compare execution plans |
| [Labs/Practice](Labs/Practice) | Practical exam practice, one folder per problem (Money, Shoes, cake, cinema, crypto, model, srls, zoo) |
| [Labs/test](Labs/test) | Practical exam (car repair shop) |

Each practice/test folder follows the exam format:

- `1.sql`: create the tables
- `2.sql`: stored procedure
- `3.sql`, `4.sql`: views or functions
- `mock_data.sql`: sample inserts (where present)

## How to run

Run the scripts in SQL Server Management Studio, Azure Data Studio or `sqlcmd` against a SQL Server instance:

```sh
sqlcmd -S localhost -U sa -P '<password>' -i 1.sql
```

On macOS/Linux, SQL Server runs in Docker (`mcr.microsoft.com/mssql/server`).

## Notes

- The scripts use T-SQL syntax (`identity`, `GO`, `create procedure`), so they will not run on MySQL or PostgreSQL as-is.
- `Lab_03/console.txt` is SQL saved as text, with the schema notes commented out at the top.
