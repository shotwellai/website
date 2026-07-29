import "dotenv/config";

import { readdir, readFile } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { Pool } from "pg";

import { poolConfig } from "../db/pool.js";

const databaseUrl = process.env.DATABASE_URL;

if (!databaseUrl) {
  throw new Error("DATABASE_URL is required to run migrations.");
}

const root = dirname(dirname(dirname(fileURLToPath(import.meta.url))));
const migrationsDir = join(root, "migrations");
const migrationFiles = (await readdir(migrationsDir))
  .filter((file) => file.endsWith(".sql"))
  .sort();

const pool = new Pool(poolConfig(databaseUrl, 1));

try {
  for (const migrationFile of migrationFiles) {
    const sql = await readFile(join(migrationsDir, migrationFile), "utf8");
    await pool.query(sql);
    console.log(`Applied migration: ${migrationFile}`);
  }
} finally {
  await pool.end();
}
