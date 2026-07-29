import "dotenv/config";

import { readFile } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { Pool } from "pg";

import { poolConfig } from "../db/pool.js";

const databaseUrl = process.env.DATABASE_URL;

if (!databaseUrl) {
  throw new Error("DATABASE_URL is required to run migrations.");
}

const root = dirname(dirname(dirname(fileURLToPath(import.meta.url))));
const migrationPath = join(root, "migrations", "001_auth.sql");
const sql = await readFile(migrationPath, "utf8");

const pool = new Pool(poolConfig(databaseUrl, 1));

try {
  await pool.query(sql);
  console.log("Applied migration: 001_auth.sql");
} finally {
  await pool.end();
}
