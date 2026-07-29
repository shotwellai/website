import type { PoolConfig } from "pg";

export function poolConfig(databaseUrl: string, max = 10): PoolConfig {
  const url = new URL(databaseUrl);
  const sslmode = url.searchParams.get("sslmode");
  const useExplicitSsl = sslmode === "require" || sslmode === "prefer";

  if (useExplicitSsl) {
    url.searchParams.delete("sslmode");
  }

  return {
    connectionString: url.toString(),
    max,
    ssl: useExplicitSsl ? { rejectUnauthorized: false } : undefined
  };
}
