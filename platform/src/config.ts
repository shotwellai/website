import "dotenv/config";

import { randomBytes } from "node:crypto";
import { z } from "zod";

const booleanFromEnv = z.preprocess((value) => {
  if (typeof value !== "string") {
    return value;
  }

  return ["1", "true", "yes", "on"].includes(value.toLowerCase());
}, z.boolean());

const envSchema = z.object({
  NODE_ENV: z.enum(["development", "test", "production"]).default("development"),
  PORT: z.coerce.number().int().positive().default(3000),
  PUBLIC_SITE_URL: z.string().url().default("https://shotwell.ai"),
  AUTH_BASE_URL: z.string().url().default("http://localhost:3000"),
  APP_BASE_URL: z.string().url().default("http://localhost:3000"),
  SESSION_SECRET: z.string().min(32).optional(),
  AUTH_STORE: z.enum(["memory", "postgres"]).optional(),
  DATABASE_URL: z.string().optional(),
  GOOGLE_CLIENT_ID: z.string().optional(),
  GOOGLE_CLIENT_SECRET: z.string().optional(),
  GOOGLE_ALLOWED_DOMAIN: z.string().optional(),
  EMAIL_FROM: z.string().optional(),
  EMAIL_REPLY_TO: z.string().optional(),
  RESEND_API_KEY: z.string().optional(),
  ALLOW_DEV_EMAIL_LOGIN: booleanFromEnv.default(false)
});

const parsedEnv = envSchema.parse(process.env);
const generatedDevSecret = randomBytes(32).toString("base64url");

if (parsedEnv.NODE_ENV === "production" && !parsedEnv.SESSION_SECRET) {
  throw new Error("SESSION_SECRET is required in production.");
}

export const config = {
  nodeEnv: parsedEnv.NODE_ENV,
  isProduction: parsedEnv.NODE_ENV === "production",
  port: parsedEnv.PORT,
  publicSiteUrl: new URL(parsedEnv.PUBLIC_SITE_URL),
  authBaseUrl: new URL(parsedEnv.AUTH_BASE_URL),
  appBaseUrl: new URL(parsedEnv.APP_BASE_URL),
  sessionSecret: parsedEnv.SESSION_SECRET ?? generatedDevSecret,
  authStore: parsedEnv.AUTH_STORE ?? (parsedEnv.NODE_ENV === "production" && parsedEnv.DATABASE_URL ? "postgres" : "memory"),
  databaseUrl: parsedEnv.DATABASE_URL,
  google: {
    clientId: parsedEnv.GOOGLE_CLIENT_ID,
    clientSecret: parsedEnv.GOOGLE_CLIENT_SECRET,
    allowedDomain: parsedEnv.GOOGLE_ALLOWED_DOMAIN
  },
  email: {
    from: parsedEnv.EMAIL_FROM,
    replyTo: parsedEnv.EMAIL_REPLY_TO,
    resendApiKey: parsedEnv.RESEND_API_KEY
  },
  allowDevEmailLogin:
    parsedEnv.ALLOW_DEV_EMAIL_LOGIN || parsedEnv.NODE_ENV !== "production"
} as const;

export type AppConfig = typeof config;
