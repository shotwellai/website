import type { Request } from "express";

import { config } from "../config.js";

export function requestOrigin(req: Request) {
  const protocol = req.get("x-forwarded-proto") ?? req.protocol;
  return `${protocol}://${req.get("host")}`;
}

export function isAuthHost(req: Request) {
  return req.hostname === config.authBaseUrl.hostname || req.hostname.startsWith("auth.");
}

export function isAppHost(req: Request) {
  return req.hostname === config.appBaseUrl.hostname || req.hostname.startsWith("app.");
}

export function defaultReturnTo() {
  return new URL("/auth/complete", config.appBaseUrl).toString();
}

export function normalizeReturnTo(value: unknown) {
  if (typeof value !== "string" || !value) {
    return defaultReturnTo();
  }

  try {
    const url = new URL(value, config.appBaseUrl);
    const allowedOrigins = new Set([
      config.appBaseUrl.origin,
      config.authBaseUrl.origin,
      "http://localhost:3000"
    ]);

    if (!allowedOrigins.has(url.origin)) {
      return defaultReturnTo();
    }

    return url.toString();
  } catch {
    return defaultReturnTo();
  }
}

export function withCode(returnTo: string, code: string) {
  const url = new URL(returnTo);
  url.searchParams.set("code", code);
  return url.toString();
}
