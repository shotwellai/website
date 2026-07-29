import type { Request, Response } from "express";

import { config } from "../config.js";
import type { SessionKind } from "./store.js";

const devCookieNames: Record<SessionKind, string> = {
  auth: "shotwell_auth_dev",
  app: "shotwell_app_dev"
};

const prodCookieNames: Record<SessionKind, string> = {
  auth: "__Host-shotwell_auth",
  app: "__Host-shotwell_app"
};

export const cookieName = (kind: SessionKind) =>
  config.isProduction ? prodCookieNames[kind] : devCookieNames[kind];

export function readSessionCookie(req: Request, kind: SessionKind) {
  return req.cookies?.[cookieName(kind)] as string | undefined;
}

export function setSessionCookie(res: Response, kind: SessionKind, sessionId: string) {
  res.cookie(cookieName(kind), sessionId, {
    httpOnly: true,
    secure: config.isProduction,
    sameSite: "lax",
    path: "/",
    maxAge: 14 * 24 * 60 * 60 * 1000
  });
}

export function clearSessionCookie(res: Response, kind: SessionKind) {
  res.clearCookie(cookieName(kind), {
    httpOnly: true,
    secure: config.isProduction,
    sameSite: "lax",
    path: "/"
  });
}
