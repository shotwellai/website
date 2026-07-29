import { Router, type Response } from "express";
import rateLimit from "express-rate-limit";

import { clearSessionCookie, readSessionCookie, setSessionCookie } from "../auth/cookies.js";
import {
  buildGoogleAuthorizationUrl,
  exchangeGoogleCode,
  hasGoogleConfig
} from "../auth/google.js";
import { authStore, type User } from "../auth/store.js";
import { config } from "../config.js";
import { defaultReturnTo, normalizeReturnTo, withCode } from "../http/hosts.js";
import { loginPage, messagePage } from "../http/render.js";

const emailLimiter = rateLimit({
  windowMs: 10 * 60 * 1000,
  limit: 10,
  standardHeaders: true,
  legacyHeaders: false
});

export const authRouter = Router();

async function finishLogin(user: User, returnTo: string, response: Response) {
  const authSession = await authStore.createSession(user.id, "auth");
  setSessionCookie(response, "auth", authSession.id);

  const handoff = await authStore.createHandoff(user.id, returnTo);
  response.redirect(303, withCode(returnTo, handoff.code));
}

authRouter.get("/login", (req, res) => {
  const returnTo = normalizeReturnTo(req.query.return_to);
  res.type("html").send(
    loginPage({
      returnTo,
      googleEnabled: hasGoogleConfig(),
      devEmailEnabled: config.allowDevEmailLogin,
      error: typeof req.query.error === "string" ? req.query.error : undefined
    })
  );
});

authRouter.post("/login/email", emailLimiter, async (req, res, next) => {
  try {
    const returnTo = normalizeReturnTo(req.body.return_to);
    const email = typeof req.body.email === "string" ? req.body.email.trim().toLowerCase() : "";

    if (!email || !email.includes("@")) {
      res.status(400).type("html").send(messagePage("Email required", "Enter a valid email address.", "/login", "Back"));
      return;
    }

    if (!config.allowDevEmailLogin) {
      res
        .status(501)
        .type("html")
        .send(messagePage("Email pending", "Email delivery is not configured for this environment.", "/login", "Back"));
      return;
    }

    const user = await authStore.upsertUser({
      email,
      provider: "email",
      name: email.split("@")[0]
    });

    await finishLogin(user, returnTo, res);
  } catch (error) {
    next(error);
  }
});

authRouter.get("/login/google", async (req, res, next) => {
  try {
    if (!hasGoogleConfig()) {
      res
        .status(501)
        .type("html")
        .send(messagePage("Google SSO pending", "Google OAuth credentials are not configured yet.", "/login", "Back"));
      return;
    }

    const returnTo = normalizeReturnTo(req.query.return_to);
    const oauthState = await authStore.createOAuthState(returnTo);
    res.redirect(302, buildGoogleAuthorizationUrl(oauthState));
  } catch (error) {
    next(error);
  }
});

authRouter.get("/auth/google/callback", async (req, res, next) => {
  try {
    const code = typeof req.query.code === "string" ? req.query.code : "";
    const state = typeof req.query.state === "string" ? req.query.state : "";
    const oauthState = await authStore.consumeOAuthState(state);

    if (!code || !oauthState) {
      res
        .status(400)
        .type("html")
        .send(messagePage("Sign-in expired", "Start the Google sign-in flow again.", "/login", "Back"));
      return;
    }

    const profile = await exchangeGoogleCode({
      code,
      nonce: oauthState.nonce
    });

    if (!profile.emailVerified) {
      res
        .status(403)
        .type("html")
        .send(messagePage("Email not verified", "Use a verified Google account to continue.", "/login", "Back"));
      return;
    }

    const user = await authStore.upsertUser({
      email: profile.email,
      name: profile.name,
      avatarUrl: profile.avatarUrl,
      provider: "google",
      providerSubject: profile.subject
    });

    await finishLogin(user, oauthState.returnTo, res);
  } catch (error) {
    next(error);
  }
});

authRouter.post("/logout", async (req, res, next) => {
  try {
    await authStore.deleteSession(readSessionCookie(req, "auth"));
    await authStore.deleteSession(readSessionCookie(req, "app"));
    clearSessionCookie(res, "auth");
    clearSessionCookie(res, "app");
    res.redirect(303, defaultReturnTo());
  } catch (error) {
    next(error);
  }
});
