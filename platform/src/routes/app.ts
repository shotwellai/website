import { Router } from "express";

import { readSessionCookie, setSessionCookie } from "../auth/cookies.js";
import { authStore } from "../auth/store.js";
import { config } from "../config.js";
import { appPage, messagePage } from "../http/render.js";

export const appRouter = Router();

function loginUrl() {
  const url = new URL("/login", config.authBaseUrl);
  url.searchParams.set("return_to", new URL("/auth/complete", config.appBaseUrl).toString());
  return url.toString();
}

appRouter.get("/", async (req, res, next) => {
  try {
    const session = await authStore.getSession(readSessionCookie(req, "app"), "app");

    if (!session) {
      res.redirect(302, loginUrl());
      return;
    }

    const user = await authStore.getUser(session.userId);
    if (!user) {
      res.redirect(302, loginUrl());
      return;
    }

    res.type("html").send(appPage(user));
  } catch (error) {
    next(error);
  }
});

appRouter.get("/auth/complete", async (req, res, next) => {
  try {
    const code = typeof req.query.code === "string" ? req.query.code : "";
    const handoff = await authStore.consumeHandoff(code);

    if (!handoff) {
      res
        .status(400)
        .type("html")
        .send(messagePage("Sign-in expired", "Start the sign-in flow again.", loginUrl(), "Sign in"));
      return;
    }

    const session = await authStore.createSession(handoff.userId, "app");
    setSessionCookie(res, "app", session.id);
    res.redirect(303, "/");
  } catch (error) {
    next(error);
  }
});

appRouter.get("/api/me", async (req, res, next) => {
  try {
    const session = await authStore.getSession(readSessionCookie(req, "app"), "app");

    if (!session) {
      res.status(401).json({ error: "unauthorized" });
      return;
    }

    const user = await authStore.getUser(session.userId);
    if (!user) {
      res.status(401).json({ error: "unauthorized" });
      return;
    }

    res.json({
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        avatarUrl: user.avatarUrl
      }
    });
  } catch (error) {
    next(error);
  }
});
