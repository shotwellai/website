import express, { type ErrorRequestHandler } from "express";
import cookieParser from "cookie-parser";
import helmet from "helmet";
import morgan from "morgan";

import { config } from "./config.js";
import { appRouter } from "./routes/app.js";
import { authRouter } from "./routes/auth.js";
import { messagePage } from "./http/render.js";

const app = express();

app.disable("x-powered-by");
app.set("trust proxy", 1);

app.use(
  helmet({
    contentSecurityPolicy: {
      directives: {
        defaultSrc: ["'self'"],
        baseUri: ["'self'"],
        formAction: ["'self'"],
        frameAncestors: ["'none'"],
        imgSrc: ["'self'", config.publicSiteUrl.origin, "https:", "data:"],
        objectSrc: ["'none'"],
        connectSrc: ["'self'", "https://storage.googleapis.com"],
        scriptSrc: ["'self'"],
        styleSrc: ["'self'", "'unsafe-inline'", "https://fonts.googleapis.com"],
        fontSrc: ["'self'", "https://fonts.gstatic.com"],
        upgradeInsecureRequests: config.isProduction ? [] : null
      }
    }
  })
);

app.use(morgan(config.isProduction ? "combined" : "dev"));
app.use(cookieParser(config.sessionSecret));
app.use(express.urlencoded({ extended: false }));
app.use(express.json({ limit: "1mb" }));

app.get("/theme.js", (_req, res) => {
  res.type("application/javascript").send(`"use strict";
(function () {
  var KEY = "shotwell-platform-theme";
  var saved = null;
  try { saved = localStorage.getItem(KEY); } catch (e) {}
  var theme = saved || (window.matchMedia && matchMedia("(prefers-color-scheme: light)").matches ? "light" : "dark");

  function apply(next) {
    document.documentElement.setAttribute("data-theme", next);
    try { localStorage.setItem(KEY, next); } catch (e) {}
    var btn = document.getElementById("themeToggle");
    if (btn) {
      btn.textContent = next === "light" ? "\u263E" : "\u2600";
      btn.setAttribute("aria-label", next === "light" ? "Switch to dark mode" : "Switch to light mode");
    }
    document.querySelectorAll(".brand-mark").forEach(function (img) {
      var src = next === "light" ? img.dataset.markLight : img.dataset.markDark;
      if (src) img.src = src;
    });
  }

  // set the attribute immediately so the first paint has the right theme
  document.documentElement.setAttribute("data-theme", theme);

  function flip() {
    apply(document.documentElement.getAttribute("data-theme") === "light" ? "dark" : "light");
  }

  function wireLongPress(el) {
    var timer = null, fired = false;
    el.addEventListener("pointerdown", function () {
      fired = false;
      timer = setTimeout(function () { fired = true; flip(); }, 600);
    });
    ["pointerup", "pointerleave", "pointercancel"].forEach(function (ev) {
      el.addEventListener(ev, function () { clearTimeout(timer); });
    });
    el.addEventListener("click", function (e) {
      if (fired) { e.preventDefault(); fired = false; }
    });
    el.addEventListener("contextmenu", function (e) {
      if (fired) e.preventDefault();
    });
  }

  document.addEventListener("DOMContentLoaded", function () {
    apply(theme);
    var btn = document.getElementById("themeToggle");
    if (btn) btn.addEventListener("click", flip);
    document.querySelectorAll(".brand").forEach(wireLongPress);
  });
})();
`);
});

app.get("/healthz", (_req, res) => {
  res.json({
    ok: true,
    service: "shotwell-platform",
    environment: config.nodeEnv,
    authStore: config.authStore,
    databaseConfigured: Boolean(config.databaseUrl),
    googleConfigured: Boolean(config.google.clientId && config.google.clientSecret),
    uploadsConfigured: Boolean(config.uploads.bucketName)
  });
});

app.use(authRouter);
app.use(appRouter);

app.use((_req, res) => {
  res.status(404).type("html").send(messagePage("Not found", "That page does not exist."));
});

const errorHandler: ErrorRequestHandler = (error, _req, res, _next) => {
  console.error(error);
  res
    .status(500)
    .type("html")
    .send(messagePage("Something went wrong", "The request could not be completed."));
};

app.use(errorHandler);

app.listen(config.port, () => {
  console.log(`shotwell-platform listening on http://localhost:${config.port}`);
});
