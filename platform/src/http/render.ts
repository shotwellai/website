import type { User } from "../auth/store.js";
import { config } from "../config.js";

export function escapeHtml(value: string) {
  return value.replace(/[&<>"']/g, (char) => {
    switch (char) {
      case "&":
        return "&amp;";
      case "<":
        return "&lt;";
      case ">":
        return "&gt;";
      case '"':
        return "&quot;";
      case "'":
        return "&#39;";
      default:
        return char;
    }
  });
}

export function page(title: string, body: string) {
  return `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>${escapeHtml(title)}</title>
  <style>
    :root {
      color-scheme: light;
      --ink: #171717;
      --muted: #666b73;
      --line: #d9dee7;
      --surface: #ffffff;
      --wash: #f4f6f8;
      --accent: #0f7a67;
      --accent-ink: #ffffff;
      --warn: #a35d00;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--wash);
      color: var(--ink);
      font: 15px/1.5 Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    a { color: inherit; }
    .shell {
      min-height: 100vh;
      display: grid;
      grid-template-rows: auto 1fr;
    }
    .topbar {
      height: 64px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 24px;
      padding: 0 28px;
      border-bottom: 1px solid var(--line);
      background: rgba(255, 255, 255, 0.92);
      backdrop-filter: blur(12px);
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 10px;
      font-weight: 700;
      letter-spacing: 0;
    }
    .mark {
      width: 28px;
      height: 28px;
      display: inline-grid;
      place-items: center;
      border: 1px solid #151515;
      border-radius: 6px;
      font-size: 13px;
      line-height: 1;
    }
    main {
      width: min(1120px, calc(100vw - 32px));
      margin: 0 auto;
      padding: 34px 0 56px;
    }
    .auth-main {
      width: min(420px, calc(100vw - 32px));
      display: grid;
      align-content: center;
      min-height: calc(100vh - 64px);
      padding: 24px 0;
    }
    .panel {
      background: var(--surface);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 24px;
      box-shadow: 0 1px 2px rgba(15, 23, 42, 0.04);
    }
    h1 {
      margin: 0 0 8px;
      font-size: 28px;
      line-height: 1.16;
      letter-spacing: 0;
    }
    h2 {
      margin: 0;
      font-size: 16px;
      line-height: 1.25;
      letter-spacing: 0;
    }
    p {
      margin: 0;
      color: var(--muted);
    }
    form {
      display: grid;
      gap: 12px;
      margin-top: 22px;
    }
    label {
      display: grid;
      gap: 6px;
      color: #2f3338;
      font-size: 13px;
      font-weight: 600;
    }
    input {
      width: 100%;
      height: 44px;
      border: 1px solid #c8ced8;
      border-radius: 6px;
      padding: 0 12px;
      background: #fff;
      color: var(--ink);
      font: inherit;
    }
    button,
    .button {
      min-height: 42px;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      border: 1px solid #131313;
      border-radius: 6px;
      padding: 0 14px;
      background: #131313;
      color: #fff;
      font: inherit;
      font-weight: 700;
      text-decoration: none;
      cursor: pointer;
    }
    .button.secondary {
      background: #fff;
      color: var(--ink);
      border-color: #c8ced8;
    }
    .button[aria-disabled="true"] {
      color: #8b919b;
      background: #eef1f5;
      border-color: #d4d9e1;
      pointer-events: none;
    }
    .divider {
      display: grid;
      grid-template-columns: 1fr auto 1fr;
      align-items: center;
      gap: 12px;
      margin: 18px 0;
      color: #7a808a;
      font-size: 12px;
      text-transform: uppercase;
    }
    .divider::before,
    .divider::after {
      content: "";
      border-top: 1px solid var(--line);
    }
    .toolbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 24px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 14px;
    }
    .card {
      min-height: 132px;
      display: grid;
      align-content: space-between;
      gap: 18px;
      background: var(--surface);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 18px;
    }
    .metric {
      font-size: 30px;
      line-height: 1;
      font-weight: 760;
      letter-spacing: 0;
    }
    .muted { color: var(--muted); }
    .notice {
      margin-top: 16px;
      padding: 12px;
      border: 1px solid #e7c579;
      border-radius: 6px;
      background: #fff8e8;
      color: var(--warn);
    }
    .identity {
      display: flex;
      align-items: center;
      gap: 10px;
      color: var(--muted);
      font-size: 14px;
    }
    .avatar {
      width: 32px;
      height: 32px;
      border-radius: 999px;
      object-fit: cover;
      background: #e3e7ed;
    }
    @media (max-width: 780px) {
      .topbar { padding: 0 16px; }
      main { width: min(100vw - 24px, 1120px); padding-top: 24px; }
      .toolbar { align-items: flex-start; flex-direction: column; }
      .grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
    @media (max-width: 480px) {
      .grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <header class="topbar">
      <a class="brand" href="${config.publicSiteUrl.toString()}">
        <span class="mark">S</span>
        <span>Shotwell</span>
      </a>
      <nav>
        <a class="button secondary" href="${config.publicSiteUrl.toString()}">Main Site</a>
      </nav>
    </header>
    ${body}
  </div>
</body>
</html>`;
}

export function loginPage(input: {
  returnTo: string;
  googleEnabled: boolean;
  emailEnabled: boolean;
  devEmailEnabled: boolean;
  error?: string;
}) {
  const googleButton = input.googleEnabled
    ? `<a class="button secondary" href="/login/google?return_to=${encodeURIComponent(input.returnTo)}">Continue with Google</a>`
    : `<a class="button secondary" aria-disabled="true">Google SSO</a>`;

  return page(
    "Shotwell Login",
    `<main class="auth-main">
      <section class="panel">
        <h1>Sign in to Shotwell</h1>
        <p>Use a Shotwell account to continue.</p>
        ${input.error ? `<div class="notice">${escapeHtml(input.error)}</div>` : ""}
        <div style="display: grid; gap: 10px; margin-top: 22px;">${googleButton}</div>
        <div class="divider">or</div>
        <form method="post" action="/login/email">
          <input type="hidden" name="return_to" value="${escapeHtml(input.returnTo)}">
          <label>
            Email
            <input name="email" type="email" autocomplete="email" placeholder="you@company.com" required>
          </label>
          <button type="submit">Continue by email</button>
        </form>
        ${
          input.devEmailEnabled
            ? `<div class="notice">Development email login is enabled.</div>`
            : input.emailEnabled
              ? `<div class="notice">We'll email you a secure sign-in link.</div>`
            : `<div class="notice">Email delivery is not configured yet.</div>`
        }
      </section>
    </main>`
  );
}

export function appPage(user: User) {
  const displayName = user.name ?? user.email;
  const avatar = user.avatarUrl
    ? `<img class="avatar" src="${escapeHtml(user.avatarUrl)}" alt="">`
    : `<span class="avatar"></span>`;

  return page(
    "Shotwell App",
    `<main>
      <div class="toolbar">
        <div>
          <h1>Operations</h1>
          <p>Review queues, model runs, and account activity.</p>
        </div>
        <form method="post" action="/logout" style="margin: 0;">
          <button class="button secondary" type="submit">Sign out</button>
        </form>
      </div>
      <div class="identity">${avatar}<span>${escapeHtml(displayName)}</span></div>
      <section class="grid" style="margin-top: 24px;">
        <article class="card">
          <h2>Intake Queue</h2>
          <div>
            <div class="metric">24</div>
            <p class="muted">items pending</p>
          </div>
        </article>
        <article class="card">
          <h2>Review Runs</h2>
          <div>
            <div class="metric">8</div>
            <p class="muted">active batches</p>
          </div>
        </article>
        <article class="card">
          <h2>Model Outputs</h2>
          <div>
            <div class="metric">96%</div>
            <p class="muted">ready rate</p>
          </div>
        </article>
        <article class="card">
          <h2>Account Health</h2>
          <div>
            <div class="metric">OK</div>
            <p class="muted">all systems normal</p>
          </div>
        </article>
      </section>
    </main>`
  );
}

export function messagePage(title: string, message: string, href = "/", label = "Continue") {
  return page(
    title,
    `<main class="auth-main">
      <section class="panel">
        <h1>${escapeHtml(title)}</h1>
        <p>${escapeHtml(message)}</p>
        <div style="margin-top: 22px;">
          <a class="button" href="${escapeHtml(href)}">${escapeHtml(label)}</a>
        </div>
      </section>
    </main>`
  );
}
