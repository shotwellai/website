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

function publicAsset(path: string) {
  return new URL(path, config.publicSiteUrl).toString();
}

export function page(title: string, body: string) {
  const publicSiteUrl = config.publicSiteUrl.toString();
  const logoUrl = publicAsset("shotwell-logo.png");

  return `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>${escapeHtml(title)}</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Instrument+Serif:ital@0;1&family=Outfit:wght@300;400;500;600;700&display=swap" rel="stylesheet">
  <style>
    :root {
      color-scheme: light;
      --color-bg: #f5f5f5;
      --color-bg-alt: #f4f4f4;
      --color-surface: #fdfaf7;
      --color-panel: rgba(253, 250, 247, 0.74);
      --color-text: #2a2a32;
      --color-text-light: #494952;
      --color-muted: #73737b;
      --color-line: rgba(42, 42, 50, 0.12);
      --color-line-strong: rgba(42, 42, 50, 0.22);
      --color-cta: #4f9e84;
      --color-cta-dark: #3d846d;
      --color-cta-text: #f4f4f4;
      --color-warn: #9d6500;
      --color-warn-bg: #fff7e6;
      --font-display: "Instrument Serif", Georgia, serif;
      --font-body: "Outfit", system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      --ease-out: cubic-bezier(0.16, 1, 0.3, 1);
      --nav-height: 72px;
      --max-width: 1280px;
    }

    *,
    *::before,
    *::after {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background:
        linear-gradient(180deg, rgba(255, 255, 255, 0.58), rgba(245, 245, 245, 0) 280px),
        var(--color-bg);
      color: var(--color-text);
      font: 15px/1.6 var(--font-body);
      -webkit-font-smoothing: antialiased;
      -moz-osx-font-smoothing: grayscale;
    }

    a {
      color: inherit;
      text-decoration: none;
    }

    .noise {
      position: fixed;
      inset: 0;
      z-index: 9999;
      width: 100%;
      height: 100%;
      pointer-events: none;
      opacity: 0.035;
      mix-blend-mode: multiply;
    }

    .shell {
      min-height: 100vh;
      display: grid;
      grid-template-rows: auto 1fr;
    }

    .topbar {
      position: sticky;
      top: 0;
      z-index: 20;
      height: var(--nav-height);
      border-bottom: 1px solid rgba(42, 42, 50, 0.06);
      background: rgba(245, 245, 245, 0.92);
      backdrop-filter: blur(16px);
      -webkit-backdrop-filter: blur(16px);
    }

    .topbar-inner {
      max-width: var(--max-width);
      height: 100%;
      margin: 0 auto;
      padding: 0 32px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 24px;
    }

    .brand {
      display: inline-flex;
      align-items: flex-end;
      gap: 6px;
      color: var(--color-text);
      font-family: var(--font-display);
      font-size: 1.85rem;
      line-height: 1;
      letter-spacing: -0.01em;
    }

    .brand-mark {
      width: 44px;
      height: 44px;
      object-fit: contain;
      display: block;
      border-radius: 8px;
    }

    .brand-dot {
      color: var(--color-cta);
    }

    main {
      width: min(var(--max-width), calc(100vw - 64px));
      margin: 0 auto;
      padding: 36px 0 64px;
    }

    h1,
    h2,
    h3,
    p {
      margin: 0;
    }

    h1 {
      font-family: var(--font-display);
      font-size: clamp(2.2rem, 5vw, 4.35rem);
      font-weight: 400;
      line-height: 0.98;
      letter-spacing: -0.02em;
    }

    h1 em,
    h2 em {
      font-style: italic;
    }

    h2 {
      font-family: var(--font-display);
      font-size: clamp(1.65rem, 3vw, 2.7rem);
      font-weight: 400;
      line-height: 1.04;
      letter-spacing: -0.02em;
    }

    h3 {
      font-size: 1rem;
      line-height: 1.25;
      letter-spacing: 0;
    }

    p {
      color: var(--color-text-light);
      font-weight: 300;
    }

    .eyebrow {
      display: inline-flex;
      align-items: center;
      width: max-content;
      max-width: 100%;
      border: 1px solid var(--color-line-strong);
      border-radius: 2px;
      padding: 6px 12px;
      color: var(--color-text);
      font-size: 0.72rem;
      font-weight: 600;
      letter-spacing: 0.15em;
      line-height: 1;
      text-transform: uppercase;
    }

    .button,
    button {
      min-height: 44px;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 10px;
      border: 1px solid var(--color-cta);
      border-radius: 2px;
      padding: 0 22px;
      background: var(--color-cta);
      color: var(--color-cta-text);
      font: inherit;
      font-size: 0.82rem;
      font-weight: 500;
      letter-spacing: 0.04em;
      line-height: 1;
      text-transform: uppercase;
      text-decoration: none;
      cursor: pointer;
      transition:
        background 0.35s var(--ease-out),
        border-color 0.35s var(--ease-out),
        color 0.35s var(--ease-out),
        transform 0.35s var(--ease-out);
    }

    .button:hover,
    button:hover {
      background: var(--color-cta-dark);
      border-color: var(--color-cta-dark);
      color: var(--color-cta-text);
    }

    .button.secondary,
    button.secondary {
      background: transparent;
      color: var(--color-cta);
      border-color: var(--color-cta);
    }

    .button.secondary:hover,
    button.secondary:hover {
      background: rgba(79, 158, 132, 0.08);
      color: var(--color-cta);
      border-color: var(--color-cta);
    }

    .button.compact,
    button.compact {
      min-height: 34px;
      padding: 0 12px;
      font-size: 0.68rem;
    }

    .button.full,
    button.full {
      width: 100%;
    }

    .button[aria-disabled="true"],
    button:disabled {
      color: rgba(42, 42, 50, 0.44);
      background: rgba(42, 42, 50, 0.06);
      border-color: rgba(42, 42, 50, 0.1);
      pointer-events: none;
    }

    .panel {
      background: var(--color-panel);
      border: 1px solid var(--color-line);
      border-radius: 8px;
      padding: 24px;
      box-shadow: 0 1px 2px rgba(42, 42, 50, 0.04);
    }

    .panel-heading {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 18px;
      margin-bottom: 18px;
    }

    .panel-kicker {
      color: var(--color-muted);
      font-size: 0.72rem;
      font-weight: 600;
      letter-spacing: 0.14em;
      line-height: 1;
      text-transform: uppercase;
    }

    form {
      display: grid;
      gap: 14px;
      margin: 0;
    }

    label,
    .field {
      display: grid;
      gap: 7px;
      color: var(--color-text);
      font-size: 0.78rem;
      font-weight: 500;
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }

    input,
    textarea,
    select {
      width: 100%;
      border: 1px solid rgba(42, 42, 50, 0.16);
      border-radius: 6px;
      background: rgba(255, 255, 255, 0.72);
      color: var(--color-text);
      font: inherit;
      font-size: 0.96rem;
      letter-spacing: 0;
      text-transform: none;
      outline: none;
      transition:
        border-color 0.2s var(--ease-out),
        box-shadow 0.2s var(--ease-out),
        background 0.2s var(--ease-out);
    }

    input,
    select {
      height: 46px;
      padding: 0 12px;
    }

    input[type="file"] {
      height: auto;
      min-height: 46px;
      padding: 10px 12px;
    }

    textarea {
      min-height: 140px;
      resize: vertical;
      padding: 12px;
    }

    input:focus,
    textarea:focus,
    select:focus {
      background: #ffffff;
      border-color: rgba(79, 158, 132, 0.72);
      box-shadow: 0 0 0 3px rgba(79, 158, 132, 0.12);
    }

    .divider {
      display: grid;
      grid-template-columns: 1fr auto 1fr;
      align-items: center;
      gap: 12px;
      margin: 20px 0;
      color: var(--color-muted);
      font-size: 0.7rem;
      font-weight: 600;
      letter-spacing: 0.14em;
      text-transform: uppercase;
    }

    .divider::before,
    .divider::after {
      content: "";
      border-top: 1px solid var(--color-line);
    }

    .notice {
      margin-top: 16px;
      border: 1px solid rgba(157, 101, 0, 0.22);
      border-radius: 6px;
      padding: 12px;
      background: var(--color-warn-bg);
      color: var(--color-warn);
      font-size: 0.9rem;
      line-height: 1.45;
    }

    .notice.neutral {
      background: rgba(79, 158, 132, 0.08);
      border-color: rgba(79, 158, 132, 0.22);
      color: var(--color-cta-dark);
    }

    .inline-link {
      color: var(--color-cta-dark);
      text-decoration: underline;
      text-decoration-thickness: 1px;
      text-underline-offset: 0.18em;
    }

    .auth-main {
      min-height: calc(100vh - var(--nav-height));
      display: grid;
      grid-template-columns: minmax(320px, 0.78fr) minmax(0, 1fr);
      align-items: center;
      gap: 40px;
      padding-top: 42px;
      padding-bottom: 72px;
    }

    .auth-main-single {
      grid-template-columns: minmax(320px, 560px);
      justify-content: center;
    }

    .login-panel {
      background: rgba(253, 250, 247, 0.92);
    }

    .login-panel h1 {
      font-size: clamp(2rem, 3.5vw, 3.25rem);
      margin-bottom: 10px;
    }

    .auth-actions {
      display: grid;
      gap: 10px;
      margin-top: 24px;
    }

    .auth-aside {
      display: grid;
      gap: 24px;
      padding: 8px 0;
    }

    .auth-aside p {
      max-width: 42rem;
      font-size: clamp(1rem, 1.6vw, 1.14rem);
    }

    .mini-flow {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }

    .mini-step {
      min-height: 156px;
      display: grid;
      align-content: space-between;
      gap: 22px;
      border: 1px solid var(--color-line);
      border-radius: 8px;
      background: rgba(253, 250, 247, 0.5);
      padding: 18px;
    }

    .mini-step-number {
      color: rgba(42, 42, 50, 0.46);
      font-size: 0.72rem;
      font-weight: 600;
      letter-spacing: 0.14em;
    }

    .mini-step strong {
      display: block;
      margin-bottom: 4px;
      font-size: 1rem;
      line-height: 1.2;
    }

    .mini-step p {
      font-size: 0.88rem;
      line-height: 1.45;
    }

    .app-main {
      display: grid;
      gap: 22px;
    }

    .app-header {
      display: flex;
      justify-content: flex-end;
      align-items: center;
      padding-top: 10px;
    }

    .app-title {
      display: grid;
      gap: 18px;
    }

    .app-title p {
      max-width: 44rem;
      font-size: 1.05rem;
    }

    .account-box {
      display: flex;
      align-items: center;
      gap: 12px;
      border: 1px solid var(--color-line);
      border-radius: 8px;
      padding: 10px;
      background: rgba(253, 250, 247, 0.62);
    }

    .identity {
      min-width: 0;
      display: flex;
      align-items: center;
      gap: 10px;
      color: var(--color-muted);
      font-size: 0.9rem;
    }

    .identity span:last-child {
      max-width: 220px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }

    .avatar {
      width: 34px;
      height: 34px;
      flex: 0 0 34px;
      border-radius: 999px;
      object-fit: cover;
      background: linear-gradient(135deg, rgba(79, 158, 132, 0.22), rgba(42, 42, 50, 0.08));
      border: 1px solid rgba(42, 42, 50, 0.08);
    }

    .workspace-grid {
      display: grid;
      grid-template-columns: minmax(0, 1fr) minmax(340px, 0.45fr);
      gap: 18px;
      align-items: start;
    }

    .upload-panel {
      display: grid;
      gap: 18px;
    }

    .upload-panel form {
      gap: 16px;
    }

    .field-description {
      max-width: 58rem;
      color: var(--color-text-light);
      font-size: 0.92rem;
      font-weight: 300;
      letter-spacing: 0;
      line-height: 1.45;
      text-transform: none;
    }

    .upload-panel textarea {
      min-height: 300px;
    }

    .prompt-tools {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }

    .metric-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }

    .metric-card {
      min-height: 124px;
      display: grid;
      align-content: space-between;
      gap: 18px;
      border: 1px solid var(--color-line);
      border-radius: 8px;
      background: rgba(253, 250, 247, 0.62);
      padding: 18px;
    }

    .metric-value {
      color: var(--color-text);
      font-family: var(--font-display);
      font-size: 2.35rem;
      line-height: 0.95;
    }

    .metric-card p {
      font-size: 0.86rem;
    }

    .job-list {
      display: grid;
      gap: 10px;
    }

    .job-row {
      display: grid;
      grid-template-columns: 1fr;
      align-items: start;
      gap: 16px;
      border: 1px solid var(--color-line);
      border-radius: 8px;
      background: rgba(255, 255, 255, 0.42);
      padding: 14px;
    }

    .job-title {
      display: grid;
      gap: 3px;
      min-width: 0;
    }

    .job-title strong {
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
      line-height: 1.25;
    }

    .job-title span,
    .job-meta,
    .job-output {
      color: var(--color-muted);
      font-size: 0.84rem;
      font-weight: 300;
    }

    .job-row .button {
      width: max-content;
    }

    .status-pill {
      width: max-content;
      border: 1px solid rgba(42, 42, 50, 0.12);
      border-radius: 999px;
      padding: 4px 9px;
      background: rgba(42, 42, 50, 0.04);
      color: var(--color-text-light);
      font-size: 0.72rem;
      font-weight: 600;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }

    .status-pill.ready {
      border-color: rgba(79, 158, 132, 0.24);
      background: rgba(79, 158, 132, 0.12);
      color: var(--color-cta-dark);
    }

    .progress {
      height: 6px;
      margin-top: 8px;
      overflow: hidden;
      border-radius: 999px;
      background: rgba(42, 42, 50, 0.08);
    }

    .progress span {
      display: block;
      height: 100%;
      border-radius: inherit;
      background: var(--color-cta);
    }

    .results-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 14px;
    }

    .result-card {
      min-height: 180px;
      display: grid;
      align-content: space-between;
      gap: 22px;
    }

    .result-card ul {
      display: grid;
      gap: 8px;
      margin: 0;
      padding: 0;
      list-style: none;
      color: var(--color-text-light);
      font-size: 0.9rem;
      font-weight: 300;
    }

    .message-main {
      min-height: calc(100vh - var(--nav-height));
      display: grid;
      place-items: center;
      padding-top: 32px;
      padding-bottom: 64px;
    }

    .message-panel {
      width: min(460px, 100%);
      display: grid;
      gap: 14px;
      background: rgba(253, 250, 247, 0.92);
    }

    .message-panel h1 {
      font-size: clamp(2rem, 4vw, 3rem);
    }

    .policy-main {
      display: grid;
      gap: 22px;
      padding-top: 56px;
    }

    .policy-panel {
      width: min(860px, 100%);
      margin: 0 auto;
      display: grid;
      gap: 24px;
      background: rgba(253, 250, 247, 0.92);
    }

    .policy-panel h1 {
      font-size: clamp(2.3rem, 5vw, 4rem);
    }

    .policy-section {
      display: grid;
      gap: 8px;
    }

    .policy-section h2 {
      font-family: var(--font-body);
      font-size: 1rem;
      font-weight: 600;
      letter-spacing: 0.04em;
      line-height: 1.25;
      text-transform: uppercase;
    }

    .policy-section ul {
      display: grid;
      gap: 8px;
      margin: 0;
      padding-left: 1.2rem;
      color: var(--color-text-light);
      font-weight: 300;
    }

    @media (max-width: 980px) {
      .auth-main,
      .workspace-grid,
      .app-header {
        grid-template-columns: 1fr;
      }

      .app-header {
        align-items: start;
      }

      .account-box {
        width: 100%;
        justify-content: space-between;
      }

      .mini-flow,
      .results-grid {
        grid-template-columns: 1fr;
      }

      .job-row {
        grid-template-columns: 1fr;
        align-items: start;
      }
    }

    @media (max-width: 680px) {
      .topbar-inner {
        padding: 0 18px;
      }

      .brand {
        font-size: 1.5rem;
      }

      .brand-mark {
        width: 38px;
        height: 38px;
      }

      .topbar nav .button {
        min-height: 38px;
        padding: 0 12px;
        font-size: 0.68rem;
      }

      main {
        width: min(100vw - 28px, var(--max-width));
        padding-top: 26px;
      }

      .panel {
        padding: 18px;
      }

      .prompt-tools,
      .metric-grid {
        grid-template-columns: 1fr;
      }

      .account-box {
        align-items: flex-start;
        flex-direction: column;
      }
    }
  </style>
</head>
<body>
  <svg class="noise" aria-hidden="true">
    <filter id="grain">
      <feTurbulence type="fractalNoise" baseFrequency="0.65" numOctaves="3" stitchTiles="stitch"></feTurbulence>
      <feColorMatrix type="saturate" values="0"></feColorMatrix>
    </filter>
    <rect width="100%" height="100%" filter="url(#grain)"></rect>
  </svg>
  <div class="shell">
    <header class="topbar">
      <div class="topbar-inner">
        <a class="brand" href="${escapeHtml(publicSiteUrl)}">
          <img class="brand-mark" src="${escapeHtml(logoUrl)}" alt="Shotwell logo">
          <span>Shotwell<span class="brand-dot">.</span></span>
        </a>
        <nav>
          <a class="button secondary" href="${escapeHtml(publicSiteUrl)}">Main Site</a>
        </nav>
      </div>
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
    ? `<a class="button secondary full" href="/login/google?return_to=${encodeURIComponent(input.returnTo)}">Continue with Google</a>`
    : `<a class="button secondary full" aria-disabled="true">Google SSO</a>`;

  const emailNotice = input.devEmailEnabled
    ? `<div class="notice neutral">Development email login is enabled.</div>`
    : input.emailEnabled
      ? ""
      : `<div class="notice">Email delivery is not configured yet.</div>`;

  return page(
    "Shotwell Login",
    `<main class="auth-main auth-main-single">
      <section class="panel login-panel">
        <h1>Sign in to start uploading.</h1>
        <p>Login with your company email. Get 10 hours of robot video annotated for free, with 24 hour turnaround time. <a class="inline-link" href="/privacy">Zero Data Retention.</a></p>
        ${input.error ? `<div class="notice">${escapeHtml(input.error)}</div>` : ""}
        <div class="auth-actions">${googleButton}</div>
        <div class="divider">or</div>
        <form method="post" action="/login/email">
          <input type="hidden" name="return_to" value="${escapeHtml(input.returnTo)}">
          <label>
            Email
            <input name="email" type="email" autocomplete="email" placeholder="you@example.com" required>
          </label>
          <button class="full" type="submit">Continue by email</button>
        </form>
        ${emailNotice}
      </section>
    </main>`
  );
}

export function privacyPolicyPage() {
  return page(
    "Shotwell Privacy Policy",
    `<main class="policy-main">
      <section class="panel policy-panel">
        <span class="eyebrow">Privacy Policy</span>
        <h1>Zero Data Retention.</h1>
        <p>Last updated: July 30, 2026</p>
        <section class="policy-section">
          <h2>Summary</h2>
          <p>Shotwell processes robot episode files, prompts, and generated annotations only to provide the annotation service requested by you. We do not retain uploaded customer data after processing and delivery, and we do not use it to train models.</p>
        </section>
        <section class="policy-section">
          <h2>Customer Data</h2>
          <p>Customer Data means robot episode files, raw video files, MCAP files, prompts, instructions, metadata included with an upload, generated labels, quality reports, and other output artifacts produced for that upload.</p>
        </section>
        <section class="policy-section">
          <h2>Zero Retention Commitment</h2>
          <ul>
            <li>We use Customer Data only to process your upload and produce your requested annotation results.</li>
            <li>We delete Customer Data after processing and delivery. Any transient copies exist only while the requested job is running or results are being transmitted to you.</li>
            <li>We do not use Customer Data to train, fine-tune, improve, evaluate, or benchmark any model.</li>
            <li>We do not sell, rent, disclose, or share Customer Data with third parties for advertising, model training, or unrelated analytics.</li>
          </ul>
        </section>
        <section class="policy-section">
          <h2>Operational Data</h2>
          <p>We may retain limited account, authentication, billing, security, and service log information as needed to operate the service, prevent abuse, debug failures, satisfy legal obligations, and maintain business records. Operational Data does not include your uploaded robot episode content or annotation outputs.</p>
        </section>
        <section class="policy-section">
          <h2>Vendors</h2>
          <p>We may use infrastructure and service providers to host, secure, transmit, or process uploads on our behalf. They are permitted to process Customer Data only as needed to provide Shotwell's service and not for their own model training or advertising purposes.</p>
        </section>
        <section class="policy-section">
          <h2>Deletion</h2>
          <p>You can request deletion of Customer Data, account records, or related operational records by contacting Shotwell. We will delete eligible records from active systems unless we are required to keep them for security, legal, compliance, or dispute-resolution reasons.</p>
        </section>
        <section class="policy-section">
          <h2>Contact</h2>
          <p>Questions about this policy can be sent to <a class="inline-link" href="mailto:hello@shotwell.ai">hello@shotwell.ai</a>.</p>
        </section>
      </section>
    </main>`
  );
}

export function appPage(user: User) {
  const displayName = user.name ?? user.email;
  const avatar = user.avatarUrl
    ? `<img class="avatar" src="${escapeHtml(user.avatarUrl)}" alt="">`
    : `<span class="avatar"></span>`;

  const promptPlaceholder = `These videos are episodes of a robot folding a box. Provide timestamps for all of the following events:
- Pick Box
- Assemble Base
- Fold Left Side
- Fold Right Side
- Lower Lid
- Stack Box
Also keep track of "Retry" and "Fail" attributes for each step. "Retry" means multiple attempts were required for the step. "Fail" means the step was not completed successfully.`;

  return page(
    "Shotwell App",
    `<main class="app-main">
      <section class="app-header">
        <div class="account-box">
          <div class="identity">${avatar}<span>${escapeHtml(displayName)}</span></div>
          <form method="post" action="/logout">
            <button class="secondary compact" type="submit">Sign out</button>
          </form>
        </div>
      </section>

      <section class="workspace-grid">
        <section class="panel upload-panel">
          <div class="panel-heading">
            <div>
              <h2>Upload robot episodes for labeling.</h2>
            </div>
          </div>
          <form>
            <label>
              Robot Episodes
              <span class="field-description">Upload one or more robot episodes as MCAP files or raw video files, e.g. mp4 files of head cam.</span>
              <input type="file" name="videos" accept="video/*,.mcap" multiple>
            </label>
            <label>
              Prompt
              <textarea name="prompt" placeholder="${escapeHtml(promptPlaceholder)}"></textarea>
            </label>
            <button type="button">Create upload</button>
          </form>
        </section>

        <section class="panel status-panel">
          <div class="panel-heading">
            <div>
              <h2>Previous Uploads</h2>
            </div>
            <a class="button secondary compact" href="#">Refresh</a>
          </div>
          <div class="job-list">
            <article class="job-row">
              <div class="job-title">
                <strong>box_folding_headcam_batch_01</strong>
                <span>10 videos, 2.4 hours total</span>
              </div>
              <div class="job-meta">Prompt: box folding event timestamps with retry attributes</div>
              <a class="button compact" href="#">Download results</a>
            </article>
          </div>
        </section>
      </section>
    </main>`
  );
}

export function messagePage(title: string, message: string, href = "/", label = "Continue") {
  return page(
    title,
    `<main class="message-main">
      <section class="panel message-panel">
        <span class="eyebrow">Shotwell</span>
        <h1>${escapeHtml(title)}</h1>
        <p>${escapeHtml(message)}</p>
        <div>
          <a class="button" href="${escapeHtml(href)}">${escapeHtml(label)}</a>
        </div>
      </section>
    </main>`
  );
}
