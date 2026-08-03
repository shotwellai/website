import type { User } from "../auth/store.js";
import { config } from "../config.js";
import type { UploadRecord } from "../uploads/store.js";

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

export function page(title: string, body: string, options: { navHtml?: string } = {}) {
  const publicSiteUrl = config.publicSiteUrl.toString();
  const logoUrl = publicAsset("shotwell-logo.png");
  const navHtml = options.navHtml ?? `<a class="button secondary" href="${escapeHtml(publicSiteUrl)}">Main Site</a>`;

  return `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>${escapeHtml(title)}</title>
  <style>
    :root {
      color-scheme: light;
      --color-bg: #d7c7a9;
      --color-bg-alt: #ddc596;
      --color-surface: #efe5ce;
      --color-panel: #efe5ce;
      --color-panel-soft: #f6ecd5;
      --color-text: #201c17;
      --color-text-light: #4a4034;
      --color-muted: #6d5e4b;
      --color-line: #3d3428;
      --color-line-strong: #3d3428;
      --color-cta: #201c17;
      --color-cta-text: #efe5ce;
      --color-accent: #9b3328;
      --color-accent-soft: #ead8b7;
      --color-warn: #9b3328;
      --color-warn-bg: #f6ecd5;
      --font-body: Arial, Helvetica, sans-serif;
      --font-display: Georgia, serif;
      --ease-out: cubic-bezier(0.16, 1, 0.3, 1);
      --nav-height: 72px;
      --max-width: 1280px;
      --radius: 0;
    }

    *,
    *::before,
    *::after {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background: var(--color-bg);
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
      opacity: 0.04;
      mix-blend-mode: multiply;
    }

    .shell {
      width: min(1220px, calc(100% - 34px));
      min-height: calc(100vh - 68px);
      margin: 34px auto 80px;
      display: grid;
      grid-template-rows: auto 1fr;
      background: var(--color-surface);
      border: 1px solid var(--color-line);
      box-shadow: 10px 12px 0 rgba(32, 28, 23, 0.18);
    }

    .topbar {
      position: sticky;
      top: 0;
      z-index: 20;
      height: var(--nav-height);
      border-bottom: 1px solid var(--color-line);
      background: rgba(239, 229, 206, 0.96);
      backdrop-filter: blur(12px);
      -webkit-backdrop-filter: blur(12px);
    }

	    .topbar-inner {
	      max-width: none;
	      height: 100%;
	      margin: 0 auto;
	      padding: 0 22px;
      display: flex;
      align-items: center;
      justify-content: space-between;
	      gap: 24px;
	    }

	    .topbar nav {
	      min-width: 0;
	      display: flex;
	      align-items: center;
	      justify-content: flex-end;
	      gap: 12px;
	    }

    .brand {
      display: inline-flex;
      align-items: center;
      gap: 12px;
      color: var(--color-text);
      font-family: var(--font-body);
      font-size: 1.5rem;
      font-weight: 800;
      line-height: 1;
      letter-spacing: -0.03em;
    }

    .brand-mark {
      width: 42px;
      height: 42px;
      object-fit: contain;
      display: block;
      border-radius: var(--radius);
    }

    .brand-dot {
      color: var(--color-text);
    }

    main {
      width: 100%;
      margin: 0 auto;
      padding: clamp(34px, 5vw, 64px);
    }

    h1,
    h2,
    h3,
    p {
      margin: 0;
    }

    h1 {
      font-family: var(--font-display);
      font-size: clamp(3rem, 6vw, 5.25rem);
      font-weight: 400;
      line-height: 0.96;
      letter-spacing: -0.045em;
    }

    h1 em,
    h2 em {
      font-style: italic;
    }

    h2 {
      font-family: var(--font-display);
      font-size: clamp(2rem, 3.5vw, 3.3rem);
      font-weight: 400;
      line-height: 1.02;
      letter-spacing: -0.04em;
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
      border: 1px solid var(--color-accent);
      border-radius: var(--radius);
      padding: 7px 9px;
      color: var(--color-accent);
      font-size: 0.72rem;
      font-weight: 800;
      letter-spacing: 0.14em;
      line-height: 1;
      text-transform: uppercase;
    }

    .button,
    button {
      min-height: 52px;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 10px;
      border: 1px solid var(--color-cta);
      border-radius: var(--radius);
      padding: 0 18px;
      background: var(--color-cta);
      color: var(--color-cta-text);
      font: inherit;
      font-size: 0.82rem;
      font-weight: 700;
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
      background: var(--color-accent);
      border-color: var(--color-accent);
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
      background: rgba(32, 28, 23, 0.06);
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
      color: rgba(32, 28, 23, 0.44);
      background: rgba(32, 28, 23, 0.06);
      border-color: rgba(32, 28, 23, 0.18);
      pointer-events: none;
    }

    .panel {
      background: var(--color-panel);
      border: 1px solid var(--color-line);
      border-radius: var(--radius);
      padding: 24px;
      box-shadow: none;
    }

    .panel-heading {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 18px;
      margin-bottom: 18px;
    }

    .panel h2 {
      font-size: clamp(1.8rem, 2.8vw, 2.55rem);
      line-height: 1.02;
    }

    .panel-kicker {
      color: var(--color-muted);
      font-size: 0.72rem;
      font-weight: 800;
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
      border: 1px solid var(--color-line);
      border-radius: var(--radius);
      background: var(--color-panel-soft);
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
      background: #fff4dd;
      border-color: var(--color-accent);
      box-shadow: 0 0 0 2px rgba(155, 51, 40, 0.16);
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
      border-radius: var(--radius);
      padding: 12px;
      background: var(--color-warn-bg);
      color: var(--color-warn);
      font-size: 0.9rem;
      line-height: 1.45;
    }

    .notice[hidden] {
      display: none;
    }

    .notice.neutral {
      background: var(--color-panel-soft);
      border-color: var(--color-line);
      color: var(--color-text);
    }

	    .inline-link {
	      color: var(--color-text);
	      text-decoration: underline;
	      text-decoration-thickness: 1px;
	      text-underline-offset: 0.18em;
	    }

	    .zdr-link {
	      color: var(--color-text);
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
      background: var(--color-panel);
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
      border-radius: var(--radius);
      background: var(--color-panel-soft);
      padding: 18px;
    }

    .mini-step-number {
      color: var(--color-accent);
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

	    .app-intro {
	      max-width: 820px;
	    }

    .app-intro p {
      color: var(--color-text);
      font-size: 1.08rem;
	      line-height: 1.55;
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
	      border-radius: var(--radius);
	      padding: 8px;
	      background: var(--color-panel-soft);
	    }

	    .account-box form {
	      display: block;
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
      border-radius: var(--radius);
      object-fit: cover;
      background: var(--color-cta);
      border: 1px solid var(--color-line);
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

    .upload-choice-divider {
      margin: 2px 0;
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
      border-radius: var(--radius);
      background: var(--color-panel-soft);
      padding: 18px;
    }

    .metric-value {
      color: var(--color-text);
      font-family: var(--font-display);
      font-size: 2.9rem;
      font-weight: 400;
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
	      grid-template-columns: minmax(0, 1fr) auto;
	      align-items: start;
	      gap: 16px;
	      border: 1px solid var(--color-line);
	      border-radius: var(--radius);
	      background: var(--color-panel-soft);
	      padding: 14px;
	    }

	    .job-content {
	      min-width: 0;
	      display: grid;
	      gap: 10px;
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

	    .job-prompt {
	      display: grid;
	      gap: 5px;
	      border-left: 2px solid var(--color-accent);
	      padding-left: 10px;
	    }

	    .job-prompt-label {
	      color: var(--color-muted);
	      font-size: 0.68rem;
	      font-weight: 600;
	      letter-spacing: 0.12em;
	      line-height: 1;
	      text-transform: uppercase;
	    }

	    .job-prompt p {
	      white-space: pre-wrap;
	      overflow-wrap: anywhere;
	      font-size: 0.9rem;
	      line-height: 1.45;
	    }

	    .job-actions {
	      display: flex;
	      align-items: flex-start;
	      justify-content: flex-end;
	      gap: 8px;
	    }

	    .job-row .button,
	    .job-row button {
	      width: max-content;
	    }

	    .prompt-editor {
	      grid-column: 1 / -1;
	    }

	    .prompt-editor summary {
	      width: max-content;
	      min-height: 34px;
	      display: inline-flex;
	      align-items: center;
	      justify-content: center;
	      border: 1px solid var(--color-cta);
	      border-radius: var(--radius);
	      padding: 0 12px;
	      color: var(--color-cta);
	      font-size: 0.68rem;
	      font-weight: 500;
	      letter-spacing: 0.04em;
	      line-height: 1;
	      text-transform: uppercase;
	      cursor: pointer;
	    }

	    .prompt-editor summary::-webkit-details-marker {
	      display: none;
	    }

	    .prompt-editor[open] summary {
	      margin-bottom: 12px;
	    }

	    .prompt-editor form {
	      gap: 10px;
	    }

	    .prompt-editor textarea {
	      min-height: 190px;
	    }

	    .prompt-editor-actions {
	      display: flex;
	      justify-content: flex-end;
	    }

	    .prompt-editor-actions button {
	      min-height: 36px;
	    }

    .empty-row {
      border: 1px solid var(--color-line);
      border-radius: var(--radius);
      background: var(--color-panel-soft);
      padding: 16px;
      color: var(--color-muted);
      font-weight: 300;
    }

    .status-pill {
      width: max-content;
      border: 1px solid var(--color-line);
      border-radius: var(--radius);
      padding: 5px 9px;
      background: rgba(32, 28, 23, 0.04);
      color: var(--color-text-light);
      font-size: 0.72rem;
      font-weight: 600;
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }

    .status-pill.ready {
      border-color: var(--color-accent);
      background: var(--color-accent-soft);
      color: var(--color-cta);
    }

    .progress {
      height: 6px;
      margin-top: 8px;
      overflow: hidden;
      border-radius: var(--radius);
      background: rgba(42, 42, 50, 0.08);
    }

    .progress span {
      display: block;
      height: 100%;
      border-radius: var(--radius);
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
      background: var(--color-panel);
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
      background: var(--color-panel);
    }

    .policy-panel h1 {
      font-size: 3.3rem;
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
	      .workspace-grid {
	        grid-template-columns: 1fr;
	      }

	      .account-box {
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

	      .job-actions {
	        justify-content: flex-start;
	      }
	    }

    @media (max-width: 680px) {
      .shell {
        width: min(100% - 24px, 1220px);
        min-height: calc(100vh - 36px);
        margin: 18px auto 48px;
        box-shadow: 6px 8px 0 rgba(32, 28, 23, 0.18);
      }

      .topbar {
        height: auto;
        min-height: var(--nav-height);
      }

      .topbar-inner {
        min-height: var(--nav-height);
        padding: 12px 18px;
        flex-wrap: wrap;
        justify-content: center;
        gap: 10px;
      }

      .brand {
        font-size: 1.5rem;
      }

      .brand-mark {
        width: 38px;
        height: 38px;
      }

	      .topbar nav .button {
	        flex: 1 1 120px;
	        min-height: 38px;
	        padding: 0 12px;
	        font-size: 0.68rem;
	      }

	      .topbar nav {
	        width: 100%;
	        flex-wrap: wrap;
	        justify-content: center;
	        gap: 8px;
	      }

	      .topbar .account-box {
	        width: 100%;
	        max-width: 360px;
	      }

	      .topbar .account-box .button,
	      .topbar .account-box button {
	        flex: 0 0 auto;
	      }

      main {
        width: 100%;
        padding: 26px 18px 40px;
      }

      .auth-main,
      .auth-main-single {
        grid-template-columns: minmax(0, 1fr);
        padding-top: 26px;
        padding-bottom: 40px;
      }

      .panel {
        padding: 18px;
      }

      h1 {
        font-size: 2.35rem;
      }

      h2 {
        font-size: 1.55rem;
      }

      .policy-panel h1 {
        font-size: 2.35rem;
      }

      .prompt-tools,
      .metric-grid {
        grid-template-columns: 1fr;
      }

	      .account-box {
	        gap: 8px;
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
	        <nav>${navHtml}</nav>
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
        <p>Login with your company email. Get 10 hours of robot video annotated for free, with 24 hour turnaround time. <a class="zdr-link" href="/privacy">Zero Data Retention.</a></p>
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

function uploadTitle(upload: UploadRecord) {
  const firstFile = upload.files[0]?.originalName ?? "Untitled upload";
  const otherCount = Math.max(0, upload.files.length - 1);

  if (upload.files.length === 0 && upload.sourceUrl) {
    return upload.sourceUrl;
  }

  if (otherCount === 0) {
    return firstFile;
  }

  return `${firstFile} & ${otherCount} ${otherCount === 1 ? "other" : "others"}`;
}

function promptPreview(prompt: string) {
  const compact = prompt.trim().replace(/\s+/g, " ");

  if (!compact) {
    return "No prompt provided.";
  }

  if (compact.length <= 100) {
    return compact;
  }

  return `${compact.slice(0, 97).trimEnd()}...`;
}

function renderUploadRow(upload: UploadRecord) {
  const resultButton = upload.status === "completed" && upload.resultObjectName
    ? `<a class="button compact" href="/uploads/${escapeHtml(upload.id)}/results">Download Results</a>`
    : `<span class="status-pill">Pending</span>`;
  const prompt = promptPreview(upload.prompt);
  const editPrompt = upload.status === "pending"
    ? `<details class="prompt-editor">
      <summary>Edit Prompt</summary>
      <form method="post" action="/uploads/${escapeHtml(upload.id)}/prompt">
        <label>
          Prompt
          <textarea name="prompt">${escapeHtml(upload.prompt)}</textarea>
        </label>
        <div class="prompt-editor-actions">
          <button class="compact" type="submit">Save prompt</button>
        </div>
      </form>
    </details>`
    : "";

  return `<article class="job-row">
    <div class="job-content">
      <div class="job-title">
        <strong>${escapeHtml(uploadTitle(upload))}</strong>
      </div>
      <div class="job-prompt">
        <span class="job-prompt-label">Prompt</span>
        <p>${escapeHtml(prompt)}</p>
      </div>
    </div>
    <div class="job-actions">${resultButton}</div>
    ${editPrompt}
  </article>`;
}

export function appPage(user: User, uploads: UploadRecord[] = []) {
  const publicSiteUrl = config.publicSiteUrl.toString();
  const displayName = user.name ?? user.email;
  const avatar = user.avatarUrl
    ? `<img class="avatar" src="${escapeHtml(user.avatarUrl)}" alt="">`
    : `<span class="avatar"></span>`;
  const uploadRows = uploads.length > 0
    ? uploads.map(renderUploadRow).join("")
    : `<div class="empty-row">No uploads yet.</div>`;

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
      <section class="app-intro">
        <p>Try out our service for free with up to 10 hours of robot data. We'll annotate it and send it back within 24 hours. <a class="zdr-link" href="/privacy">Zero Data Retention.</a></p>
      </section>

      <section class="workspace-grid">
        <section class="panel upload-panel">
          <div class="panel-heading">
            <div>
              <h2>Upload robot episodes for labeling.</h2>
            </div>
          </div>
          <form data-upload-form>
            <label>
              Robot Episode Files
              <span class="field-description">Upload one or more files for each robot episode, including MCAP files, raw videos, logs, images, or any supporting artifacts.</span>
              <input type="file" name="videos" multiple data-upload-files>
            </label>
            <div class="divider upload-choice-divider">or</div>
            <label>
              Episode URL
              <span class="field-description">Enter URL to episodes, e.g. HuggingFace LeRobot dataset, S3 bucket link, etc.</span>
              <input type="text" name="sourceUrl" placeholder="https://huggingface.co/datasets/... or s3://bucket/path" data-upload-source-url>
            </label>
            <label>
              Prompt
              <textarea name="prompt" placeholder="${escapeHtml(promptPlaceholder)}" data-upload-prompt></textarea>
            </label>
            <button type="submit" data-upload-submit>Create upload</button>
            <div class="notice neutral" data-upload-message hidden></div>
          </form>
        </section>

        <section class="panel status-panel" id="previous-uploads">
          <div class="panel-heading">
            <div>
              <h2>Previous Uploads</h2>
            </div>
            <a class="button secondary compact" href="/#previous-uploads">Refresh</a>
          </div>
          <div class="job-list">${uploadRows}</div>
        </section>
      </section>
    </main>
    <script src="/app.js" defer></script>`,
    {
      navHtml: `<a class="button secondary" href="${escapeHtml(publicSiteUrl)}">Main Site</a>
        <div class="account-box">
          <div class="identity">${avatar}<span>${escapeHtml(displayName)}</span></div>
          <form method="post" action="/logout">
            <button class="secondary compact" type="submit">Sign out</button>
          </form>
        </div>`
    }
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
