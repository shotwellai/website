import type { User } from "../auth/store.js";
import type { AdminDashboard, AdminUploadSummary, AdminUserSummary } from "../admin/store.js";
import { config } from "../config.js";
import type { ResultAnnotation, ResultEpisode, UploadResult } from "../uploads/results.js";
import type { UploadRecord } from "../uploads/store.js";

type ResultUploadFile = {
  id: string;
  uploadId: string;
  originalName: string;
  objectName: string;
  contentType: string;
  sizeBytes: number;
};

type ResultPageUpload = {
  id: string;
  sourceUrl?: string;
  resultFileName?: string;
  files: ResultUploadFile[];
};

type ResultPageOptions = {
  backHref?: string;
  backLabel?: string;
  jsonHref?: string;
  mediaHref?: (file: ResultUploadFile, upload: ResultPageUpload) => string;
  eyebrow?: string;
};

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
  const logoUrl = publicAsset("brand/assets/SVG/shotwell-mark-white.svg");
  const logoLightUrl = publicAsset("brand/assets/SVG/shotwell-mark-black.svg");
  const navHtml = options.navHtml ?? `<a class="button secondary" href="${escapeHtml(publicSiteUrl)}">Main Site</a>`;

  return `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>${escapeHtml(title)}</title>
  <link rel="icon" type="image/svg+xml" href="${publicAsset("favicon.svg")}">
  <link rel="icon" type="image/png" sizes="32x32" href="${publicAsset("favicon-32.png")}">
  <link rel="icon" type="image/png" sizes="16x16" href="${publicAsset("favicon-16.png")}">
  <link rel="apple-touch-icon" sizes="180x180" href="${publicAsset("apple-touch-icon.png")}">
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Fraunces:opsz,wght@9..144,300;9..144,400&family=Outfit:wght@300;400;500&family=IBM+Plex+Mono:wght@400;500&display=swap" rel="stylesheet">
  <script src="/theme.js"></script>
  <style>
    :root {
      color-scheme: dark;
      --color-bg: #0B0B0F;
      --color-bg-alt: #101016;
      --color-surface: #111116;
      --color-panel: #16161C;
      --color-panel-soft: #1A1A21;
      --color-text: #F5F2EB;
      --color-text-light: rgba(245, 242, 235, 0.72);
      --color-muted: rgba(245, 242, 235, 0.45);
      --color-line: rgba(245, 242, 235, 0.18);
      --color-line-strong: rgba(245, 242, 235, 0.4);
      --color-cta: #EFB702;
      --color-cta-text: #0C0C0E;
      --color-accent: #EFB702;
      --color-accent-soft: rgba(239, 183, 2, 0.14);
      --color-warn: #E38FB4;
      --color-warn-bg: rgba(175, 50, 97, 0.16);
      --font-body: 'Outfit', system-ui, sans-serif;
      --font-display: 'Fraunces', Georgia, serif;
      --font-mono: 'IBM Plex Mono', ui-monospace, SFMono-Regular, Menlo, monospace;
      --color-hover: rgba(245, 242, 235, 0.07);
      --color-hover-soft: rgba(245, 242, 235, 0.05);
      --color-hover-strong: rgba(245, 242, 235, 0.08);
      --color-focus-bg: #1F1F27;
      --color-topbar: rgba(17, 17, 22, 0.92);
      --ease-out: cubic-bezier(0.16, 1, 0.3, 1);
      --nav-height: 72px;
      --max-width: 1280px;
      --radius: 0;
    }

    [data-theme="light"] {
      color-scheme: light;
      --color-bg: #EDE9DF;
      --color-bg-alt: #EAE5D9;
      --color-surface: #F5F2EB;
      --color-panel: #F0EDE4;
      --color-panel-soft: #FAF8F2;
      --color-text: #22222A;
      --color-text-light: rgba(34, 34, 42, 0.75);
      --color-muted: rgba(34, 34, 42, 0.5);
      --color-line: rgba(34, 34, 42, 0.16);
      --color-line-strong: rgba(34, 34, 42, 0.42);
      --color-accent: #9C7A00;
      --color-accent-soft: rgba(239, 183, 2, 0.2);
      --color-warn: #97264F;
      --color-warn-bg: rgba(175, 50, 97, 0.1);
      --color-hover: rgba(34, 34, 42, 0.06);
      --color-hover-soft: rgba(34, 34, 42, 0.04);
      --color-hover-strong: rgba(34, 34, 42, 0.07);
      --color-focus-bg: #FFF7E0;
      --color-topbar: rgba(245, 242, 235, 0.92);
    }

    [data-theme="light"] .noise {
      mix-blend-mode: multiply;
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
      mix-blend-mode: screen;
    }

    .shell {
      width: min(1220px, calc(100% - 34px));
      min-height: calc(100vh - 68px);
      margin: 34px auto 80px;
      display: grid;
      grid-template-rows: auto 1fr;
      background: var(--color-surface);
      border: 1px solid var(--color-line);
      box-shadow: none;
    }

    .topbar {
      position: sticky;
      top: 0;
      z-index: 20;
      height: var(--nav-height);
      border-bottom: 1px solid var(--color-line);
      background: var(--color-topbar);
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
      font-family: var(--font-display);
      font-size: 1.45rem;
      font-weight: 400;
      line-height: 1;
      letter-spacing: -0.01em;
    }

    .brand-mark {
      width: 30px;
      height: 42px;
      object-fit: contain;
      display: block;
      border-radius: var(--radius);
    }

    .brand-dot {
      color: var(--color-text);
    }

    .topbar .button,
    .topbar button {
      min-height: 0;
      padding: 8px 10px;
      background: none;
      border: none;
      color: var(--color-text-light);
    }

    .topbar .button:hover,
    .topbar button:hover {
      background: none;
      border: none;
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
      font-weight: 300;
      line-height: 1.02;
      letter-spacing: -0.018em;
    }

    h1 em,
    h2 em {
      font-style: italic;
    }

    h2 {
      font-family: var(--font-display);
      font-size: clamp(2rem, 3.5vw, 3.3rem);
      font-weight: 300;
      line-height: 1.06;
      letter-spacing: -0.015em;
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
      font-family: var(--font-mono);
      font-size: 0.66rem;
      font-weight: 500;
      letter-spacing: 0.18em;
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
      font-size: 0.875rem;
      font-weight: 500;
      letter-spacing: 0.01em;
      line-height: 1;
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
      background: #F8C41C;
      border-color: #F8C41C;
      color: var(--color-cta-text);
    }

    .button.secondary,
    button.secondary {
      background: transparent;
      color: var(--color-text);
      border-color: var(--color-line-strong);
    }

    .button.secondary:hover,
    button.secondary:hover {
      background: var(--color-hover);
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
      color: var(--color-muted);
      background: var(--color-hover);
      border-color: var(--color-line);
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
      font-family: var(--font-mono);
      font-size: 0.62rem;
      font-weight: 400;
      letter-spacing: 0.2em;
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
      font-size: 0.9rem;
      font-weight: 400;
      letter-spacing: 0;
      text-transform: none;
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
      background: var(--color-focus-bg);
      border-color: var(--color-accent);
      box-shadow: 0 0 0 2px rgba(239, 183, 2, 0.28);
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
      border: 1px solid rgba(239, 183, 2, 0.3);
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
      display: inline-flex;
      align-items: center;
      justify-content: center;
      color: var(--color-cta-text);
      font-family: "Outfit", sans-serif;
      font-size: 0.9375rem;
      font-weight: 500;
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

    .upload-tabs {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
    }

    .upload-tab {
      min-height: 44px;
      border: 0;
      background: transparent;
      color: var(--color-text);
      box-shadow: none;
      transition: none;
    }

    .upload-tab + .upload-tab {
      border-left: 1px solid var(--color-line);
    }

    .upload-tab[aria-selected="true"] {
      background: var(--color-cta);
      color: var(--color-cta-text);
    }

    .upload-tab:hover,
    .upload-tab:focus-visible {
      background: transparent;
      border-color: transparent;
      color: var(--color-text);
      transform: none;
      box-shadow: none;
    }

    .upload-tab[aria-selected="true"]:hover,
    .upload-tab[aria-selected="true"]:focus-visible {
      background: var(--color-cta);
      color: var(--color-cta-text);
    }

    .upload-tab-panel {
      display: grid;
      gap: 12px;
    }

    .upload-tab-panel[hidden] {
      display: none;
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
      background: var(--color-hover-soft);
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

    .admin-main {
      display: grid;
      gap: 24px;
    }

    .admin-hero {
      display: grid;
      gap: 10px;
    }

    .admin-hero p {
      max-width: 52rem;
      color: var(--color-text-light);
      font-size: 1.02rem;
    }

    .admin-metrics {
      display: grid;
      grid-template-columns: repeat(5, minmax(0, 1fr));
      gap: 12px;
    }

    .admin-metric {
      min-height: 118px;
      display: grid;
      align-content: space-between;
      gap: 18px;
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
      padding: 16px;
    }

    .admin-metric strong {
      color: var(--color-text);
      font-size: clamp(2rem, 4vw, 3.2rem);
      font-weight: 800;
      letter-spacing: -0.05em;
      line-height: 0.9;
    }

    .admin-metric span {
      color: var(--color-muted);
      font-size: 0.68rem;
      font-weight: 800;
      letter-spacing: 0.14em;
      line-height: 1;
      text-transform: uppercase;
    }

    .admin-section {
      display: grid;
      gap: 14px;
    }

    .admin-section-header {
      display: flex;
      align-items: end;
      justify-content: space-between;
      gap: 16px;
    }

    .admin-section-header h2 {
      font-family: var(--font-body);
      font-size: 1.35rem;
      font-weight: 800;
      letter-spacing: -0.02em;
      line-height: 1;
    }

    .admin-section-header p {
      color: var(--color-muted);
      font-size: 0.82rem;
      font-weight: 500;
    }

    .admin-table-wrap {
      overflow-x: auto;
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
    }

    .admin-table {
      width: 100%;
      min-width: 860px;
      border-collapse: collapse;
      font-size: 0.86rem;
    }

    .admin-table th,
    .admin-table td {
      padding: 12px 14px;
      border-bottom: 1px solid rgba(61, 52, 40, 0.35);
      text-align: left;
      vertical-align: top;
    }

    .admin-table th {
      color: var(--color-muted);
      font-size: 0.66rem;
      font-weight: 800;
      letter-spacing: 0.14em;
      line-height: 1;
      text-transform: uppercase;
      white-space: nowrap;
    }

    .admin-table tr:last-child td {
      border-bottom: 0;
    }

    .admin-table tbody tr {
      transition: background 0.16s var(--ease-out);
    }

    .admin-table tbody tr:hover,
    .admin-table tbody tr.is-selected {
      background: rgba(155, 51, 40, 0.08);
    }

    .admin-table .numeric {
      text-align: right;
      font-variant-numeric: tabular-nums;
      white-space: nowrap;
    }

    .admin-cell-link {
      min-height: 42px;
      display: block;
      margin: -12px -14px;
      padding: 12px 14px;
      color: inherit;
      text-decoration: none;
    }

    .admin-primary {
      display: grid;
      gap: 2px;
      min-width: 210px;
    }

    .admin-primary strong {
      overflow-wrap: anywhere;
      line-height: 1.2;
    }

    .admin-primary span,
    .admin-muted {
      color: var(--color-muted);
      font-size: 0.78rem;
      font-weight: 300;
    }

    .admin-upload-list {
      display: grid;
      gap: 12px;
    }

    .admin-upload-card {
      display: grid;
      gap: 16px;
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
      padding: 16px;
    }

    .admin-upload-head {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 16px;
      align-items: start;
    }

    .admin-upload-title {
      display: grid;
      gap: 7px;
      min-width: 0;
    }

    .admin-upload-title h3 {
      overflow-wrap: anywhere;
      font-size: 1.06rem;
      line-height: 1.25;
    }

    .admin-upload-meta {
      display: flex;
      flex-wrap: wrap;
      gap: 8px 14px;
      color: var(--color-muted);
      font-size: 0.78rem;
    }

    .admin-upload-status {
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-end;
      gap: 8px;
    }

    .admin-upload-grid {
      display: grid;
      grid-template-columns: minmax(0, 0.9fr) minmax(0, 1.1fr);
      gap: 14px;
    }

    .admin-upload-block {
      display: grid;
      gap: 8px;
    }

    .admin-upload-block h4 {
      margin: 0;
      color: var(--color-muted);
      font-size: 0.66rem;
      font-weight: 800;
      letter-spacing: 0.14em;
      line-height: 1;
      text-transform: uppercase;
    }

    .admin-file-list {
      display: grid;
      gap: 8px;
      margin: 0;
      padding: 0;
      list-style: none;
    }

    .admin-file-list li,
    .admin-source,
    .admin-prompt {
      border: 1px solid rgba(61, 52, 40, 0.42);
      background: var(--color-panel);
      padding: 10px;
    }

    .admin-file-list li {
      display: grid;
      gap: 9px;
    }

    .admin-file-name,
    .admin-source {
      overflow-wrap: anywhere;
    }

    .admin-file-meta {
      color: var(--color-muted);
      font-size: 0.76rem;
      font-weight: 300;
    }

    .admin-prompt {
      max-height: 220px;
      overflow: auto;
      white-space: pre-wrap;
      color: var(--color-text-light);
      font-size: 0.86rem;
      line-height: 1.45;
    }

    .admin-object-path {
      display: block;
      margin-top: 4px;
      color: var(--color-muted);
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
      font-size: 0.68rem;
      line-height: 1.35;
      overflow-wrap: anywhere;
    }

    .admin-media-preview {
      display: block;
      width: 100%;
      max-height: 320px;
      border: 1px solid var(--color-line);
      background: #15120f;
    }

    img.admin-media-preview {
      height: auto;
      object-fit: contain;
      background: var(--color-panel-soft);
    }

    audio.admin-media-preview {
      max-height: none;
      padding: 8px;
      background: var(--color-panel-soft);
    }

    .admin-file-actions {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }

    .admin-filter-note {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
      padding: 12px;
    }

    .admin-filter-note p {
      color: var(--color-text);
      font-size: 0.9rem;
    }

    .progress {
      height: 6px;
      margin-top: 8px;
      overflow: hidden;
      border-radius: var(--radius);
      background: var(--color-hover-strong);
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

    .result-main {
      display: grid;
      gap: 22px;
    }

    .result-heading {
      display: grid;
      gap: 10px;
    }

    .result-heading p {
      max-width: 56rem;
      color: var(--color-text-light);
      font-size: 1rem;
    }

    .result-layout {
      display: grid;
      grid-template-columns: minmax(190px, 0.24fr) minmax(0, 1fr);
      gap: 18px;
      align-items: start;
    }

    .result-layout.single {
      grid-template-columns: 1fr;
    }

    .result-sidebar {
      position: sticky;
      top: calc(var(--nav-height) + 18px);
      display: grid;
      gap: 8px;
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
      padding: 12px;
    }

    .result-sidebar h2 {
      font-family: var(--font-body);
      font-size: 0.72rem;
      font-weight: 800;
      letter-spacing: 0.14em;
      line-height: 1;
      text-transform: uppercase;
    }

    .result-nav-item {
      min-height: 0;
      width: 100%;
      justify-content: flex-start;
      border-color: transparent;
      padding: 10px;
      background: transparent;
      color: var(--color-text);
      font-size: 0.78rem;
      line-height: 1.25;
      text-align: left;
      text-transform: none;
      overflow-wrap: anywhere;
    }

    .result-nav-item[aria-current="true"] {
      border-color: var(--color-line);
      background: var(--color-cta);
      color: var(--color-cta-text);
    }

    .result-viewer {
      display: grid;
      gap: 16px;
    }

    .result-panel {
      display: grid;
      gap: 18px;
    }

    .result-panel[hidden] {
      display: none;
    }

    .result-panel h2 {
      font-family: var(--font-body);
      font-size: 1.35rem;
      font-weight: 800;
      letter-spacing: -0.02em;
      line-height: 1.1;
      overflow-wrap: anywhere;
    }

    .result-media-frame {
      border: 1px solid var(--color-line);
      background: #15120f;
    }

    .result-media-frame video,
    .result-media-frame img {
      display: block;
      width: 100%;
      max-height: min(66vh, 680px);
      object-fit: contain;
      background: #15120f;
    }

    .result-missing-media {
      display: grid;
      gap: 8px;
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
      padding: 16px;
    }

    .annotation-timeline {
      position: relative;
      min-height: 104px;
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
      padding: 46px 12px 28px;
    }

    .result-annotations {
      display: grid;
      gap: 12px;
    }

    .annotation-track {
      position: relative;
      height: 18px;
      border: 1px solid var(--color-line);
      background: var(--color-panel);
    }

    .annotation-segment {
      position: absolute;
      top: -1px;
      bottom: -1px;
      left: var(--start);
      width: var(--width);
      min-width: 2px;
      border: 1px solid var(--color-line);
      background: var(--color-accent-soft);
    }

    .annotation-segment:nth-child(even) {
      background: var(--color-bg-alt);
    }

    .annotation-chip {
      position: absolute;
      left: var(--center);
      top: 11px;
      max-width: 180px;
      transform: translateX(-50%);
      border: 1px solid var(--color-line);
      background: var(--color-panel);
      padding: 5px 7px;
      color: var(--color-text);
      font-size: 0.66rem;
      font-weight: 800;
      letter-spacing: 0.04em;
      line-height: 1.15;
      text-align: center;
      text-transform: uppercase;
      overflow-wrap: anywhere;
    }

    .annotation-time-axis {
      position: absolute;
      right: 12px;
      bottom: 6px;
      color: var(--color-muted);
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
      font-size: 0.72rem;
    }

    .annotation-list {
      display: grid;
      gap: 8px;
      margin: 0;
      padding: 0;
      list-style: none;
    }

    .annotation-list li {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 12px;
      align-items: center;
      border: 1px solid var(--color-line);
      background: var(--color-panel-soft);
      padding: 10px 12px;
    }

    .annotation-list strong {
      overflow-wrap: anywhere;
    }

    .annotation-list span {
      color: var(--color-muted);
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
      font-size: 0.78rem;
      white-space: nowrap;
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

	      .admin-metrics {
	        grid-template-columns: repeat(2, minmax(0, 1fr));
	      }

	      .admin-upload-grid {
	        grid-template-columns: 1fr;
	      }

	      .result-layout {
	        grid-template-columns: 1fr;
	      }

	      .result-sidebar {
	        position: static;
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
        width: 100%;
        min-height: 100vh;
        margin: 0;
        border: 0;
        box-shadow: none;
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
        padding: 26px 16px 40px;
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

	      .admin-metrics {
	        grid-template-columns: 1fr;
	      }

	      .admin-section-header,
	      .admin-upload-head {
	        grid-template-columns: 1fr;
	        display: grid;
	        align-items: start;
	      }

	      .admin-upload-status {
	        justify-content: flex-start;
	      }

	      .annotation-list li {
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
          <img class="brand-mark" src="${escapeHtml(logoUrl)}"
            data-mark-dark="${escapeHtml(logoUrl)}"
            data-mark-light="${escapeHtml(logoLightUrl)}" alt="Shotwell logo">
          <span>Shotwell<span class="brand-dot">.</span></span>
        </a>
	        <nav><button class="button secondary compact" id="themeToggle" type="button">Light</button>${navHtml}</nav>
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

function uploadTitle(upload: Pick<ResultPageUpload, "sourceUrl" | "files">) {
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
    ? `<a class="button compact" href="/uploads/${escapeHtml(upload.id)}/result">View Result</a>
      <a class="button secondary compact" href="/uploads/${escapeHtml(upload.id)}/results">Download JSON</a>`
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

function resultFileContentType(file: { originalName: string; contentType: string }) {
  const contentType = file.contentType.trim().toLowerCase();
  const name = file.originalName.toLowerCase();
  if (name.endsWith(".mov") && (contentType === "video/quicktime" || contentType === "application/octet-stream")) {
    return "video/mp4";
  }

  if (contentType && contentType !== "application/octet-stream") {
    return contentType;
  }

  if (name.endsWith(".mp4") || name.endsWith(".m4v") || name.endsWith(".mov")) return "video/mp4";
  if (name.endsWith(".webm")) return "video/webm";
  if (name.endsWith(".png")) return "image/png";
  if (name.endsWith(".jpg") || name.endsWith(".jpeg")) return "image/jpeg";
  if (name.endsWith(".gif")) return "image/gif";
  if (name.endsWith(".webp")) return "image/webp";
  return contentType || "application/octet-stream";
}

function resultEpisodeId(index: number) {
  return `episode-${index + 1}`;
}

function filenameBasename(value: string) {
  return value.split(/[\\/]/).pop() ?? value;
}

function findEpisodeFile(upload: ResultPageUpload, episode: ResultEpisode) {
  const filename = filenameBasename(episode.filename).toLowerCase();
  return upload.files.find((file) => file.originalName === episode.filename)
    ?? upload.files.find((file) => file.originalName.toLowerCase() === filename);
}

function secondsLabel(seconds: number) {
  const safeSeconds = Math.max(0, seconds);
  const minutes = Math.floor(safeSeconds / 60);
  const wholeSeconds = Math.floor(safeSeconds % 60);
  const fraction = safeSeconds % 1;
  const suffix = fraction > 0 ? `.${Math.round(fraction * 10)}` : "";
  return `${minutes}:${String(wholeSeconds).padStart(2, "0")}${suffix}`;
}

function annotationDuration(annotations: ResultAnnotation[]) {
  return Math.max(1, ...annotations.map((annotation) => annotation.end_time));
}

function percent(value: number) {
  return `${Math.max(0, Math.min(100, value)).toFixed(4)}%`;
}

function renderResultMedia(upload: ResultPageUpload, episode: ResultEpisode, options: ResultPageOptions = {}) {
  const file = findEpisodeFile(upload, episode);
  if (!file) {
    return `<div class="result-missing-media">
      <strong>Media file not found.</strong>
      <p>The result references <code>${escapeHtml(episode.filename)}</code>, but that filename is not attached to this upload.</p>
    </div>`;
  }

  const href = options.mediaHref?.(file, upload) ?? `/uploads/${encodeURIComponent(upload.id)}/files/${encodeURIComponent(file.id)}`;
  const escapedHref = escapeHtml(href);
  const contentType = resultFileContentType(file);
  if (contentType.startsWith("video/")) {
    return `<div class="result-media-frame">
      <video controls preload="metadata">
        <source src="${escapedHref}" type="${escapeHtml(contentType)}">
      </video>
    </div>`;
  }

  if (contentType.startsWith("image/")) {
    return `<div class="result-media-frame">
      <img src="${escapedHref}" alt="${escapeHtml(file.originalName)}" loading="lazy">
    </div>`;
  }

  return `<div class="result-missing-media">
    <strong>Preview unavailable.</strong>
    <p>${escapeHtml(file.originalName)}</p>
    <a class="button secondary compact" href="${escapedHref}" target="_blank" rel="noopener">Open file</a>
  </div>`;
}

function renderAnnotationTimeline(annotations: ResultAnnotation[]) {
  if (annotations.length === 0) {
    return `<div class="empty-row">No annotations in this episode.</div>`;
  }

  const duration = annotationDuration(annotations);
  const segments = annotations.map((annotation) => {
    const start = annotation.start_time / duration * 100;
    const width = Math.max(0.25, (annotation.end_time - annotation.start_time) / duration * 100);
    const center = (annotation.start_time + annotation.end_time) / 2 / duration * 100;

    return `<span class="annotation-segment" style="--start:${percent(start)};--width:${percent(width)}"></span>
      <span class="annotation-chip" style="--center:${percent(center)}">${escapeHtml(annotation.label)}</span>`;
  }).join("");

  return `<div class="annotation-timeline">
    <div class="annotation-track">${segments}</div>
    <div class="annotation-time-axis">0:00 - ${secondsLabel(duration)}</div>
  </div>`;
}

function renderAnnotationList(annotations: ResultAnnotation[]) {
  if (annotations.length === 0) {
    return "";
  }

  return `<ul class="annotation-list">
    ${annotations.map((annotation) => `<li>
      <strong>${escapeHtml(annotation.label)}</strong>
      <span>${secondsLabel(annotation.start_time)} - ${secondsLabel(annotation.end_time)}</span>
    </li>`).join("")}
  </ul>`;
}

function renderResultEpisode(upload: ResultPageUpload, episode: ResultEpisode, index: number, options: ResultPageOptions = {}) {
  const id = resultEpisodeId(index);
  const hidden = index === 0 ? "" : " hidden";

  return `<section class="result-panel" id="${id}" data-result-panel="${id}"${hidden}>
    <h2>${escapeHtml(episode.filename)}</h2>
    ${renderResultMedia(upload, episode, options)}
    <section class="result-annotations">
      <div class="admin-section-header">
        <h2>Annotations</h2>
        <p>${formatCount(episode.annotations.length)} labels</p>
      </div>
      ${renderAnnotationTimeline(episode.annotations)}
      ${renderAnnotationList(episode.annotations)}
    </section>
  </section>`;
}

function isShotwellAdminEmail(email: string) {
  const [, domain = ""] = email.trim().toLowerCase().split("@");
  return domain === "shotwell.ai";
}

function renderAccountNav(user: User, linksHtml = "") {
  const displayName = user.name ?? user.email;
  const avatar = user.avatarUrl
    ? `<img class="avatar" src="${escapeHtml(user.avatarUrl)}" alt="">`
    : `<span class="avatar">${escapeHtml(displayName.charAt(0).toUpperCase())}</span>`;

  return `${linksHtml}
        <div class="account-box">
          <div class="identity">${avatar}<span>${escapeHtml(displayName)}</span></div>
          <form method="post" action="/logout">
            <button class="secondary compact" type="submit">Sign out</button>
          </form>
        </div>`;
}

function formatDate(date: Date | undefined) {
  if (!date) {
    return "-";
  }

  return `${date.toISOString().replace("T", " ").slice(0, 16)} UTC`;
}

function formatCount(value: number) {
  return new Intl.NumberFormat("en-US").format(value);
}

function formatBytes(bytes: number) {
  if (!Number.isFinite(bytes) || bytes <= 0) {
    return "0 B";
  }

  const units = ["B", "KB", "MB", "GB", "TB"];
  let value = bytes;
  let unitIndex = 0;

  while (value >= 1024 && unitIndex < units.length - 1) {
    value /= 1024;
    unitIndex += 1;
  }

  const digits = value >= 10 || unitIndex === 0 ? 0 : 1;
  return `${value.toFixed(digits)} ${units[unitIndex]}`;
}

function renderAdminMetric(label: string, value: string | number) {
  return `<article class="admin-metric">
    <strong>${escapeHtml(String(value))}</strong>
    <span>${escapeHtml(label)}</span>
  </article>`;
}

function adminUserUploadsHref(userId: string) {
  return `/admin/users/${encodeURIComponent(userId)}#uploads`;
}

function renderLinkedAdminCell(user: AdminUserSummary, html: string, className = "") {
  const href = adminUserUploadsHref(user.id);
  const tdClass = className ? ` class="${className}"` : "";
  return `<td${tdClass}><a class="admin-cell-link" href="${escapeHtml(href)}">${html}</a></td>`;
}

function renderAdminUsers(users: AdminUserSummary[], selectedUserId?: string) {
  const rows = users.map((user) => {
    const uploadDetail = user.urlUploadCount > 0
      ? `${formatCount(user.uploadCount)} (${formatCount(user.urlUploadCount)} links)`
      : formatCount(user.uploadCount);
    const selectedClass = user.id === selectedUserId ? ` class="is-selected"` : "";

    return `<tr${selectedClass}>
      ${renderLinkedAdminCell(user, `
        <div class="admin-primary">
          <strong>${escapeHtml(user.email)}</strong>
          <span>${escapeHtml(user.name ?? "No name")}</span>
        </div>`)}
      ${renderLinkedAdminCell(user, escapeHtml(user.provider))}
      ${renderLinkedAdminCell(user, formatDate(user.createdAt))}
      ${renderLinkedAdminCell(user, formatDate(user.lastSessionAt))}
      ${renderLinkedAdminCell(user, formatCount(user.sessionCount), "numeric")}
      ${renderLinkedAdminCell(user, escapeHtml(uploadDetail), "numeric")}
      ${renderLinkedAdminCell(user, formatCount(user.fileCount), "numeric")}
      ${renderLinkedAdminCell(user, formatDate(user.lastUploadAt))}
    </tr>`;
  }).join("");

  return `<div class="admin-table-wrap">
    <table class="admin-table">
      <thead>
        <tr>
          <th>Account</th>
          <th>Provider</th>
          <th>Created</th>
          <th>Last session</th>
          <th class="numeric">Sessions</th>
          <th class="numeric">Uploads</th>
          <th class="numeric">Files</th>
          <th>Last upload</th>
        </tr>
      </thead>
      <tbody>${rows || `<tr><td colspan="8">No accounts yet.</td></tr>`}</tbody>
    </table>
  </div>`;
}

function adminUploadTitle(upload: AdminUploadSummary) {
  if (upload.sourceUrl) {
    return upload.sourceUrl;
  }

  const firstFile = upload.files[0]?.originalName ?? "Untitled upload";
  const otherCount = Math.max(0, upload.files.length - 1);

  if (otherCount === 0) {
    return firstFile;
  }

  return `${firstFile} & ${otherCount} ${otherCount === 1 ? "other" : "others"}`;
}

function adminFileContentType(file: AdminUploadSummary["files"][number]) {
  const contentType = file.contentType.trim().toLowerCase();
  const name = file.originalName.toLowerCase();
  if (name.endsWith(".mov") && (contentType === "video/quicktime" || contentType === "application/octet-stream")) {
    return "video/mp4";
  }

  if (contentType && contentType !== "application/octet-stream") {
    return contentType;
  }

  if (name.endsWith(".mp4") || name.endsWith(".m4v")) return "video/mp4";
  if (name.endsWith(".mov")) return "video/mp4";
  if (name.endsWith(".webm")) return "video/webm";
  if (name.endsWith(".mp3")) return "audio/mpeg";
  if (name.endsWith(".wav")) return "audio/wav";
  if (name.endsWith(".m4a")) return "audio/mp4";
  if (name.endsWith(".png")) return "image/png";
  if (name.endsWith(".jpg") || name.endsWith(".jpeg")) return "image/jpeg";
  if (name.endsWith(".gif")) return "image/gif";
  if (name.endsWith(".webp")) return "image/webp";
  return contentType || "application/octet-stream";
}

function adminFileHref(file: AdminUploadSummary["files"][number]) {
  return `/admin/uploads/${encodeURIComponent(file.uploadId)}/files/${encodeURIComponent(file.id)}`;
}

function renderAdminFilePreview(file: AdminUploadSummary["files"][number]) {
  const href = adminFileHref(file);
  const contentType = adminFileContentType(file);
  const escapedHref = escapeHtml(href);
  const escapedName = escapeHtml(file.originalName);

  if (contentType.startsWith("video/")) {
    return `<video class="admin-media-preview" controls preload="metadata">
      <source src="${escapedHref}" type="${escapeHtml(contentType)}">
    </video>`;
  }

  if (contentType.startsWith("image/")) {
    return `<img class="admin-media-preview" src="${escapedHref}" alt="${escapedName}" loading="lazy">`;
  }

  if (contentType.startsWith("audio/")) {
    return `<audio class="admin-media-preview" controls preload="metadata" src="${escapedHref}"></audio>`;
  }

  return "";
}

function renderAdminUploadFiles(upload: AdminUploadSummary) {
  if (upload.files.length === 0) {
    return upload.sourceUrl
      ? `<div class="admin-source">${escapeHtml(upload.sourceUrl)}</div>`
      : `<div class="admin-source">No files or source URL recorded.</div>`;
  }

  const files = upload.files.map((file) => `<li>
    ${renderAdminFilePreview(file)}
    <div class="admin-file-name">${escapeHtml(file.originalName)}</div>
    <div class="admin-file-meta">${escapeHtml(file.contentType)} - ${formatBytes(file.sizeBytes)}</div>
    <code class="admin-object-path">${escapeHtml(file.objectName)}</code>
    <div class="admin-file-actions">
      <a class="button secondary compact" href="${escapeHtml(adminFileHref(file))}" target="_blank" rel="noopener">Open file</a>
    </div>
  </li>`).join("");

  return `<ul class="admin-file-list">${files}</ul>`;
}

function renderAdminUploads(uploads: AdminUploadSummary[]) {
  if (uploads.length === 0) {
    return `<div class="empty-row">No uploads yet.</div>`;
  }

  return `<div class="admin-upload-list">${uploads.map((upload) => {
    const statusClass = upload.status === "completed" ? "status-pill ready" : "status-pill";
    const userLabel = upload.userName
      ? `${upload.userName} <${upload.userEmail}>`
      : upload.userEmail;
    const prompt = upload.prompt.trim() || "No prompt provided.";
    const resultMeta = upload.resultObjectName
      ? `<span>Result: ${escapeHtml(upload.resultFileName ?? upload.resultObjectName)}</span>`
      : "";
    const resultActions = upload.resultObjectName
      ? `<a class="button compact" href="/admin/uploads/${escapeHtml(upload.id)}/result">View result</a>
        <a class="button secondary compact" href="/admin/uploads/${escapeHtml(upload.id)}/results">Download JSON</a>`
      : "";

    return `<article class="admin-upload-card">
      <div class="admin-upload-head">
        <div class="admin-upload-title">
          <h3>${escapeHtml(adminUploadTitle(upload))}</h3>
          <div class="admin-upload-meta">
            <span>${escapeHtml(userLabel)}</span>
            <span>${formatDate(upload.createdAt)}</span>
            <span>${formatCount(upload.fileCount)} files</span>
            <span>${formatBytes(upload.totalSizeBytes)}</span>
            ${resultMeta}
          </div>
        </div>
        <div class="admin-upload-status">
          ${resultActions}
          <span class="${statusClass}">${escapeHtml(upload.status)}</span>
        </div>
      </div>
      <div class="admin-upload-grid">
        <section class="admin-upload-block">
          <h4>Input</h4>
          ${renderAdminUploadFiles(upload)}
        </section>
        <section class="admin-upload-block">
          <h4>Prompt</h4>
          <div class="admin-prompt">${escapeHtml(prompt)}</div>
        </section>
      </div>
    </article>`;
  }).join("")}</div>`;
}

export function adminPage(user: User, dashboard: AdminDashboard) {
  const publicSiteUrl = config.publicSiteUrl.toString();
  const selectedUser = dashboard.selectedUser;
  const selectedUserLabel = selectedUser
    ? selectedUser.name
      ? `${selectedUser.name} <${selectedUser.email}>`
      : selectedUser.email
    : "";
  const filterNote = selectedUser
    ? `<div class="admin-filter-note">
        <p>Showing uploads from <strong>${escapeHtml(selectedUserLabel)}</strong>.</p>
        <a class="button secondary compact" href="/admin#uploads">View all uploads</a>
      </div>`
    : "";
  const uploadHeading = selectedUser ? "Uploads for account" : "Uploads";
  const uploadCountText = selectedUser
    ? `${formatCount(dashboard.uploads.length)} upload batches from this account`
    : `Latest ${formatCount(dashboard.uploads.length)} upload batches`;
  const navHtml = renderAccountNav(
    user,
    `<a class="button secondary" href="/">App</a>
        <a class="button secondary" href="${escapeHtml(publicSiteUrl)}">Main Site</a>`
  );

  return page(
    "Shotwell Admin",
    `<main class="admin-main">
      <section class="admin-hero">
        <span class="eyebrow">Admin</span>
        <h1>Platform activity.</h1>
        <p>Accounts, sessions, upload batches, source links, uploaded files, prompts, and result status across Shotwell's authenticated app.</p>
      </section>

      <section class="admin-metrics" aria-label="Platform totals">
        ${renderAdminMetric("Accounts", formatCount(dashboard.stats.userCount))}
        ${renderAdminMetric("Uploads", formatCount(dashboard.stats.uploadCount))}
        ${renderAdminMetric("Pending", formatCount(dashboard.stats.pendingUploadCount))}
        ${renderAdminMetric("Completed", formatCount(dashboard.stats.completedUploadCount))}
        ${renderAdminMetric("Files stored", `${formatCount(dashboard.stats.fileCount)} / ${formatBytes(dashboard.stats.totalSizeBytes)}`)}
      </section>

      <section class="admin-section">
        <div class="admin-section-header">
          <h2>Accounts</h2>
          <p>${formatCount(dashboard.users.length)} total account records</p>
        </div>
        ${renderAdminUsers(dashboard.users, selectedUser?.id)}
      </section>

      <section class="admin-section" id="uploads">
        <div class="admin-section-header">
          <h2>${escapeHtml(uploadHeading)}</h2>
          <p>${escapeHtml(uploadCountText)}</p>
        </div>
        ${filterNote}
        ${renderAdminUploads(dashboard.uploads)}
      </section>
    </main>`,
    { navHtml }
  );
}

export function resultPage(user: User, upload: ResultPageUpload, result: UploadResult, options: ResultPageOptions = {}) {
  const publicSiteUrl = config.publicSiteUrl.toString();
  const backHref = options.backHref ?? "/";
  const backLabel = options.backLabel ?? "App";
  const jsonHref = options.jsonHref ?? `/uploads/${encodeURIComponent(upload.id)}/results`;
  const navHtml = renderAccountNav(
    user,
    `<a class="button secondary" href="${escapeHtml(backHref)}">${escapeHtml(backLabel)}</a>
        <a class="button secondary" href="${escapeHtml(jsonHref)}">Download JSON</a>
        <a class="button secondary" href="${escapeHtml(publicSiteUrl)}">Main Site</a>`
  );
  const hasSidebar = result.episodes.length > 1;
  const sidebar = hasSidebar
    ? `<aside class="result-sidebar">
        <h2>Episodes</h2>
        ${result.episodes.map((episode, index) => {
          const id = resultEpisodeId(index);
          const current = index === 0 ? ` aria-current="true"` : "";
          return `<button class="result-nav-item" type="button" data-result-nav="${id}"${current}>${escapeHtml(episode.filename)}</button>`;
        }).join("")}
      </aside>`
    : "";
  const panels = result.episodes.map((episode, index) => renderResultEpisode(upload, episode, index, options)).join("");

  return page(
    "Shotwell Result",
    `<main class="result-main">
      <section class="result-heading">
        <span class="eyebrow">${escapeHtml(options.eyebrow ?? "Result")}</span>
        <h1>Shotwell annotations.</h1>
        <p>${escapeHtml(uploadTitle(upload))}</p>
      </section>

      <section class="result-layout${hasSidebar ? "" : " single"}" data-result-viewer>
        ${sidebar}
        <div class="result-viewer">${panels}</div>
      </section>
    </main>
    <script src="/app.js" defer></script>`,
    { navHtml }
  );
}

export function appPage(user: User, uploads: UploadRecord[] = []) {
  const publicSiteUrl = config.publicSiteUrl.toString();
  const uploadRows = uploads.length > 0
    ? uploads.map(renderUploadRow).join("")
    : `<div class="empty-row">No uploads yet.</div>`;
  const adminLink = isShotwellAdminEmail(user.email)
    ? `<a class="button secondary" href="/admin">Admin</a>`
    : "";

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
              <h2>Upload or provide a link to robot episodes for labeling.</h2>
            </div>
          </div>
          <form data-upload-form>
            <div class="upload-tabs" role="tablist" aria-label="Episode source">
              <button class="upload-tab" type="button" role="tab" id="upload-tab-files" aria-controls="upload-panel-files" aria-selected="true" data-upload-tab="files">Upload Files</button>
              <button class="upload-tab" type="button" role="tab" id="upload-tab-url" aria-controls="upload-panel-url" aria-selected="false" data-upload-tab="url">Provide Link</button>
            </div>
            <div class="upload-tab-panel" role="tabpanel" id="upload-panel-files" aria-labelledby="upload-tab-files" data-upload-panel="files">
              <label>
                Robot Episode Files
                <span class="field-description">Upload one or more files for each robot episode, including MCAP files, raw videos, logs, images, or any supporting artifacts.</span>
                <input type="file" name="videos" multiple data-upload-files>
              </label>
            </div>
            <div class="upload-tab-panel" role="tabpanel" id="upload-panel-url" aria-labelledby="upload-tab-url" data-upload-panel="url" hidden>
              <label>
                Episode URL
                <span class="field-description">Enter URL to episodes, e.g. HuggingFace LeRobot dataset, S3 bucket link, etc.</span>
                <input type="text" name="sourceUrl" placeholder="https://huggingface.co/datasets/... or s3://bucket/path" data-upload-source-url disabled>
              </label>
            </div>
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
        ${adminLink}
        ${renderAccountNav(user)}`
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
