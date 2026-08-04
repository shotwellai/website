import { config } from "../config.js";
import { escapeHtml } from "../http/render.js";
import type { User } from "../auth/store.js";
import type { UploadRecord } from "../uploads/store.js";
import { emailDeliveryConfigured, sendAdminNotification } from "./sender.js";

const maxPromptLength = 2_000;
const maxListedFiles = 12;

function truncate(value: string, maxLength: number) {
  if (value.length <= maxLength) {
    return value;
  }

  return `${value.slice(0, maxLength - 3)}...`;
}

function formatDate(value: Date) {
  return value.toISOString();
}

function formatBytes(value: number) {
  if (!Number.isFinite(value) || value <= 0) {
    return "0 B";
  }

  const units = ["B", "KB", "MB", "GB", "TB"];
  let size = value;
  let unitIndex = 0;
  while (size >= 1024 && unitIndex < units.length - 1) {
    size /= 1024;
    unitIndex += 1;
  }

  return `${size >= 10 || unitIndex === 0 ? size.toFixed(0) : size.toFixed(1)} ${units[unitIndex]}`;
}

function displayName(user: User) {
  return user.name ? `${user.name} <${user.email}>` : user.email;
}

function htmlRow(label: string, value: string) {
  return `<tr>
    <th style="padding:8px 12px;text-align:left;vertical-align:top;border-bottom:1px solid #e5e7eb;color:#374151;width:150px;">${escapeHtml(label)}</th>
    <td style="padding:8px 12px;vertical-align:top;border-bottom:1px solid #e5e7eb;color:#111827;">${value}</td>
  </tr>`;
}

async function deliverAdminNotification(input: { subject: string; text: string; html: string }) {
  if (!emailDeliveryConfigured()) {
    console.warn("Admin notification skipped because email delivery is not configured.");
    return;
  }

  try {
    await sendAdminNotification(input);
  } catch (error) {
    console.error("Admin notification failed:", error);
  }
}

export async function notifyNewSignup(user: User) {
  const subject = `New Shotwell signup: ${user.email}`;
  const rows = [
    htmlRow("User", escapeHtml(displayName(user))),
    htmlRow("Email", escapeHtml(user.email)),
    htmlRow("Provider", escapeHtml(user.provider)),
    htmlRow("User ID", escapeHtml(user.id)),
    htmlRow("Created", escapeHtml(formatDate(user.createdAt)))
  ].join("");

  await deliverAdminNotification({
    subject,
    text: [
      "New Shotwell signup",
      "",
      `User: ${displayName(user)}`,
      `Email: ${user.email}`,
      `Provider: ${user.provider}`,
      `User ID: ${user.id}`,
      `Created: ${formatDate(user.createdAt)}`
    ].join("\n"),
    html: `<!doctype html>
<html lang="en">
<body style="margin:0;padding:24px;background:#f4f1e8;color:#111827;font-family:Arial,sans-serif;">
  <table role="presentation" width="100%" cellspacing="0" cellpadding="0" style="max-width:680px;margin:0 auto;background:#ffffff;border:1px solid #d1d5db;">
    <tr>
      <td style="padding:24px;">
        <p style="margin:0 0 8px;color:#9b3328;font-size:12px;font-weight:700;letter-spacing:.12em;text-transform:uppercase;">Shotwell platform</p>
        <h1 style="margin:0 0 18px;font-size:24px;line-height:1.2;">New signup</h1>
        <table role="presentation" width="100%" cellspacing="0" cellpadding="0" style="border-collapse:collapse;font-size:14px;">${rows}</table>
      </td>
    </tr>
  </table>
</body>
</html>`
  });
}

export async function notifyUploadSubmitted(user: User, upload: UploadRecord) {
  const totalBytes = upload.files.reduce((sum, file) => sum + file.sizeBytes, 0);
  const listedFiles = upload.files.slice(0, maxListedFiles);
  const extraFileCount = Math.max(upload.files.length - listedFiles.length, 0);
  const fileSummary = upload.files.length > 0
    ? `${upload.files.length} file${upload.files.length === 1 ? "" : "s"} (${formatBytes(totalBytes)})`
    : "No files";
  const source = upload.sourceUrl ? `URL: ${upload.sourceUrl}` : fileSummary;
  const prompt = upload.prompt ? truncate(upload.prompt, maxPromptLength) : "(none)";
  const appUploadsUrl = new URL("/#previous-uploads", config.appBaseUrl).toString();
  const fileLines = listedFiles.map((file) => `- ${file.originalName} (${formatBytes(file.sizeBytes)}, ${file.contentType})`);
  if (extraFileCount > 0) {
    fileLines.push(`- ...and ${extraFileCount} more`);
  }

  const rows = [
    htmlRow("User", escapeHtml(displayName(user))),
    htmlRow("Upload ID", escapeHtml(upload.id)),
    htmlRow("Created", escapeHtml(formatDate(upload.createdAt))),
    htmlRow("Source", escapeHtml(source)),
    htmlRow("Files", escapeHtml(fileSummary)),
    htmlRow("Upload prefix", escapeHtml(upload.uploadPrefix)),
    upload.sourceUrl ? htmlRow("Source URL", escapeHtml(upload.sourceUrl)) : "",
    htmlRow("Prompt", `<pre style="margin:0;white-space:pre-wrap;font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:13px;line-height:1.45;">${escapeHtml(prompt)}</pre>`),
    fileLines.length > 0
      ? htmlRow("File list", `<pre style="margin:0;white-space:pre-wrap;font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:13px;line-height:1.45;">${escapeHtml(fileLines.join("\n"))}</pre>`)
      : "",
    htmlRow("App", `<a href="${escapeHtml(appUploadsUrl)}" style="color:#9b3328;">${escapeHtml(appUploadsUrl)}</a>`)
  ].filter(Boolean).join("");

  await deliverAdminNotification({
    subject: `New Shotwell upload from ${user.email}`,
    text: [
      "New Shotwell upload",
      "",
      `User: ${displayName(user)}`,
      `Upload ID: ${upload.id}`,
      `Created: ${formatDate(upload.createdAt)}`,
      `Source: ${source}`,
      `Files: ${fileSummary}`,
      `Upload prefix: ${upload.uploadPrefix}`,
      upload.sourceUrl ? `Source URL: ${upload.sourceUrl}` : undefined,
      "",
      "Prompt:",
      prompt,
      "",
      fileLines.length > 0 ? ["Files:", ...fileLines].join("\n") : undefined,
      "",
      `App: ${appUploadsUrl}`
    ].filter((line): line is string => typeof line === "string").join("\n"),
    html: `<!doctype html>
<html lang="en">
<body style="margin:0;padding:24px;background:#f4f1e8;color:#111827;font-family:Arial,sans-serif;">
  <table role="presentation" width="100%" cellspacing="0" cellpadding="0" style="max-width:760px;margin:0 auto;background:#ffffff;border:1px solid #d1d5db;">
    <tr>
      <td style="padding:24px;">
        <p style="margin:0 0 8px;color:#9b3328;font-size:12px;font-weight:700;letter-spacing:.12em;text-transform:uppercase;">Shotwell platform</p>
        <h1 style="margin:0 0 18px;font-size:24px;line-height:1.2;">New upload submitted</h1>
        <table role="presentation" width="100%" cellspacing="0" cellpadding="0" style="border-collapse:collapse;font-size:14px;">${rows}</table>
      </td>
    </tr>
  </table>
</body>
</html>`
  });
}
