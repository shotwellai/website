import { config } from "../config.js";
import { escapeHtml } from "../http/render.js";

type SendLoginLinkInput = {
  email: string;
  token: string;
};

type SendEmailInput = {
  to: string | string[];
  subject: string;
  text: string;
  html: string;
  replyTo?: string;
};

type SendAdminNotificationInput = {
  subject: string;
  text: string;
  html: string;
};

type ResendResponse = {
  id?: string;
  name?: string;
  message?: string;
};

export function emailDeliveryConfigured() {
  return Boolean(config.email.from && config.email.resendApiKey);
}

export function emailLoginUrl(token: string) {
  const url = new URL("/auth/email/complete", config.authBaseUrl);
  url.searchParams.set("token", token);
  return url.toString();
}

async function sendEmail(input: SendEmailInput) {
  if (!config.email.from || !config.email.resendApiKey) {
    throw new Error("Email delivery is not configured.");
  }

  const response = await fetch("https://api.resend.com/emails", {
    method: "POST",
    headers: {
      Authorization: `Bearer ${config.email.resendApiKey}`,
      "Content-Type": "application/json"
    },
    body: JSON.stringify({
      from: config.email.from,
      to: input.to,
      reply_to: input.replyTo ?? config.email.replyTo ?? undefined,
      subject: input.subject,
      text: input.text,
      html: input.html
    })
  });

  const body = (await response.json().catch(() => ({}))) as ResendResponse;

  if (!response.ok) {
    throw new Error(body.message ?? body.name ?? "Email provider rejected the message.");
  }

  return body.id;
}

export async function sendLoginLink(input: SendLoginLinkInput) {
  const loginUrl = emailLoginUrl(input.token);

  return sendEmail({
    to: input.email,
    subject: "Sign in to Shotwell",
    text: [
      "Sign in to Shotwell with this link:",
      "",
      loginUrl,
      "",
      "This link expires in 15 minutes. If you did not request it, you can ignore this email."
    ].join("\n"),
    html: `<!doctype html>
<html lang="en">
<body style="margin:0;padding:24px;background:#f4f6f8;color:#171717;font-family:Arial,sans-serif;">
  <table role="presentation" width="100%" cellspacing="0" cellpadding="0" style="max-width:560px;margin:0 auto;background:#ffffff;border:1px solid #d9dee7;border-radius:8px;">
    <tr>
      <td style="padding:28px;">
        <h1 style="margin:0 0 12px;font-size:24px;line-height:1.2;">Sign in to Shotwell</h1>
        <p style="margin:0 0 24px;color:#555b64;line-height:1.5;">Use this secure link to finish signing in. It expires in 15 minutes.</p>
        <p style="margin:0 0 24px;">
          <a href="${escapeHtml(loginUrl)}" style="display:inline-block;background:#131313;color:#ffffff;text-decoration:none;border-radius:6px;padding:12px 16px;font-weight:700;">Sign in</a>
        </p>
        <p style="margin:0;color:#6b7280;font-size:13px;line-height:1.5;">If the button does not work, paste this URL into your browser:<br>${escapeHtml(loginUrl)}</p>
      </td>
    </tr>
  </table>
</body>
</html>`
  });
}

export async function sendAdminNotification(input: SendAdminNotificationInput) {
  return sendEmail({
    to: config.email.adminNotificationTo,
    subject: input.subject,
    text: input.text,
    html: input.html
  });
}
