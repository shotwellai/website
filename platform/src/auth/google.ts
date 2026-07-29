import { createRemoteJWKSet, jwtVerify } from "jose";

import { config } from "../config.js";

const googleAuthUrl = new URL("https://accounts.google.com/o/oauth2/v2/auth");
const googleTokenUrl = "https://oauth2.googleapis.com/token";
const googleJwks = createRemoteJWKSet(new URL("https://www.googleapis.com/oauth2/v3/certs"));

export type GoogleProfile = {
  subject: string;
  email: string;
  emailVerified: boolean;
  name?: string;
  avatarUrl?: string;
};

export function hasGoogleConfig() {
  return Boolean(config.google.clientId && config.google.clientSecret);
}

export function googleCallbackUrl() {
  return new URL("/auth/google/callback", config.authBaseUrl).toString();
}

export function buildGoogleAuthorizationUrl(input: { state: string; nonce: string }) {
  if (!config.google.clientId) {
    throw new Error("GOOGLE_CLIENT_ID is not configured.");
  }

  const url = new URL(googleAuthUrl);
  url.searchParams.set("client_id", config.google.clientId);
  url.searchParams.set("redirect_uri", googleCallbackUrl());
  url.searchParams.set("response_type", "code");
  url.searchParams.set("scope", "openid email profile");
  url.searchParams.set("state", input.state);
  url.searchParams.set("nonce", input.nonce);
  url.searchParams.set("access_type", "offline");
  url.searchParams.set("prompt", "select_account");
  return url.toString();
}

export async function exchangeGoogleCode(input: {
  code: string;
  nonce: string;
}): Promise<GoogleProfile> {
  if (!config.google.clientId || !config.google.clientSecret) {
    throw new Error("Google OAuth is not configured.");
  }

  const response = await fetch(googleTokenUrl, {
    method: "POST",
    headers: {
      "Content-Type": "application/x-www-form-urlencoded"
    },
    body: new URLSearchParams({
      code: input.code,
      client_id: config.google.clientId,
      client_secret: config.google.clientSecret,
      redirect_uri: googleCallbackUrl(),
      grant_type: "authorization_code"
    })
  });

  const tokenResponse = (await response.json()) as {
    id_token?: string;
    error?: string;
    error_description?: string;
  };

  if (!response.ok || !tokenResponse.id_token) {
    throw new Error(tokenResponse.error_description ?? tokenResponse.error ?? "Google token exchange failed.");
  }

  const verified = await jwtVerify(tokenResponse.id_token, googleJwks, {
    audience: config.google.clientId,
    issuer: ["https://accounts.google.com", "accounts.google.com"]
  });

  const payload = verified.payload;

  if (payload.nonce !== input.nonce) {
    throw new Error("Google nonce mismatch.");
  }

  if (typeof payload.email !== "string" || !payload.email) {
    throw new Error("Google profile did not include an email address.");
  }

  if (config.google.allowedDomain) {
    const hostedDomain = typeof payload.hd === "string" ? payload.hd : "";
    if (hostedDomain !== config.google.allowedDomain) {
      throw new Error("Google account is not allowed for this workspace.");
    }
  }

  return {
    subject: String(payload.sub),
    email: payload.email,
    emailVerified: payload.email_verified === true,
    name: typeof payload.name === "string" ? payload.name : undefined,
    avatarUrl: typeof payload.picture === "string" ? payload.picture : undefined
  };
}
