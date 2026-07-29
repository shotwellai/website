import { randomBytes, randomUUID } from "node:crypto";

import { config } from "../config.js";
import { PostgresAuthStore } from "./postgres-store.js";

export type SessionKind = "auth" | "app";

export type User = {
  id: string;
  email: string;
  name?: string;
  avatarUrl?: string;
  provider: "email" | "google";
  providerSubject?: string;
  createdAt: Date;
  updatedAt: Date;
};

export type AuthSession = {
  id: string;
  userId: string;
  kind: SessionKind;
  createdAt: Date;
  expiresAt: Date;
};

export type OAuthState = {
  state: string;
  nonce: string;
  returnTo: string;
  expiresAt: Date;
};

export type Handoff = {
  code: string;
  userId: string;
  returnTo: string;
  expiresAt: Date;
};

export type EmailLoginToken = {
  token: string;
  email: string;
  returnTo: string;
  expiresAt: Date;
};

export type UpsertUserInput = {
  email: string;
  name?: string;
  avatarUrl?: string;
  provider: User["provider"];
  providerSubject?: string;
};

const token = () => randomBytes(32).toString("base64url");
const now = () => new Date();
const minutesFromNow = (minutes: number) => new Date(Date.now() + minutes * 60_000);
const daysFromNow = (days: number) => new Date(Date.now() + days * 24 * 60 * 60_000);

export interface AuthStore {
  upsertUser(input: UpsertUserInput): Promise<User>;
  createSession(userId: string, kind: SessionKind): Promise<AuthSession>;
  getSession(sessionId: string | undefined, kind: SessionKind): Promise<AuthSession | null>;
  deleteSession(sessionId: string | undefined): Promise<void>;
  getUser(userId: string): Promise<User | null>;
  createOAuthState(returnTo: string): Promise<OAuthState>;
  consumeOAuthState(state: string | undefined): Promise<OAuthState | null>;
  createEmailLoginToken(email: string, returnTo: string): Promise<EmailLoginToken>;
  consumeEmailLoginToken(token: string | undefined): Promise<EmailLoginToken | null>;
  createHandoff(userId: string, returnTo: string): Promise<Handoff>;
  consumeHandoff(code: string | undefined): Promise<Handoff | null>;
}

export class MemoryAuthStore implements AuthStore {
  private readonly usersById = new Map<string, User>();
  private readonly userIdByEmail = new Map<string, string>();
  private readonly sessions = new Map<string, AuthSession>();
  private readonly oauthStates = new Map<string, OAuthState>();
  private readonly emailLoginTokens = new Map<string, EmailLoginToken>();
  private readonly handoffs = new Map<string, Handoff>();

  async upsertUser(input: UpsertUserInput): Promise<User> {
    const email = input.email.trim().toLowerCase();
    const existingId = this.userIdByEmail.get(email);

    if (existingId) {
      const existing = this.usersById.get(existingId);
      if (!existing) {
        throw new Error("User index is inconsistent.");
      }

      const updated: User = {
        ...existing,
        name: input.name ?? existing.name,
        avatarUrl: input.avatarUrl ?? existing.avatarUrl,
        provider: input.provider,
        providerSubject: input.providerSubject ?? existing.providerSubject,
        updatedAt: now()
      };

      this.usersById.set(updated.id, updated);
      return updated;
    }

    const user: User = {
      id: randomUUID(),
      email,
      name: input.name,
      avatarUrl: input.avatarUrl,
      provider: input.provider,
      providerSubject: input.providerSubject,
      createdAt: now(),
      updatedAt: now()
    };

    this.usersById.set(user.id, user);
    this.userIdByEmail.set(user.email, user.id);
    return user;
  }

  async createSession(userId: string, kind: SessionKind): Promise<AuthSession> {
    const session: AuthSession = {
      id: token(),
      userId,
      kind,
      createdAt: now(),
      expiresAt: daysFromNow(14)
    };

    this.sessions.set(session.id, session);
    return session;
  }

  async getSession(sessionId: string | undefined, kind: SessionKind): Promise<AuthSession | null> {
    if (!sessionId) {
      return null;
    }

    const session = this.sessions.get(sessionId);
    if (!session || session.kind !== kind) {
      return null;
    }

    if (session.expiresAt.getTime() <= Date.now()) {
      this.sessions.delete(sessionId);
      return null;
    }

    return session;
  }

  async deleteSession(sessionId: string | undefined): Promise<void> {
    if (sessionId) {
      this.sessions.delete(sessionId);
    }
  }

  async getUser(userId: string): Promise<User | null> {
    return this.usersById.get(userId) ?? null;
  }

  async createOAuthState(returnTo: string): Promise<OAuthState> {
    const entry: OAuthState = {
      state: token(),
      nonce: token(),
      returnTo,
      expiresAt: minutesFromNow(10)
    };

    this.oauthStates.set(entry.state, entry);
    return entry;
  }

  async consumeOAuthState(state: string | undefined): Promise<OAuthState | null> {
    if (!state) {
      return null;
    }

    const entry = this.oauthStates.get(state);
    this.oauthStates.delete(state);

    if (!entry || entry.expiresAt.getTime() <= Date.now()) {
      return null;
    }

    return entry;
  }

  async createEmailLoginToken(email: string, returnTo: string): Promise<EmailLoginToken> {
    const entry: EmailLoginToken = {
      token: token(),
      email: email.trim().toLowerCase(),
      returnTo,
      expiresAt: minutesFromNow(15)
    };

    this.emailLoginTokens.set(entry.token, entry);
    return entry;
  }

  async consumeEmailLoginToken(tokenValue: string | undefined): Promise<EmailLoginToken | null> {
    if (!tokenValue) {
      return null;
    }

    const entry = this.emailLoginTokens.get(tokenValue);
    this.emailLoginTokens.delete(tokenValue);

    if (!entry || entry.expiresAt.getTime() <= Date.now()) {
      return null;
    }

    return entry;
  }

  async createHandoff(userId: string, returnTo: string): Promise<Handoff> {
    const handoff: Handoff = {
      code: token(),
      userId,
      returnTo,
      expiresAt: minutesFromNow(5)
    };

    this.handoffs.set(handoff.code, handoff);
    return handoff;
  }

  async consumeHandoff(code: string | undefined): Promise<Handoff | null> {
    if (!code) {
      return null;
    }

    const handoff = this.handoffs.get(code);
    this.handoffs.delete(code);

    if (!handoff || handoff.expiresAt.getTime() <= Date.now()) {
      return null;
    }

    return handoff;
  }
}

function createAuthStore(): AuthStore {
  if (config.authStore === "postgres") {
    if (!config.databaseUrl) {
      throw new Error("DATABASE_URL is required when AUTH_STORE=postgres.");
    }

    return new PostgresAuthStore(config.databaseUrl);
  }

  return new MemoryAuthStore();
}

export const authStore = createAuthStore();
