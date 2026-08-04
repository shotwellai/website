import { randomBytes, randomUUID } from "node:crypto";
import { Pool } from "pg";

import { poolConfig } from "../db/pool.js";
import type {
  AuthSession,
  EmailLoginToken,
  AuthStore,
  Handoff,
  OAuthState,
  SessionKind,
  UpsertUserInput,
  UpsertUserResult,
  User
} from "./store.js";

const token = () => randomBytes(32).toString("base64url");
const minutesFromNow = (minutes: number) => new Date(Date.now() + minutes * 60_000);
const daysFromNow = (days: number) => new Date(Date.now() + days * 24 * 60 * 60_000);

type UserRow = {
  id: string;
  email: string;
  name: string | null;
  avatar_url: string | null;
  provider: User["provider"];
  provider_subject: string | null;
  created_at: Date;
  updated_at: Date;
};

type UpsertUserRow = UserRow & {
  created: boolean;
};

type SessionRow = {
  id: string;
  user_id: string;
  kind: SessionKind;
  created_at: Date;
  expires_at: Date;
};

type OAuthStateRow = {
  state: string;
  nonce: string;
  return_to: string;
  expires_at: Date;
};

type HandoffRow = {
  code: string;
  user_id: string;
  return_to: string;
  expires_at: Date;
};

type EmailLoginTokenRow = {
  token: string;
  email: string;
  return_to: string;
  expires_at: Date;
};

function mapUser(row: UserRow): User {
  return {
    id: row.id,
    email: row.email,
    name: row.name ?? undefined,
    avatarUrl: row.avatar_url ?? undefined,
    provider: row.provider,
    providerSubject: row.provider_subject ?? undefined,
    createdAt: row.created_at,
    updatedAt: row.updated_at
  };
}

function mapSession(row: SessionRow): AuthSession {
  return {
    id: row.id,
    userId: row.user_id,
    kind: row.kind,
    createdAt: row.created_at,
    expiresAt: row.expires_at
  };
}

function mapOAuthState(row: OAuthStateRow): OAuthState {
  return {
    state: row.state,
    nonce: row.nonce,
    returnTo: row.return_to,
    expiresAt: row.expires_at
  };
}

function mapHandoff(row: HandoffRow): Handoff {
  return {
    code: row.code,
    userId: row.user_id,
    returnTo: row.return_to,
    expiresAt: row.expires_at
  };
}

function mapEmailLoginToken(row: EmailLoginTokenRow): EmailLoginToken {
  return {
    token: row.token,
    email: row.email,
    returnTo: row.return_to,
    expiresAt: row.expires_at
  };
}

export class PostgresAuthStore implements AuthStore {
  private readonly pool: Pool;

  constructor(databaseUrl: string) {
    this.pool = new Pool(poolConfig(databaseUrl));
  }

  async upsertUser(input: UpsertUserInput): Promise<UpsertUserResult> {
    const email = input.email.trim().toLowerCase();
    const result = await this.pool.query<UpsertUserRow>(
      `insert into shotwell_auth_users (
        id,
        email,
        name,
        avatar_url,
        provider,
        provider_subject
      ) values ($1, $2, $3, $4, $5, $6)
      on conflict (email) do update set
        name = coalesce(excluded.name, shotwell_auth_users.name),
        avatar_url = coalesce(excluded.avatar_url, shotwell_auth_users.avatar_url),
        provider = excluded.provider,
        provider_subject = coalesce(excluded.provider_subject, shotwell_auth_users.provider_subject),
        updated_at = now()
      returning id, email, name, avatar_url, provider, provider_subject, created_at, updated_at, (xmax = 0) as created`,
      [
        randomUUID(),
        email,
        input.name ?? null,
        input.avatarUrl ?? null,
        input.provider,
        input.providerSubject ?? null
      ]
    );

    return {
      user: mapUser(result.rows[0]),
      created: result.rows[0].created
    };
  }

  async createSession(userId: string, kind: SessionKind): Promise<AuthSession> {
    const result = await this.pool.query<SessionRow>(
      `insert into shotwell_auth_sessions (id, user_id, kind, expires_at)
      values ($1, $2, $3, $4)
      returning id, user_id, kind, created_at, expires_at`,
      [token(), userId, kind, daysFromNow(14)]
    );

    return mapSession(result.rows[0]);
  }

  async getSession(sessionId: string | undefined, kind: SessionKind): Promise<AuthSession | null> {
    if (!sessionId) {
      return null;
    }

    const result = await this.pool.query<SessionRow>(
      `select id, user_id, kind, created_at, expires_at
      from shotwell_auth_sessions
      where id = $1 and kind = $2 and expires_at > now()`,
      [sessionId, kind]
    );

    return result.rows[0] ? mapSession(result.rows[0]) : null;
  }

  async deleteSession(sessionId: string | undefined): Promise<void> {
    if (!sessionId) {
      return;
    }

    await this.pool.query("delete from shotwell_auth_sessions where id = $1", [sessionId]);
  }

  async getUser(userId: string): Promise<User | null> {
    const result = await this.pool.query<UserRow>(
      `select id, email, name, avatar_url, provider, provider_subject, created_at, updated_at
      from shotwell_auth_users
      where id = $1`,
      [userId]
    );

    return result.rows[0] ? mapUser(result.rows[0]) : null;
  }

  async createOAuthState(returnTo: string): Promise<OAuthState> {
    const result = await this.pool.query<OAuthStateRow>(
      `insert into shotwell_auth_oauth_states (state, nonce, return_to, expires_at)
      values ($1, $2, $3, $4)
      returning state, nonce, return_to, expires_at`,
      [token(), token(), returnTo, minutesFromNow(10)]
    );

    return mapOAuthState(result.rows[0]);
  }

  async consumeOAuthState(state: string | undefined): Promise<OAuthState | null> {
    if (!state) {
      return null;
    }

    const result = await this.pool.query<OAuthStateRow>(
      `delete from shotwell_auth_oauth_states
      where state = $1 and expires_at > now()
      returning state, nonce, return_to, expires_at`,
      [state]
    );

    return result.rows[0] ? mapOAuthState(result.rows[0]) : null;
  }

  async createEmailLoginToken(email: string, returnTo: string): Promise<EmailLoginToken> {
    const result = await this.pool.query<EmailLoginTokenRow>(
      `insert into shotwell_auth_email_login_tokens (token, email, return_to, expires_at)
      values ($1, $2, $3, $4)
      returning token, email, return_to, expires_at`,
      [token(), email.trim().toLowerCase(), returnTo, minutesFromNow(15)]
    );

    return mapEmailLoginToken(result.rows[0]);
  }

  async consumeEmailLoginToken(tokenValue: string | undefined): Promise<EmailLoginToken | null> {
    if (!tokenValue) {
      return null;
    }

    const result = await this.pool.query<EmailLoginTokenRow>(
      `delete from shotwell_auth_email_login_tokens
      where token = $1 and expires_at > now()
      returning token, email, return_to, expires_at`,
      [tokenValue]
    );

    return result.rows[0] ? mapEmailLoginToken(result.rows[0]) : null;
  }

  async createHandoff(userId: string, returnTo: string): Promise<Handoff> {
    const result = await this.pool.query<HandoffRow>(
      `insert into shotwell_auth_handoffs (code, user_id, return_to, expires_at)
      values ($1, $2, $3, $4)
      returning code, user_id, return_to, expires_at`,
      [token(), userId, returnTo, minutesFromNow(5)]
    );

    return mapHandoff(result.rows[0]);
  }

  async consumeHandoff(code: string | undefined): Promise<Handoff | null> {
    if (!code) {
      return null;
    }

    const result = await this.pool.query<HandoffRow>(
      `delete from shotwell_auth_handoffs
      where code = $1 and expires_at > now()
      returning code, user_id, return_to, expires_at`,
      [code]
    );

    return result.rows[0] ? mapHandoff(result.rows[0]) : null;
  }
}
