create table if not exists shotwell_auth_users (
  id uuid primary key,
  email text not null unique,
  name text,
  avatar_url text,
  provider text not null check (provider in ('email', 'google')),
  provider_subject text,
  created_at timestamptz not null default now(),
  updated_at timestamptz not null default now()
);

create table if not exists shotwell_auth_sessions (
  id text primary key,
  user_id uuid not null references shotwell_auth_users(id) on delete cascade,
  kind text not null check (kind in ('auth', 'app')),
  created_at timestamptz not null default now(),
  expires_at timestamptz not null
);

create index if not exists shotwell_auth_sessions_user_id_idx
  on shotwell_auth_sessions(user_id);

create index if not exists shotwell_auth_sessions_expires_at_idx
  on shotwell_auth_sessions(expires_at);

create table if not exists shotwell_auth_oauth_states (
  state text primary key,
  nonce text not null,
  return_to text not null,
  expires_at timestamptz not null
);

create index if not exists shotwell_auth_oauth_states_expires_at_idx
  on shotwell_auth_oauth_states(expires_at);

create table if not exists shotwell_auth_handoffs (
  code text primary key,
  user_id uuid not null references shotwell_auth_users(id) on delete cascade,
  return_to text not null,
  expires_at timestamptz not null
);

create index if not exists shotwell_auth_handoffs_expires_at_idx
  on shotwell_auth_handoffs(expires_at);
