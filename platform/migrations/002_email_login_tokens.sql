create table if not exists shotwell_auth_email_login_tokens (
  token text primary key,
  email text not null,
  return_to text not null,
  created_at timestamptz not null default now(),
  expires_at timestamptz not null
);

create index if not exists shotwell_auth_email_login_tokens_email_idx
  on shotwell_auth_email_login_tokens(email);

create index if not exists shotwell_auth_email_login_tokens_expires_at_idx
  on shotwell_auth_email_login_tokens(expires_at);
