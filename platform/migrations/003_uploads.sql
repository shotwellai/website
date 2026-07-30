create table if not exists shotwell_uploads (
  id uuid primary key,
  user_id uuid not null references shotwell_auth_users(id) on delete cascade,
  prompt text not null default '',
  status text not null default 'pending' check (status in ('pending', 'completed')),
  upload_prefix text not null,
  result_object_name text,
  result_file_name text,
  result_content_type text,
  files_uploaded_at timestamptz,
  completed_at timestamptz,
  created_at timestamptz not null default now(),
  updated_at timestamptz not null default now()
);

create index if not exists shotwell_uploads_user_id_created_at_idx
  on shotwell_uploads(user_id, created_at desc);

create index if not exists shotwell_uploads_status_idx
  on shotwell_uploads(status);

create table if not exists shotwell_upload_files (
  id uuid primary key,
  upload_id uuid not null references shotwell_uploads(id) on delete cascade,
  original_name text not null,
  object_name text not null unique,
  content_type text not null default 'application/octet-stream',
  size_bytes bigint not null default 0,
  created_at timestamptz not null default now()
);

create index if not exists shotwell_upload_files_upload_id_idx
  on shotwell_upload_files(upload_id);
