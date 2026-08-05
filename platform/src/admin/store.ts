import { Pool } from "pg";

import { config } from "../config.js";
import { poolConfig } from "../db/pool.js";
import type { User } from "../auth/store.js";
import type { UploadStatus } from "../uploads/store.js";

export type AdminStats = {
  userCount: number;
  uploadCount: number;
  pendingUploadCount: number;
  completedUploadCount: number;
  fileCount: number;
  totalSizeBytes: number;
  urlUploadCount: number;
};

export type AdminUserSummary = {
  id: string;
  email: string;
  name?: string;
  provider: User["provider"];
  createdAt: Date;
  updatedAt: Date;
  sessionCount: number;
  lastSessionAt?: Date;
  uploadCount: number;
  fileCount: number;
  urlUploadCount: number;
  lastUploadAt?: Date;
};

export type AdminUploadFileSummary = {
  id: string;
  uploadId: string;
  originalName: string;
  objectName: string;
  contentType: string;
  sizeBytes: number;
  createdAt: Date;
};

export type AdminUploadSummary = {
  id: string;
  userId: string;
  userEmail: string;
  userName?: string;
  prompt: string;
  sourceUrl?: string;
  status: UploadStatus;
  uploadPrefix: string;
  resultObjectName?: string;
  resultFileName?: string;
  resultContentType?: string;
  filesUploadedAt?: Date;
  completedAt?: Date;
  createdAt: Date;
  updatedAt: Date;
  fileCount: number;
  totalSizeBytes: number;
  files: AdminUploadFileSummary[];
};

export type AdminDashboard = {
  stats: AdminStats;
  users: AdminUserSummary[];
  uploads: AdminUploadSummary[];
  selectedUser?: AdminUserSummary;
};

export interface AdminStore {
  getDashboard(input?: { userId?: string }): Promise<AdminDashboard>;
  getUpload(uploadId: string): Promise<AdminUploadSummary | null>;
  getUploadFile(uploadId: string, fileId: string): Promise<AdminUploadFileSummary | null>;
}

type StatsRow = {
  user_count: string | number;
  upload_count: string | number;
  pending_upload_count: string | number;
  completed_upload_count: string | number;
  file_count: string | number;
  total_size_bytes: string | number;
  url_upload_count: string | number;
};

type UserSummaryRow = {
  id: string;
  email: string;
  name: string | null;
  provider: User["provider"];
  created_at: Date;
  updated_at: Date;
  session_count: string | number;
  last_session_at: Date | null;
  upload_count: string | number;
  file_count: string | number;
  url_upload_count: string | number;
  last_upload_at: Date | null;
};

type UploadSummaryRow = {
  id: string;
  user_id: string;
  user_email: string;
  user_name: string | null;
  prompt: string;
  source_url: string | null;
  status: UploadStatus;
  upload_prefix: string;
  result_object_name: string | null;
  result_file_name: string | null;
  result_content_type: string | null;
  files_uploaded_at: Date | null;
  completed_at: Date | null;
  created_at: Date;
  updated_at: Date;
  file_count: string | number;
  total_size_bytes: string | number;
};

type UploadFileSummaryRow = {
  id: string;
  upload_id: string;
  original_name: string;
  object_name: string;
  content_type: string;
  size_bytes: string | number;
  created_at: Date;
};

function toNumber(value: string | number | null | undefined) {
  if (typeof value === "number") {
    return value;
  }

  return Number(value ?? 0);
}

function mapStats(row: StatsRow): AdminStats {
  return {
    userCount: toNumber(row.user_count),
    uploadCount: toNumber(row.upload_count),
    pendingUploadCount: toNumber(row.pending_upload_count),
    completedUploadCount: toNumber(row.completed_upload_count),
    fileCount: toNumber(row.file_count),
    totalSizeBytes: toNumber(row.total_size_bytes),
    urlUploadCount: toNumber(row.url_upload_count)
  };
}

function mapUserSummary(row: UserSummaryRow): AdminUserSummary {
  return {
    id: row.id,
    email: row.email,
    name: row.name ?? undefined,
    provider: row.provider,
    createdAt: row.created_at,
    updatedAt: row.updated_at,
    sessionCount: toNumber(row.session_count),
    lastSessionAt: row.last_session_at ?? undefined,
    uploadCount: toNumber(row.upload_count),
    fileCount: toNumber(row.file_count),
    urlUploadCount: toNumber(row.url_upload_count),
    lastUploadAt: row.last_upload_at ?? undefined
  };
}

function mapUploadSummary(row: UploadSummaryRow, files: AdminUploadFileSummary[]): AdminUploadSummary {
  return {
    id: row.id,
    userId: row.user_id,
    userEmail: row.user_email,
    userName: row.user_name ?? undefined,
    prompt: row.prompt,
    sourceUrl: row.source_url ?? undefined,
    status: row.status,
    uploadPrefix: row.upload_prefix,
    resultObjectName: row.result_object_name ?? undefined,
    resultFileName: row.result_file_name ?? undefined,
    resultContentType: row.result_content_type ?? undefined,
    filesUploadedAt: row.files_uploaded_at ?? undefined,
    completedAt: row.completed_at ?? undefined,
    createdAt: row.created_at,
    updatedAt: row.updated_at,
    fileCount: toNumber(row.file_count),
    totalSizeBytes: toNumber(row.total_size_bytes),
    files
  };
}

function mapUploadFileSummary(row: UploadFileSummaryRow): AdminUploadFileSummary {
  return {
    id: row.id,
    uploadId: row.upload_id,
    originalName: row.original_name,
    objectName: row.object_name,
    contentType: row.content_type,
    sizeBytes: toNumber(row.size_bytes),
    createdAt: row.created_at
  };
}

export class PostgresAdminStore implements AdminStore {
  private readonly pool: Pool;

  constructor(databaseUrl: string) {
    this.pool = new Pool(poolConfig(databaseUrl));
  }

  async getDashboard(input: { userId?: string } = {}): Promise<AdminDashboard> {
    const [stats, users, uploads] = await Promise.all([
      this.getStats(),
      this.getUsers(),
      this.getUploads(input.userId)
    ]);
    const selectedUser = input.userId
      ? users.find((user) => user.id === input.userId)
      : undefined;

    return { stats, users, uploads, selectedUser };
  }

  private async getStats(): Promise<AdminStats> {
    const result = await this.pool.query<StatsRow>(
      `select
        (select count(*) from shotwell_auth_users) as user_count,
        (select count(*) from shotwell_uploads) as upload_count,
        (select count(*) from shotwell_uploads where status = 'pending') as pending_upload_count,
        (select count(*) from shotwell_uploads where status = 'completed') as completed_upload_count,
        (select count(*) from shotwell_upload_files) as file_count,
        (select coalesce(sum(size_bytes), 0) from shotwell_upload_files) as total_size_bytes,
        (
          select count(*)
          from shotwell_uploads
          where source_url is not null and source_url <> ''
        ) as url_upload_count`
    );

    return mapStats(result.rows[0]);
  }

  private async getUsers(): Promise<AdminUserSummary[]> {
    const result = await this.pool.query<UserSummaryRow>(
      `select
        u.id,
        u.email,
        u.name,
        u.provider,
        u.created_at,
        u.updated_at,
        count(distinct s.id) as session_count,
        max(s.created_at) as last_session_at,
        count(distinct up.id) as upload_count,
        count(distinct f.id) as file_count,
        count(distinct up.id) filter (
          where up.source_url is not null and up.source_url <> ''
        ) as url_upload_count,
        max(up.created_at) as last_upload_at
      from shotwell_auth_users u
      left join shotwell_auth_sessions s on s.user_id = u.id
      left join shotwell_uploads up on up.user_id = u.id
      left join shotwell_upload_files f on f.upload_id = up.id
      group by u.id, u.email, u.name, u.provider, u.created_at, u.updated_at
      order by u.created_at desc`
    );

    return result.rows.map(mapUserSummary);
  }

  async getUploadFile(uploadId: string, fileId: string): Promise<AdminUploadFileSummary | null> {
    const result = await this.pool.query<UploadFileSummaryRow>(
      `select id, upload_id, original_name, object_name, content_type, size_bytes, created_at
      from shotwell_upload_files
      where upload_id = $1 and id = $2`,
      [uploadId, fileId]
    );

    return result.rows[0] ? mapUploadFileSummary(result.rows[0]) : null;
  }

  async getUpload(uploadId: string): Promise<AdminUploadSummary | null> {
    const uploadResult = await this.pool.query<UploadSummaryRow>(
      `select
        up.id,
        up.user_id,
        u.email as user_email,
        u.name as user_name,
        up.prompt,
        up.source_url,
        up.status,
        up.upload_prefix,
        up.result_object_name,
        up.result_file_name,
        up.result_content_type,
        up.files_uploaded_at,
        up.completed_at,
        up.created_at,
        up.updated_at,
        count(f.id) as file_count,
        coalesce(sum(f.size_bytes), 0) as total_size_bytes
      from shotwell_uploads up
      join shotwell_auth_users u on u.id = up.user_id
      left join shotwell_upload_files f on f.upload_id = up.id
      where up.id = $1
      group by
        up.id,
        up.user_id,
        u.email,
        u.name,
        up.prompt,
        up.source_url,
        up.status,
        up.upload_prefix,
        up.result_object_name,
        up.result_file_name,
        up.result_content_type,
        up.files_uploaded_at,
        up.completed_at,
        up.created_at,
        up.updated_at`,
      [uploadId]
    );

    return (await this.hydrateUploads(uploadResult.rows))[0] ?? null;
  }

  private async getUploads(userId?: string): Promise<AdminUploadSummary[]> {
    const whereClause = userId ? "where up.user_id = $1" : "";
    const params = userId ? [userId] : [];
    const uploadResult = await this.pool.query<UploadSummaryRow>(
      `select
        up.id,
        up.user_id,
        u.email as user_email,
        u.name as user_name,
        up.prompt,
        up.source_url,
        up.status,
        up.upload_prefix,
        up.result_object_name,
        up.result_file_name,
        up.result_content_type,
        up.files_uploaded_at,
        up.completed_at,
        up.created_at,
        up.updated_at,
        count(f.id) as file_count,
        coalesce(sum(f.size_bytes), 0) as total_size_bytes
      from shotwell_uploads up
      join shotwell_auth_users u on u.id = up.user_id
      left join shotwell_upload_files f on f.upload_id = up.id
      ${whereClause}
      group by
        up.id,
        up.user_id,
        u.email,
        u.name,
        up.prompt,
        up.source_url,
        up.status,
        up.upload_prefix,
        up.result_object_name,
        up.result_file_name,
        up.result_content_type,
        up.files_uploaded_at,
        up.completed_at,
        up.created_at,
        up.updated_at
      order by up.created_at desc
      limit 200`,
      params
    );

    return this.hydrateUploads(uploadResult.rows);
  }

  private async hydrateUploads(uploadRows: UploadSummaryRow[]): Promise<AdminUploadSummary[]> {
    if (uploadRows.length === 0) {
      return [];
    }

    const uploadIds = uploadRows.map((row) => row.id);
    const fileResult = await this.pool.query<UploadFileSummaryRow>(
      `select id, upload_id, original_name, object_name, content_type, size_bytes, created_at
      from shotwell_upload_files
      where upload_id = any($1::uuid[])
      order by created_at asc, original_name asc`,
      [uploadIds]
    );

    const filesByUploadId = new Map<string, AdminUploadFileSummary[]>();
    for (const row of fileResult.rows) {
      const files = filesByUploadId.get(row.upload_id) ?? [];
      files.push(mapUploadFileSummary(row));
      filesByUploadId.set(row.upload_id, files);
    }

    return uploadRows.map((row) => mapUploadSummary(row, filesByUploadId.get(row.id) ?? []));
  }
}

class EmptyAdminStore implements AdminStore {
  async getDashboard(): Promise<AdminDashboard> {
    return {
      stats: {
        userCount: 0,
        uploadCount: 0,
        pendingUploadCount: 0,
        completedUploadCount: 0,
        fileCount: 0,
        totalSizeBytes: 0,
        urlUploadCount: 0
      },
      users: [],
      uploads: []
    };
  }

  async getUpload(): Promise<AdminUploadSummary | null> {
    return null;
  }

  async getUploadFile(): Promise<AdminUploadFileSummary | null> {
    return null;
  }
}

function createAdminStore(): AdminStore {
  if (config.authStore === "postgres") {
    if (!config.databaseUrl) {
      throw new Error("DATABASE_URL is required when AUTH_STORE=postgres.");
    }

    return new PostgresAdminStore(config.databaseUrl);
  }

  return new EmptyAdminStore();
}

export const adminStore = createAdminStore();
